function verify_ik()
% Numerical consistency AND independent closed-form physical-chain checks.
m=miniarm_example_model(); MiniArmIK.validate(m); rng(4664);
worst=0;
for k=1:100
    q=m.limits(:,1)+rand(4,1).*(m.limits(:,2)-m.limits(:,1));
    J=MiniArmIK.jacobian(m,q,'partial');
    worst=max(worst,norm(J-MiniArmIK.jacobian(m,q,'finite'),'fro'));
    % Independent yaw + three planar pitch joints (example model only).
    t=cumsum(q(2:4)); lengths=[.1;.1;.06];
    rho=sum(lengths.*cos(t)); z=.06-sum(lengths.*sin(t));
    expected=[rho*cos(q(1));rho*sin(q(1));z];
    assert(norm(MiniArmIK.fk(m,q)-expected)<1e-12);
    dq=1e-6*randn(4,1);
    assert(norm(MiniArmIK.fk(m,q+dq)-MiniArmIK.fk(m,q)-J*dq)<1e-10);
end
assert(worst<1e-8); fprintf('100 poses: max Jacobian discrepancy %.3g m/rad\n',worst);
% Nonidentity base/tool, theta offsets, twist and displacement exercise.
g=m; g.mdh(:,4)=[.2;-.3;.4;.1]; g.mdh(3,2)=.35; g.mdh(3,3)=.02;
g.base=MiniArmIK.mdh([.03 .2 .04 .3],0);
g.tool=MiniArmIK.mdh([.05 -.4 .03 .2],0);
assert(norm(MiniArmIK.jacobian(g,g.q0)-MiniArmIK.jacobian(g,g.q0,'finite'),'fro')<1e-8);
for method={'partial','finite'}
    target=MiniArmIK.fk(m,[.35;-.8;1.2;-.2]);
    [q,r]=MiniArmIK.solve(m,target,m.q0,method{1});
    assert(r.success && r.error<=1e-4);
    assert(all(q>=m.limits(:,1) & q<=m.limits(:,2)));
    assert(all(diff(r.errors)<=1e-12));
    [~,bad]=MiniArmIK.solve(m,[2;0;0],m.q0,method{1}); assert(~bad.success);
    [~,same]=MiniArmIK.solve(m,MiniArmIK.fk(m,m.q0),m.q0,method{1});
    assert(same.success && same.iterations==0);
end
fprintf('Reachable, unreachable, zero-error, limits and descent checks passed.\n');
if exist('rigidBodyTree','class')==8
    verify_toolbox(m); verify_toolbox(g);
end
end
