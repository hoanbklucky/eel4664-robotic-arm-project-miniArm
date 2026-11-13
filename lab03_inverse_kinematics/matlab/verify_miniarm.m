function verify_miniarm()
% Source-model checks: instructor dimensions, positive shoulder twist.
m=miniarm_model(); MiniArmIK.validate(m);
% Base 0 right, 90 forward, 180 left; no base angular offset.
assert(norm(MiniArmIK.fk(m,deg2rad([90;90;0;0]))-[0;0;.323])<1e-12);
assert(norm(MiniArmIK.fk(m,[0;0;0;0])-[.203;0;.120])<1e-12);
assert(norm(MiniArmIK.fk(m,deg2rad([90;0;0;0]))-[0;.203;.120])<1e-12);
assert(norm(MiniArmIK.fk(m,deg2rad([180;0;0;0]))-[-.203;0;.120])<1e-12);
rng(4664); worst=0;
for k=1:100
    q=m.limits(:,1)+rand(4,1).*(m.limits(:,2)-m.limits(:,1));
    a=cumsum(q(2:4)); L=[.0565;.0615;.085];
    d=sum(L.*cos(a)); h=.120+sum(L.*sin(a));
    assert(norm(MiniArmIK.fk(m,q)-[d*cos(q(1));d*sin(q(1));h])<1e-12);
    J=MiniArmIK.jacobian(m,q); Jn=MiniArmIK.jacobian(m,q,'finite');
    worst=max(worst,norm(J-Jn,'fro'));
    % Original planar Jacobian, mm/rad, from closed-form partials.
    P=zeros(2,3);
    for j=1:3
        P(:,j)=1000*[-sum(L(j:3).*sin(a(j:3)));sum(L(j:3).*cos(a(j:3)))];
    end
    planeProjection=[cos(q(1)) sin(q(1)) 0; 0 0 1];
    assert(norm(1000*planeProjection*J(:,2:4)-P,'fro')<1e-9);
end
assert(worst<1e-8);
[~,r]=MiniArmIK.solve(m,MiniArmIK.fk(m,m.demoGoal),m.q0);
assert(r.success); fprintf('Instructor-source model checks passed; Jacobian discrepancy %.3g m/rad\n',worst);
if exist('rigidBodyTree','class')==8, verify_toolbox(m); end
end
