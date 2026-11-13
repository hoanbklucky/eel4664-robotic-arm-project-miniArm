classdef MiniArmIK
% Position-only IK for an all-revolute Craig modified-DH chain.
methods(Static)
function validate(m)
    MiniArmCollision.validate(m);
    n = size(m.mdh,1);
    assert(n > 0 && size(m.mdh,2)==4 && all(isfinite(m.mdh(:))), 'Invalid MDH table');
    assert(isequal(size(m.limits),[n 2]) && all(isfinite(m.limits(:))) && ...
        all(m.limits(:,1)<m.limits(:,2)), 'Invalid limits');
    assert(numel(m.q0)==n && all(isfinite(m.q0(:))) && ...
        all(m.q0(:)>=m.limits(:,1) & m.q0(:)<=m.limits(:,2)), 'Invalid q0');
    for F = {m.base,m.tool}
        H=F{1}; assert(isequal(size(H),[4 4]) && all(isfinite(H(:))), 'Invalid transform');
        assert(norm(H(4,:)-[0 0 0 1])<1e-10 && ...
            norm(H(1:3,1:3)'*H(1:3,1:3)-eye(3),'fro')<1e-10 && ...
            abs(det(H(1:3,1:3))-1)<1e-10, 'Transform must be rigid');
    end
end
function [A,D] = mdh(row,q)
    % Rx(alpha) Tx(a) Rz(thetaOffset+q) Tz(d), Craig convention.
    a=row(1); al=row(2); d=row(3); t=row(4)+q;
    c=cos(t); s=sin(t); ca=cos(al); sa=sin(al);
    A=[c -s 0 a; s*ca c*ca -sa -d*sa; s*sa c*sa ca d*ca; 0 0 0 1];
    % Exact partial derivative dA/dq; no Symbolic Math Toolbox needed.
    D=[-s -c 0 0; c*ca -s*ca 0 0; c*sa -s*sa 0 0; 0 0 0 0];
end
function [p,T,P] = fk(m,q)
    q=q(:); n=size(m.mdh,1);
    assert(numel(q)==n && all(isfinite(q)), 'Invalid joint vector');
    T=m.base; P=zeros(3,n+2); P(:,1)=T(1:3,4);
    for i=1:n
        T=T*MiniArmIK.mdh(m.mdh(i,:),q(i)); P(:,i+1)=T(1:3,4);
    end
    T=T*m.tool; p=T(1:3,4); P(:,end)=p;
end
function J = jacobian(m,q,method,h)
    if nargin<3, method='partial'; end
    if nargin<4, h=1e-5; end
    q=q(:); n=numel(q); J=zeros(3,n);
    assert(n==size(m.mdh,1) && all(isfinite(q)), 'Invalid joint vector');
    if strcmp(method,'finite')
        assert(isscalar(h) && isfinite(h) && h>0, 'h must be positive');
        for j=1:n
            dq=zeros(n,1); dq(j)=h;
            J(:,j)=(MiniArmIK.fk(m,q+dq)-MiniArmIK.fk(m,q-dq))/(2*h);
        end
    elseif strcmp(method,'partial')
        % Product rule: base*A1*...*D_j*...*An*tool.
        for j=1:n
            dT=m.base;
            for i=1:n
                [A,D]=MiniArmIK.mdh(m.mdh(i,:),q(i));
                if i==j, dT=dT*D; else, dT=dT*A; end
            end
            dT=dT*m.tool; J(:,j)=dT(1:3,4);
        end
    else
        error('Use partial or finite');
    end
end
function [q,r] = solve(m,target,q0,method)
    if nargin<4, method='partial'; end
    MiniArmIK.validate(m); target=target(:); q=q0(:);
    assert(numel(target)==3 && all(isfinite(target)), 'Target must be finite xyz');
    assert(numel(q)==size(m.mdh,1) && all(isfinite(q)) && ...
        all(q>=m.limits(:,1) & q<=m.limits(:,2)), 'Seed outside limits');
    tol=1e-4; lambda=.01; maxStep=.10; maxIter=400;
    r.history=q; r.errors=norm(target-MiniArmIK.fk(m,q));
    r.status='max_iterations';
    [r.clearance,r.contact]=MiniArmCollision.clearance(m,q);
    if r.clearance<=0
        r.status='seed_in_collision'; r.error=r.errors(1); r.success=false; r.iterations=0;
        return;
    end
    blocked=false;
    for k=1:maxIter
        e=target-MiniArmIK.fk(m,q);
        if norm(e)<=tol, r.status='converged'; break; end
        J=MiniArmIK.jacobian(m,q,method);
        % Damped least squares: solves least-squares local position update.
        dq=J'*((J*J'+lambda^2*eye(3))\e);
        dq=dq*min(1,maxStep/max(norm(dq),eps));
        accepted=false;
        for ls=0:15
            trial=min(max(q+(.5^ls)*dq,m.limits(:,1)),m.limits(:,2));
            if norm(target-MiniArmIK.fk(m,trial)) < norm(e)-1e-12
                if MiniArmCollision.edgeClear(m,q,trial)
                    q=trial; accepted=true; break;
                end
                blocked=true;
            end
        end
        if ~accepted
            if blocked, r.status='stalled_or_collision'; else, r.status='stalled'; end
            break;
        end
        r.history(:,end+1)=q; r.errors(end+1)=norm(target-MiniArmIK.fk(m,q));
    end
    r.error=norm(target-MiniArmIK.fk(m,q));
    [r.clearance,r.contact]=MiniArmCollision.clearance(m,q);
    r.success=r.error<=tol && r.clearance>0;
    if r.success, r.status='converged'; end
    r.iterations=size(r.history,2)-1;
end
end
end
