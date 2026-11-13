classdef MiniArmCollision
% Capsule approximations, not a measured CAD collision model.
methods(Static)
function enabled=enabled(m)
    enabled=isfield(m,'collision') && ~isempty(m.collision);
end
function validate(m)
    if ~MiniArmCollision.enabled(m), return; end
    c=m.collision; n=size(m.mdh,1)+2;
    assert(size(c.edges,2)==2 && all(isfinite(c.edges(:))) && ...
        all(c.edges(:)==fix(c.edges(:))) && all(c.edges(:)>=1 & c.edges(:)<=n), 'Invalid collision edges');
    assert(numel(c.radii)==size(c.edges,1) && all(isfinite(c.radii(:))) && ...
        all(c.radii(:)>0), 'Invalid capsule radii');
    assert(size(c.pairs,2)==2 && all(isfinite(c.pairs(:))) && ...
        all(c.pairs(:)==fix(c.pairs(:))) && ...
        all(c.pairs(:)>=1 & c.pairs(:)<=size(c.edges,1)), 'Invalid collision pairs');
    assert(isscalar(c.margin) && isfinite(c.margin) && c.margin>=0 && ...
        isscalar(c.floorZ) && isfinite(c.floorZ), 'Invalid clearance/floor');
    assert(all(isfinite(c.floorEdges(:))) && all(c.floorEdges(:)==fix(c.floorEdges(:))) && ...
        all(c.floorEdges(:)>=1 & c.floorEdges(:)<=size(c.edges,1)), 'Invalid floor edges');
    assert(numel(c.names)==size(c.edges,1) && (~isempty(c.pairs) || ~isempty(c.floorEdges)), ...
        'Missing names or collision checks');
end
function d=pointSegment(p,a,b)
    v=b-a; vv=dot(v,v);
    if vv<1e-24, t=0; else, t=min(1,max(0,dot(p-a,v)/vv)); end
    d=norm(p-(a+t*v));
end
function d=segmentDistance(a,b,c,e)
    u=b-a; v=e-c; w=a-c;
    aa=dot(u,u); cc=dot(v,v);
    d=min([MiniArmCollision.pointSegment(a,c,e),MiniArmCollision.pointSegment(b,c,e), ...
        MiniArmCollision.pointSegment(c,a,b),MiniArmCollision.pointSegment(e,a,b)]);
    normal=cross(u,v); detv=dot(normal,normal);
    if aa>1e-24 && cc>1e-24 && detv>1e-30
        s=dot(cross(c-a,v),normal)/detv; t=dot(cross(c-a,u),normal)/detv;
        if s>=0 && s<=1 && t>=0 && t<=1, d=min(d,norm(w+s*u-t*v)); end
    end
end
function [value,contact]=clearance(m,q)
    if ~MiniArmCollision.enabled(m), value=Inf; contact='disabled'; return; end
    c=m.collision; [~,~,P]=MiniArmIK.fk(m,q); values=[]; labels={};
    for k=1:size(c.pairs,1)
        i=c.pairs(k,1); j=c.pairs(k,2); u=P(:,c.edges(i,:)); v=P(:,c.edges(j,:));
        values(end+1)=MiniArmCollision.segmentDistance(u(:,1),u(:,2),v(:,1),v(:,2)) ...
            -c.radii(i)-c.radii(j)-c.margin;
        labels{end+1}=[c.names{i} ' / ' c.names{j}];
    end
    for i=c.floorEdges(:)'
        values(end+1)=min(P(3,c.edges(i,:)))-c.floorZ-c.radii(i)-c.margin;
        labels{end+1}=[c.names{i} ' / table'];
    end
    [value,k]=min(values); contact=labels{k};
end
function safe=edgeClear(m,start,finish,depth)
    % Continuous straight joint interpolation, conservatively bounded.
    % Point travel <= chainLength*sum(abs(dq)); clearance changes <=2*travel.
    % Subdivide uncertified intervals; fail closed at depth 12.
    if nargin<4, depth=0; end
    if ~MiniArmCollision.enabled(m), safe=true; return; end
    c0=MiniArmCollision.clearance(m,start); c1=MiniArmCollision.clearance(m,finish);
    if min(c0,c1)<=0, safe=false; return; end
    reach=sum(abs(m.mdh(:,1)))+sum(abs(m.mdh(:,3)))+norm(m.tool(1:3,4));
    bound=2*reach*sum(abs(finish(:)-start(:)));
    if max(c0,c1)>bound+1e-12, safe=true; return; end
    if depth>=12, safe=false; return; end
    mid=(start+finish)/2;
    safe=MiniArmCollision.edgeClear(m,start,mid,depth+1) && ...
        MiniArmCollision.edgeClear(m,mid,finish,depth+1);
end
end
end
