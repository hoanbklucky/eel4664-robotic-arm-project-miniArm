function robot=verify_toolbox(m)
% Optional independent implementation using MathWorks Robotics System Toolbox.
% Toolbox coordinates are theta=q+offset; do NOT rely on MDH theta column.
if nargin==0, m=miniarm_model(); end
MiniArmIK.validate(m);
robot=rigidBodyTree('DataFormat','column');
b=rigidBody('model_base'); j=rigidBodyJoint('base_fixed','fixed');
setFixedTransform(j,m.base); b.Joint=j; addBody(robot,b,robot.BaseName);
parent='model_base';
for i=1:size(m.mdh,1)
    name=sprintf('link%d',i); b=rigidBody(name);
    j=rigidBodyJoint(sprintf('q%d',i),'revolute');
    setFixedTransform(j,[m.mdh(i,1:3) 0],'mdh');
    j.PositionLimits=m.limits(i,:)+m.mdh(i,4); b.Joint=j;
    addBody(robot,b,parent); parent=name;
end
b=rigidBody('tcp'); j=rigidBodyJoint('tool_fixed','fixed');
setFixedTransform(j,m.tool); b.Joint=j; addBody(robot,b,parent);
for k=1:20
    q=m.limits(:,1)+rand(size(m.mdh,1),1).*(m.limits(:,2)-m.limits(:,1));
    theta=q+m.mdh(:,4); T=getTransform(robot,theta,'tcp');
    [~,ours]=MiniArmIK.fk(m,q); assert(norm(T-ours,'fro')<1e-10);
    G=geometricJacobian(robot,theta,'tcp'); % angular first, linear last
    assert(norm(G(4:6,:)-MiniArmIK.jacobian(m,q),'fro')<1e-9);
end
fprintf('20 Robotics System Toolbox FK/Jacobian comparisons passed.\n');
figure; show(robot,m.q0+m.mdh(:,4)); title('Toolbox model: example geometry');
end
