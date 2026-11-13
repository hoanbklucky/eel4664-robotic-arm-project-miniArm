function verify_collision()
% Collision regressions. Run in MATLAB after refreshing all .m files.
m=miniarm_model(); MiniArmIK.validate(m);
assert(abs(MiniArmCollision.segmentDistance([0;0;0],[1;0;0],[.5;-1;0],[.5;1;0]))<1e-12);
assert(abs(MiniArmCollision.segmentDistance([0;0;0],[1;0;0],[0;1;0],[1;1;0])-1)<1e-12);
assert(abs(MiniArmCollision.segmentDistance([0;0;0],[0;0;0],[1;-1;0],[1;1;0])-1)<1e-12);
q=deg2rad([90;100;-120;-120]);
assert(MiniArmCollision.clearance(m,q)<0);
[~,r]=MiniArmIK.solve(m,MiniArmIK.fk(m,q),q);
assert(~r.success && strcmp(r.status,'seed_in_collision'));
% The prior position-only solver reaches this target THROUGH the base.
target=MiniArmIK.fk(m,deg2rad([90;120;-100;-120]));
old=m; old=rmfield(old,'collision');
[oldq,oldr]=MiniArmIK.solve(old,target,old.q0);
assert(oldr.success && MiniArmCollision.clearance(m,oldq)<0);
[~,r]=MiniArmIK.solve(m,target,m.q0);
assert(~r.success && r.clearance>0);
for k=2:size(r.history,2)
    assert(MiniArmCollision.edgeClear(m,r.history(:,k-1),r.history(:,k)));
end
% Both endpoints clear the table, but the middle of interpolation does not.
tableOnly=m; tableOnly.collision.pairs=zeros(0,2); tableOnly.collision.floorEdges=4;
a=deg2rad([90;30;-120;-160]); b=deg2rad([90;30;-120;160]);
assert(MiniArmCollision.clearance(tableOnly,a)>0 && MiniArmCollision.clearance(tableOnly,b)>0);
assert(MiniArmCollision.clearance(tableOnly,(a+b)/2)<0);
assert(~MiniArmCollision.edgeClear(tableOnly,a,b));
[~,r]=MiniArmIK.solve(m,MiniArmIK.fk(m,m.demoGoal),m.q0);
assert(r.success && r.clearance>0);
fprintf('Collision rejection, interpolation and safe-goal regressions passed.\n');
end
