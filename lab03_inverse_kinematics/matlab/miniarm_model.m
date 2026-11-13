function m = miniarm_model()
% Instructor-source geometry; joint limits still require physical calibration.
% Row i = [a_(i-1), alpha_(i-1), d_i, thetaOffset_i]. Metres, radians.
% Four positioning joints; gripper opening is NOT a positioning joint.
m.mdh = [0 0 .120 0; 0 pi/2 0 0; .0565 0 0 0; .0615 0 0 0];
m.base = eye(4);
m.tool = eye(4); m.tool(1,4) = .085;
% q = [base_angle-90; arm_q1; forearm_q2; wrist_q3], in radians.
% These are SIMULATION limits, NOT inferred servo limits.
m.limits = deg2rad([-90 90; 0 180; -160 160; -160 160]);
m.q0 = deg2rad([0;60;-60;-60]);
m.demoGoal = deg2rad([20;70;-80;-40]);
m.exampleOnly = true; % keeps export guarded until limits/calibration verified
m.label = 'Instructor miniArm geometry; uncalibrated simulation limits';
% Optional collision approximation. FK columns: table, shoulder, shoulder,
% elbow, wrist, TCP. Capsules: base, upper arm, forearm, tool.
% Radii/margin are PROVISIONAL simulation values: measure your actual robot.
m.collision.edges = [1 2; 3 4; 4 5; 5 6];
m.collision.radii = [.020 .006 .006 .006];
m.collision.pairs = [1 3; 1 4; 2 4]; % exclude connected neighbouring links
m.collision.names = {'base','upper arm','forearm','tool'};
m.collision.margin = .003;
m.collision.floorEdges = [2 3 4];
m.collision.floorZ = 0;
% Only set false after replacing geometry AND limits with your measurements.
end
