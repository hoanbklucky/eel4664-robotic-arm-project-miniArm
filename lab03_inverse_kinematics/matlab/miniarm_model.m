function m = miniarm_model()
% SIMULATION EXAMPLE ONLY: replace with measured Craig modified-DH table.
% Row i = [a_(i-1), alpha_(i-1), d_i, thetaOffset_i]. Metres, radians.
% Four positioning joints; gripper opening is NOT a positioning joint.
m = miniarm_example_model();
% Replace the fields below with YOUR measurements; for example:
% m.mdh = [...]; m.base = eye(4); m.tool = ...;
% m.limits = [...]; m.q0 = [...]; m.exampleOnly = false;
% Only set false after replacing geometry AND limits with your measurements.
end
