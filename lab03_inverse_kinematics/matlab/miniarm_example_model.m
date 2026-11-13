function m = miniarm_example_model()
% Fixed test fixture: illustrative yaw + three pitch joints, NOT miniArm.
m.mdh = [0 0 .060 0; 0 -pi/2 0 0; .100 0 0 0; .100 0 0 0];
m.base = eye(4);
m.tool = eye(4); m.tool(1,4) = .060;
m.limits = repmat([-pi*0.8 pi*0.8],4,1);
m.q0 = [0; -.5; 1; -.3];
m.exampleOnly = true;
end
