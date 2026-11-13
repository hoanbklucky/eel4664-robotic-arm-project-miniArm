function demo_ik(target)
% Run demo_ik or demo_ik([0.20;0.04;0.10]); no add-on required.
m=miniarm_model(); MiniArmIK.validate(m);
if m.exampleOnly, warning('Geometry/limits require hardware validation before motion'); end
if nargin==0
    target=MiniArmIK.fk(m,m.demoGoal); % known reachable under simulation limits
end
target=target(:);
[q,r]=MiniArmIK.solve(m,target,m.q0,'partial');
fprintf('%s: residual %.6f m, %d updates\n',r.status,r.error,r.iterations);
disp(table((1:numel(q))',q,rad2deg(q),'VariableNames',{'Joint','Radians','Degrees'}));
figure('Name','Craig MDH position IK');
subplot(1,2,1); ax=gca;
for k=1:size(r.history,2)
    [~,~,P]=MiniArmIK.fk(m,r.history(:,k));
    cla(ax); plot3(ax,P(1,:),P(2,:),P(3,:),'o-','LineWidth',2); hold(ax,'on');
    plot3(ax,target(1),target(2),target(3),'rx','MarkerSize',12,'LineWidth',2);
    grid(ax,'on'); axis(ax,'equal'); view(ax,40,25);
    reach=sum(abs(m.mdh(:,1)))+sum(abs(m.mdh(:,3)))+norm(m.tool(1:3,4));
    centre=m.base(1:3,4); xlim(ax,centre(1)+[-reach reach]);
    ylim(ax,centre(2)+[-reach reach]); zlim(ax,centre(3)+[-reach reach]);
    xlabel(ax,'x (m)'); ylabel(ax,'y (m)'); zlabel(ax,'z (m)');
    title(ax,sprintf('Solver update %d (not elapsed time)',k-1)); drawnow;
end
subplot(1,2,2); semilogy(0:r.iterations,max(r.errors,eps),'o-');
xlabel('Solver update'); ylabel('Position error (m)'); grid on;
if ~r.success, warning('No solution found: do not command hardware.'); end
end
