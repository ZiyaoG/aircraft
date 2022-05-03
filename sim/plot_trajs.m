close;clf;
n = plant.n; nu = plant.nu;
color = {'k','b',[0 0.5 0],'r',[0.8 0.9 0.9741],[0.8 0.98 0.9],[1 0.8 0.8],[0.7, 0.7 1]};
%-------------------------------------------------

addtext = 'ccm';

%% trajectory error
figure(1);hold on;
clf;
subplot(3,1,1);
plot(t_vec,xTraj(1,:)*180/pi,'r-',t_vec, xnomTraj(1,:)*180/pi,'k--','Linewidth',1.5); hold on;
ylabel('\theta (deg)')
subplot(3,1,2);
plot(t_vec,xTraj(2,:)*180/pi,'r-',t_vec, xnomTraj(2,:)*180/pi,'k--','Linewidth',1.5); hold on;
ylabel('\alpha (deg)')
subplot(3,1,3);
plot(t_vec,xTraj(3,:)*180/pi,'r-',t_vec, xnomTraj(3,:)*180/pi,'k--','Linewidth',1.5); hold on;
ylabel('$q$ (deg/s)','interpreter','latex')

figname = [addtext '_trajs'];
if print_file == 1
%     savefig('traj_err_10.fig');
    print(figname,'-dpdf', '-r300');
%     export_fig(figname,'-pdf', '-q101');
end


%% Control effort & Energy
figure(2)
clf;
% control effort
subplot(2,1,1);hold on;
h2 = plot(t_vec(1:end-1), uTraj(1,:),'r-','linewidth',1.5);hold on
h1 = plot(t_vec(1:end-1), unomTraj(1,:),'k--','linewidth',1.5); 

% xlabel('Time (s)');
ylabel('$u$ (Nm)','interpreter','latex');
legend([h1 h2],{'nominal','actual'},'location','best','orientation','horizontal');
goodplot([6 6]);

subplot(2,1,2);hold on
% Geodesic Energy
plot(t_vec(1:end-1),energyTraj,'b-','linewidth',1.5); hold on
% grid on
xlabel('Time (s)'); ylabel('Energy');
goodplot([6 2.5],[0.1 0.01])
if print_file == 1
    figname = [addtext '_input_energy'];
%     savefig('energy.fig');
    print(figname,'-painters', '-dpdf', '-r150');
end


% %% disturbance estimation
% figure(3)
% clf;
% h3 = plot(t_vec(:,1:end-1),estDistTraj,'r-');
% hold on
% h4 = plot(t_vec(:,1:end-1),DistTraj,'k--');
% legend([h3 h4],{'Estimated','Actual'},'location','best','orientation','horizontal');