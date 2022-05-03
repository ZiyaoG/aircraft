clear;clf;
close all;

figure(1);
%% Task 1
figure(1)
load('data/sim_deccm_lam_1_w_dist_1.mat');
% subplot(3,1,1)
hold on;
h1 = plot(t_vec(1,1:end-1),180/pi*unomTraj(1,:),'k--','Linewidth',1);
h2 = plot(t_vec(1,1:end-1),180/pi*uTraj(1,:),'Color', 'r','LineStyle','-','Linewidth',1);
% subplot(3,1,2)
% hold on;
% h3 = plot(t_vec,unomTraj(2,:),'k--','Linewidth',1);
% h4 = plot(t_vec,xTraj(2,:),'Color', 'r','LineStyle','-','Linewidth',1);
% subplot(3,1,3)
% hold on;
% h5 = plot(t_vec,xnomTraj(3,:),'k--','Linewidth',1);
% h6 = plot(t_vec,xTraj(3,:),'Color', 'r','LineStyle','-','Linewidth',1);

%% 
load('data/updated_sim_adaptiveccm_gamma_1000_w_dist_1.mat');
% subplot(3,1,1)
h7 = plot(t_vec(1,1:end-1),180/pi*uTraj(1,:),'Color', 'b','LineStyle','-.','Linewidth',1);
% subplot(3,1,2)
% h8 = plot(t_vec,xTraj(2,:),'Color', 'b','LineStyle',':','Linewidth',1);
% subplot(3,1,3)
% h9 = plot(t_vec,xTraj(3,:),'Color', 'b','LineStyle',':','Linewidth',1);
%% 
load('data/updated_sim_ccm_lam_1_w_dist_1.mat');
% subplot(3,1,1)
h10 = plot(t_vec(1,1:end-1),180/pi*uTraj(1,:),'Color', 'm','LineStyle',':','Linewidth',1);
% subplot(3,1,2)
% h11 = plot(t_vec,xTraj(2,:),'Color', 'm','LineStyle','-.','Linewidth',1);
% subplot(3,1,3)
% h12 = plot(t_vec,xTraj(3,:),'Color', 'm','LineStyle','-.','Linewidth',1);
%%
% subplot(3,1,1)
xlabel('Time (s)','interpreter','latex')
ylabel('$u (^{\circ}/s^2$)','interpreter','latex')
legend([h1,h2,h7,h10],{'Planned', 'DE-CCM','Ad-CCM', 'CCM'},'NumColumns',4,'Location','North','Orientation','horizontal','interpreter','latex');
xlim([0 13]);
goodplot([6.5 6.5]);

% subplot(3,1,2)
% xlabel('$p_x$ (m)','interpreter','latex')
% ylabel('$p_z$ (m)','interpreter','latex')
% legend([h3,h4,h8,h11],{'Planned', 'DE-CCM','Ad-CCM($\Gamma=1000$)', 'CCM'},'NumColumns',4,'Location','South','Orientation','horizontal','interpreter','latex');
% goodplot([7 8]);
% xlim([0 13]);
% 
% subplot(3,1,3)
% xlabel('$p_x$ (m)','interpreter','latex')
% ylabel('$p_z$ (m)','interpreter','latex')
% legend([h5,h6,h9,h12],{'Planned', 'DE-CCM','Ad-CCM($\Gamma=1000$)', 'CCM'},'NumColumns',4,'Location','South','Orientation','horizontal','interpreter','latex');
% xlim([0 13]);
% % ylim([0 8.5]);
% goodplot([7 8]);

print('figures/Input_deccm_ad_ccm.pdf', '-painters', '-dpdf', '-r150');