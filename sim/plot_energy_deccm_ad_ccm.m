clear;clf;
close all;

figure(1);
%% Task 1
figure(1)
load('data/sim_deccm_lam_1_w_dist_1.mat');
% subplot(3,1,1)
hold on;
delta_x = xnomTraj-xTraj;
x_xnom = vecnorm(delta_x);
R = sqrt(controller.w_upper/controller.w_lower);
h1 = fplot(@(t) R*norm([0 0 0] - [5/180*pi 5/180*pi 0])*exp(-controller.lambda*t),'Color', 'k','LineStyle','--','Linewidth',1);
h2 = plot(t_vec(1,1:end-1),x_xnom(1,1:end-1),'Color', 'r','LineStyle','-','Linewidth',1);
xlim([0 13]);

%% 
load('data/updated_sim_adaptiveccm_gamma_1000_w_dist_1.mat');
% subplot(3,1,1)
delta_x = xnomTraj-xTraj;
x_xnom = vecnorm(delta_x);
h7 = plot(t_vec(1,1:end-1),x_xnom(1,1:end-1),'Color', 'b','LineStyle','-.','Linewidth',1);

%% 
load('data/updated_sim_ccm_lam_1_w_dist_1.mat');
% subplot(3,1,1)
delta_x = xnomTraj-xTraj;
x_xnom = vecnorm(delta_x);
h10 = plot(t_vec(1,1:end-1),x_xnom(1,1:end-1),'Color', 'm','LineStyle',':','Linewidth',1);

%%
% subplot(3,1,1)
xlabel('Time (s)','interpreter','latex')
ylabel('$\|x-x^*\|$','interpreter','latex')
legend([h1,h2,h7,h10],{'$R\|x(0)-x^*(0)\|e^{-\lambda t}$','DE-CCM','Ad-CCM', 'CCM'},'NumColumns',4,'Location','North','Orientation','horizontal','interpreter','latex');
xlim([0 13]);
ylim([0 0.23]);
goodplot([6.5 6.5]);

print('figures/x_xnom_deccm_ad_ccm.pdf', '-painters', '-dpdf', '-r150');