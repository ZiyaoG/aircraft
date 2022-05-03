clear;
%% calculate disturbance
load('data/sim_deccm_lam_1_w_dist_1.mat');
dist = zeros(length(t_vec),1);
x_dot_traj = zeros(length(t_vec),3);
for i=1 : length(t_vec)-1
    dist(i,1) = dist_config.gen_dist(t_vec(i),xTraj(:,i));
    x_dot_traj(i,:) = norm((xTraj(:,i+1) - xTraj(:,i))/(t_vec(:,i+1) - t_vec(:,i)));
end
plot(dist)
figure(2)
plot(x_dot_traj)

%% grid x to get bound on x_dot
x1 = (-10:1:180)/180*pi;
x2 = (-5:1:40)/180*pi;
x3 = (-10:1:50)/180*pi;
u_b = 4*15/180*pi;
[theta,alpha,q] = meshgrid(x1,x2,x3);
x = [reshape(theta,[],1),reshape(alpha,[],1),reshape(q,[],1)];
dx_dot = plant.f_fcn(x') + plant.B*u_b;
max_dx_dot = vecnorm(dx_dot);
plot(max_dx_dot) % 2.1

%% compute theoretical maximal step size
b_d = 1.6+0.6*50/180*pi; %%%%%%%%%%%%%%%%%%%%%%%%%%% is this right? %%%%%%%%%%%%%%%%%%%
n = 3;
B = [0;0;1];
L_d = 1.6*2+0.6; %%%%%%%%%%%%%%%%% ?
L_B = 0; 
l_d = 0;
Phi = max(max_dx_dot);
a = 10;
syms T;

eq = (0.05 == (2*sqrt(n)*T*(L_d*Phi+l_d)+(1-exp(-a*T))*sqrt(n)*b_d)*1 + 2*sqrt(n)*T*L_B*b_d); 
T_sol = solve(eq, T);
vpa(T_sol)
