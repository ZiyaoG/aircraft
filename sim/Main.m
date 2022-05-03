%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Simulation of tracking control of the pitch dynamics of an aircraft

%  Author: Pan Zhao, UIUC, Advanced Controls Research Lab,
%  panzhao2@illinois.edu
%  Codes for the paper:
%  XX
%  Last update: Feb 10, 2022.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;
addpath('../metric');
%% 
print_file = 0; 

%% simulation settings 
sim_config.include_dist = 1;        % include the disturbance  
sim_config.save_sim_rst = 0;        % whether to save simulation results
sim_config.replan_nom_traj = 0;     % whether to replan a trajectory
sim_config.dt_sim = 5e-3;          % sampling interval for controller 
use_generated_code = 0;             % whether to use the generated codes for simulations: using generated codes can accelerate by at least ten fold
sim_config.duration = 15;           % length of simulation
sim_config.x0 = [0 0 0]';           % initial condition
sim_config.xF = [pi 0 0]';          % initial condition


%% uncertainty settings
plant.k = 0.8; % nominal = 0.2, range = [0.1, 0.8]
plant.la = -3; % nominal = -1, range = [-3, 1]

w_max = 1;                         % maximum amplitude of wind disturbance
T_w = 10;                          % period of disturbance
dist_config.sim_config.include_dist = sim_config.include_dist;
% dist_config.gen_dist= @(t,x) -(plant.k-0.2)*x(3)-(plant.la+1)*0.8*sin(2*x(2));
dist_config.gen_dist= @(t,x) -0.6*x(3)-(-2)*0.8*sin(2*x(2));

% ------------------- disturbance estimation setting ----------------------
distEst_config.a_pred = 10;                    % positive constant used in the state predictor
distEst_config.Ts = sim_config.dt_sim;                    % estimation sampling time
distEst_config.adapt_gain = -distEst_config.a_pred/(exp(distEst_config.a_pred*distEst_config.Ts)-1);
distEst_config.est_errBnd = 0.05;

%% load dynamics, controller and setup simulation environment
% ---------------------- load plant and controller ------------------------

file_controller = 'ccm_1.mat';  
load(file_controller);
controller.dW_dxi_fcn = @(i,x) (i==2)*dW_dalpha_fcn(x);
controller.ccm_law_form = ...
    CtrlDesignOpts.ccm_min_norm;                % ccm_min_norm, ccm_integration
controller.use_distModel_in_planning_control = 0;
% ---whether to include dist. estimation and error bound in CCM control----
controller.distEstScheme = 2;       %{0,1,2}: 0 for ignoring, 1 for estimating the remainder disturbance $\tilde ur$ (between the learned disturbance and true disturbance), 2 for estimating the total disturbance d
controller.use_distEst_errBnd = 1; 
controller.filter_distEst = 1;

controller.use_generated_code = use_generated_code;

%% generate nominal trajectories
gray_color = [1 1 1]*80/255;
file_traj = 'nomTraj.mat';
if sim_config.replan_nom_traj == 1
    trajGen_config.x0 = sim_config.x0;
    trajGen_config.xF = sim_config.xF;
    trajGen_config.x_bnd = [-10 180;-10 50;-10 50]*pi/180;
    trajGen_config.u_bnd = [-15 15]*pi/180;
    trajGen_config.duration = sim_config.duration;
    soln = plan_traj_aircraft(plant,trajGen_config);

    tF = soln.grid.time(end); trajGen_config.tF = tF;
    save(file_traj,'trajGen_config','soln');
else
    load(file_traj);
end
duration = trajGen_config.tF;   % modify the duration according to the computed trajectory
sim_config.duration = duration;
%% --------------------- show the planned traj -----------------------------
x_nom_fcn = soln.interp.state;
u_nom_fcn = soln.interp.control;
times = 0:0.05:duration;
simuLen = length(times);
xnomTraj = zeros(plant.n,simuLen);
unomTraj = zeros(plant.nu,simuLen);
for t =1:simuLen
    xnomTraj(:,t) = x_nom_fcn(times(t));
    unomTraj(:,t) = u_nom_fcn(times(t));
end
sim_config.trajGen_config = trajGen_config;

figure(2);
subplot(2,2,1)
plot(times,xnomTraj(1,:)*180/pi);
ylabel('\theta (deg)')
subplot(2,2,2)
plot(times,xnomTraj(2,:)*180/pi);
ylabel('\alpha (deg)')
subplot(2,2,3)
plot(times,xnomTraj(3,:)*180/pi);
ylabel('q (deg/s)')
subplot(2,2,4)
plot(times,unomTraj*180/pi);
ylabel('u (deg/s^2)')
controller.x_nom_fcn = x_nom_fcn;
controller.u_nom_fcn = u_nom_fcn;
% return
%% Formulate the NLP problem for geodesic computation
D = 3;      % degree of the polynomial
N = 6;      % stopping index for the CGL (Chebyshev-Gauss-Lobatto) nodes: #notes N+1
geodesic = setup_geo_opt(plant.n,D,N,controller);
% geodesic.nlprob = []; % setting geodesic.nlprob to [] indicates that the geodesic will be approximated by a straight line 
% geodesic = [];

controller.geodesic = geodesic;
controller.w_nom = 0;  % nominal value for disturbances

sim_config.x0 = [5/180*pi 5/180*pi 0]';
% simulate
t_vec = 0:sim_config.dt_sim:sim_config.duration;
T_steps = length(t_vec)-1;

% note that 
% x: theta, alpha, q
xTraj = zeros(plant.n,T_steps+1);
% xhatTraj = zeros(plant.n,T_steps+1);
uTraj = zeros(plant.nu,T_steps);
% DistTraj = zeros(plant.nu,T_steps);
% estDistTraj = zeros(plant.nu,T_steps);
xnomTraj = xTraj;
unomTraj = uTraj;
energyTraj = zeros(1,T_steps);

% distEst = 0;
% Bsigmahat = zeros(3,1);

% initial condition
x = sim_config.x0;
xTraj(:,1) = x;

for i=1:T_steps
    t = t_vec(i);

    x_0 = xTraj(:,i);
%     xhat_0 = xhatTraj(:,i);
%     estDist_0 = estDistTraj(i);
%     x_xhat_0 = [x_0;xhat_0];

    % disturbance estimation 
%     xtilde = xhat_0-x_0;
%     Bsigmahat = distEst_config.adapt_gain*xtilde; 
%     distEst = pinv(plant.B)*Bsigmahat;

    % get the nominal state and input
    [u,energy,x_nom,u_nom] = ccm_law(t,x_0,plant,controller,@(x)0,0,0.1);
%     distEst_ccm_control = distEst;
%     [u,energy,x_nom,u_nom] = deccm_law(t,x_0,plant,controller,@(x)0,distEst_ccm_control,distEst_config.est_errBnd);

    % record
    xnomTraj(:,i) = x_nom;
    uTraj(:,i) = u; 
    unomTraj(:,i) = u_nom;
    energyTraj(i) = energy;
        
    % propagate with zero-hold for control inputs
    [d_t,x_xhat] = ode23(@(t,x) aircraft_dyn(t,x,u,plant,dist_config),[t_vec(i) t_vec(i+1)],x); %,ode_options
%     [d_t,x_xhat] = ode23(@(t,x_xhat) aircraft_dyn_estimator(t,x_xhat,u,plant,dist_config,distEst_config,Bsigmahat),[t_vec(i) t_vec(i+1)],[x_0;xhat_0]); %,ode_options

    % update and record;
    x_xhat = x_xhat';
    xTraj(:,i+1) = x_xhat(1:plant.n,end); 
%     xhatTraj(:,i+1) = x_xhat(plant.n+1:2*plant.n,end);
%     estDistTraj(:,i) = distEst;
%     DistTraj(:,i) = dist_config.gen_dist(i*sim_config.dt_sim,x_0);
end  
% xnomTraj(:,i+1) = xnomTraj(:,1) ;
% [~,energy(i+1)] = ccm_law(t_vec(i+1),x,plant,controller,@(x)0,0,0.1);
%% plot the result
plot_trajs;

%% save_data
if sim_config.save_sim_rst == 1
    file_name = 'data/sim_ccm';      
    file_name = [file_name '_lam_' num2str(controller.lambda,2)];
    if dist_config.sim_config.include_dist == 1
        file_name = [file_name '_w_dist_' num2str(w_max)];
    end    
    file_name  = [file_name '.mat'];
    save(file_name,'t_vec','xTraj','uTraj','xnomTraj','unomTraj','energyTraj','dist_config','sim_config','plant','controller');
end

%%
function dx_dot = aircraft_dyn(t,x,u,plant,dist_config)
dist = dist_config.gen_dist(t,x);
% tmp_nomdist = -0.2*x(3)-(-1)*0.8*sin(2*x(2));
tmp_nomdist = 0;
if dist_config.sim_config.include_dist == 1
    dx_dot = plant.f_fcn(x) + plant.B*(u + tmp_nomdist + dist);
else
    dx_dot = plant.f_fcn(x) + plant.B*(u + tmp_nomdist);
end

end

%%
function dx_dt= aircraft_dyn_estimator(t,x_xhat_d,u,plant,dist_config,distEst_config,Bsigmahat)

%propagate
n = plant.n;
x = x_xhat_d(1:n);
xhat = x_xhat_d(n+1:2*n,:);
% d = x_xhat_d(2*n+1,:);
xtilde = xhat-x;

wt = dist_config.gen_dist(t,x);
xdot = plant.f_fcn(x)+plant.B*u; % nominal dynamics;
xhatdot = xdot + Bsigmahat - distEst_config.a_pred*xtilde;

dx_dt = [xdot; xhatdot];
if dist_config.sim_config.include_dist == 1
   dx_dt(1:n,:) = dx_dt(1:n,:) + plant.B*wt;
end
end