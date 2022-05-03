%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Synthesis of a CCM for the simplified pitch dynamics of an aircraft used in
% the following paper
%Lopez, Brett T., Jean-Jacques E. Slotine, and Jonathan P. How. "Robust adaptive control barrier functions: An adaptive and data-driven approach to safety." IEEE Control Systems Letters 5.3 (2020): 1031-1036.

%  Author: Pan Zhao, UIUC, Advanced Controls Research Lab,
%  panzhao2@illinois.edu
  
%  Last update: April 18, 2022
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; %clc; 
yalmip('clear')
addpath('metrics_search');
addpath('../../ccm/metric_search_offline/');
addpath('../../ccm/control_law_online/');
addpath('../../utilities/');
%% general settings 
save_rsts = 1;                                  % whether to save the results to a file

%% load plant
load_plant;

%% settings for searching CCM & RCCM 
lambda = 1;                                  % a line search or bisection search should be performed for lambda
consider_state_set = 1;                         % whether to consider a compact set for the states when formulating the constraints

w_lower_bound = .1;
w_states_index = [2];
w_order = 2; 
w_states = x(w_states_index);
[w_poly, w_poly_mat] = monomials(w_states,0:w_order);  % monomials of T, phi, theta, 

% w_poly= w_poly(w_poly_mat(:,1)<=2);%only keep monomials whose degree w.r.t T(thrust) <=2
% w_poly_mat = w_poly_mat(w_poly_mat(:,1)<=2,:);

% if controller.type ==  CtrlDesignOpts.rccm
%     [w_poly, w_poly_mat] = monomials([w_states;u],0:w_order);  % monomials of T, phi, theta, and u
%     II = find(w_poly_mat(:,1)<=1 & sum(w_poly_mat(:,length(w_states)+1:end),2)<=1);
%     w_poly= w_poly(II);%only keep monomials whose degree w.r.t T(thrust) <=2, w.r.t u<=1
%     w_poly_mat = w_poly_mat(II,:);
% end
n_monos_W = length(w_poly); 

% ------------state constraints imposed when searching CCM/RCCM-----------
alpha_lim = [-5 50]*deg2rad;  
q_lim = [-10 40]*deg2rad;  
state_set.ccm_states =  w_states;               % states invovled in the CCM condition
state_set.alpha_lim = alpha_lim;
state_set.q_lim = q_lim;                
state_set.w_states = w_states;
% -------------------------------------------------------------------------
     
dw_poly_dx = diff(w_poly,w_states);
dw_poly_dt = dw_poly_dx*(f(w_states_index)+plant.B(w_states_index,:)*u); 

% create the functions
w_poly_fcn = mss2fnc(w_poly,x,randn(length(x),2));
dw_poly_dt_fcn = mss2fnc(dw_poly_dt,[x;u],randn(length([x;u]),2));

prog = spotsosprog;
W_list  = cell(n_monos_W,1);

for i = 1:length(w_poly) 
    [prog, W_list{i}] =  prog.newSym(n);    
end
% ------------------------------------------------------------------
% create the function handle
W_exec = 'W_eval = @(ml)';
for i = 1:length(w_poly)
    if i<length(w_poly)
        W_exec = strcat(W_exec,sprintf('W_list{%d}*ml(%d) +',i,i));
    else
        W_exec = strcat(W_exec,sprintf('W_list{%d}*ml(%d);',i,i));
    end
end
eval(W_exec);    
    

controller.w_lower_bound = w_lower_bound;
state_set.consider_state_set = consider_state_set;
state_set.w_states = w_states;
state_set.w_states_index = w_states_index;

%% Search CCM
tic;
controller.lambda = lambda; 
[cond_num_W,W_bar,w_lower,w_upper,W_coef,w_poly] = ccm_synthesis_aircraft(prog,plant,controller,W_list,W_eval,w_poly,w_poly_fcn,dw_poly_dt_fcn,lambda,state_set); 
controller.w_upper = w_upper;
controller.w_lower = w_lower;    
controller.W_bar = W_bar;

toc;
%% extract functions -------------------------
extract_funcs;

controller.W_fcn = W_fcn;
controller.dW_dxi_fcn = dW_dxi_fcn;
controller.dW_dt_fcn = dW_dt_fcn;

%% check CCM conditions, compute tubes 
check_ccm_conditions;

%% save data
if save_rsts == 1
    if n_monos_W == 1
        file_name = ['ccm_const_' num2str(lambda0) '.mat'];
    else
        file_name = ['ccm_' num2str(lambda0) '.mat'];
    end   
    save(file_name,'plant','controller','state_set');
end
%% generate the c codes for accelerating geodesic computation
% To use the generated codes, copy .mex and .mat files to the sim folder

% parameters used in the pseudospectral method for geodesic computation
geodesic_setting_for_codegen.D = 3; 
geodesic_setting_for_codegen.N = 6;
answer = questdlg('Do you want to generate the C codes for accelerating geodesic computation used for determining the control law?','Question for code generation','Yes','No','No');
switch answer 
    case 'Yes'
        generate_code_for_geodesic_cal(plant.n,plant.nu,plant.nw,geodesic_setting_for_codegen);        
        save('geodesic_setting_for_codegen.mat','geodesic_setting_for_codegen');
    case 'No'
end
