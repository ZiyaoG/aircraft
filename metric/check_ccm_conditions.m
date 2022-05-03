lambda0 = lambda;
if isfield(plant,'df_dx')
    plant= rmfield(plant,{'df_dx','u','wdist','sinx','cosx','A','dBw_dx_fcn','A_fcn','df_dx_approx_fcn'});
end
if isfield(controller,'rho')
    controller= rmfield(controller,{'rho','c_rho','v_rho'});
end
if isfield(state_set,'box_lim')
    state_set = rmfield(state_set,{'box_lim','w_states'});
end
plant.state_set = state_set;

disp('Checking CCM conditions ...');

lambda = 0.998*lambda; % slightly reduce lambda to avoid infeasible problems due to numerical issues    
ctrl_N = 10;
alpha_range = linspace(alpha_lim(1), alpha_lim(2), ctrl_N);
q_range = [q_lim(1), q_lim(2)];
controller.df_dx_fcn = df_dx_fcn;
controller.A_fcn = plant.df_dx_fcn;

% --------------- following the approach in the following paper -------
% S.  Singh, et al. Robust  feedback  motion  planning  via
% contraction  theory. IJRR, 2019.
delta_u = zeros(ctrl_N,ctrl_N);
eig_CCM = zeros(ctrl_N, ctrl_N);
eig_W = zeros(ctrl_N,2);
for i = 1:length(alpha_range)
    x0 = [randn(1);alpha_range(i);rand(1)];    
    W = W_fcn(x0); % note that W depends only on T, phi and theta
    M = W\eye(n);
    eig_W(i,1) = min(eig(W));
    eig_W(i,2) = max(eig(W));
    for j = 1:length(q_range)  
        x0(3)=q_range(j);
        df_dx0 = plant.df_dx_fcn(x0);                
        F = dW_dt_fcn(x0,[0 0 0]') - df_dx0*W - (df_dx0*W)'-2*lambda*W; %%%%% there might be some issues here when the strong CCM condition does not hold
        R_CCM = B_perp'*F*B_perp;
        eig_CCM(i,j) = min(eig(R_CCM));
    end
end
% verify the CCM conditions
fprintf(1,'CCM, lambda = %.2f, cond(W) = %.3f\n',...
    lambda,cond_num_W);
%     fprintf(1,'Control: %.3f\n',controller.u_bnd);
fprintf(1,'min and max eigenvalues of W: %.3e, %.3e\n',min(vec(eig_W(:,1))),max(vec(eig_W(:,2))));
fprintf(1,'minimum eigenvalue of CCM matrix (should be positive): %.3e\n',min(eig_CCM(:)));
    
