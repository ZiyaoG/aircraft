
function [cond_num_W_opt,W_bar,w_lower,w_upper,W_coef,w_poly] = ccm_synthesis_aircraft(prog,plant,controller,W_list,W_eval,w_poly,w_poly_fcn,dw_poly_dt_fcn,lambda,state_set)
% The following codes are based on the codes available at https://github.com/stanfordASL/RobustMP
norm_scale = 1e-7;
% W_scale = (1e-3)*diag([1 1 1]);
ccm_eps =  0.01;
A_fcn = plant.A_fcn;
n = plant.n;nu = plant.nu;

[prog, w_lower] = prog.newPos(1);
[prog, w_upper] = prog.newPos(1);
[prog, W_bar] = prog.newSym(n);


%W uniform bounds
prog = prog.withPos(w_lower-controller.w_lower_bound);
prog = prog.withPSD(w_upper*eye(n)-W_bar);

ctrl_N = 10;
alpha_range = linspace(state_set.alpha_lim(1), state_set.alpha_lim(2), ctrl_N);
% Note that LMI1 is affine w.r.t to u and q; therefore, evaluation at the
% vertices is enough
q_range = [state_set.q_lim(1), state_set.q_lim(2)];
for i = 1:length(alpha_range)
    % x: theta, alpha, q
    x0 = [randn(1);alpha_range(i);rand(1)];   
    w_poly0 = w_poly_fcn(x0);
    W = W_list{1}*w_poly0(1);
    for s=2:length(w_poly0)
        W = W+ W_list{s}*w_poly0(s);
    end
    %W pos def  & W upper bound
    prog = prog.withPSD(W - w_lower*eye(n));     
    prog = prog.withPSD(W_bar - W);     
    for j = 1:length(q_range) 
            x0(3) = q_range(j);
            %CCM condition
            tmp = A_fcn(x0)*W;
            dw_poly_dt0 = dw_poly_dt_fcn(x0);  
            % PSD constraint: equation (11)            
            dW_dt = W_eval(dw_poly_dt0); % note that the dependence u will disappear after multiplication by B_perp' and its transpose if the top left block does not depends on any states
            CCM_pos = plant.B_perp'*(dW_dt-(tmp+tmp')-2*lambda*W)*plant.B_perp;
            prog = prog.withPSD(CCM_pos - ccm_eps*eye(n-nu));                
    end
end

%Norm constraint
free_vars = [prog.coneVar(2:end); prog.freeVar];
len = length(free_vars);
[prog, a] = prog.newPos(len);
prog = prog.withPos(-free_vars + a);
prog = prog.withPos(free_vars + a);

options = spot_sdp_mosek_options();
% options.solver_options.mosek.MSK_IPAR_BI_CLEAN_OPTIMIZER = 'MSK_OPTIMIZER_INTPNT'; % Use just the interior point algorithm to clean up
% options.solver_options.mosek.MSK_IPAR_INTPNT_BASIS = 'MSK_BI_NEVER'; % Don't use basis identification (it's slow)
options.verbose = 1;
        
disp('LMI formulation finished! Solving...');
SOS_soln = prog.minimize(norm_scale*sum(a) + w_upper*1e-3, @spot_mosek, options);

try
    solved = (strcmp(SOS_soln.info.solverInfo.itr.prosta, 'PRIMAL_AND_DUAL_FEASIBLE'));% && ...
%            strcmp(SOS_soln.info.solverInfo.itr.solsta, 'OPTIMAL'));
catch
    solved = 0;
end

%% parse
if (solved == 0)
    disp('There may be some numerical issues...');
%     w_lower = nan;
%     w_upper = nan;
%     W_bar = nan(n);
%     cond_num_W_opt = nan;
%     W_coef = nan;
%     return;   
else
    disp('feasible, getting results...');
end

w_lower = double(SOS_soln.eval(w_lower));
w_upper = double(SOS_soln.eval(w_upper));
cond_num_W_opt = w_upper/w_lower;
W_bar = clean(double(SOS_soln.eval(W_bar)),1e-4);

len_w_poly = length(w_poly);
W_coef = zeros(n,n,len_w_poly);
NNZ_list = zeros(len_w_poly,1);
for i = 1:len_w_poly
    W_coef(:,:,i) = clean(double(SOS_soln.eval(W_list{i})),1e-7);
    if sum(sum(abs(W_coef(:,:,i)))) > 0
        NNZ_list(i) = 1;
    end
end
%     
w_poly = w_poly(NNZ_list==1);
W_coef = W_coef(:,:,NNZ_list==1);

fprintf('%d non-zero monomials\n',length(w_poly));

%     %save in coefficient form (for copying over to C++)
%     p = W_poly_mat(find(NNZ_list),:);
%     save('Quad_Metric_Coeffs.mat','W_sol','p');
end

