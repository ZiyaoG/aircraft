% MAIN  --  Planar rotor  --  Minimal-Force trajectory
%
% Fin the minimal torque-squared trajectory to move the quad-rotor from an
% arbitrary state to the origin.

function soln = trajOpt_aircraft(plant,trajGen_config,initGuess,stateCst)

x0 = trajGen_config.x0;
xF = trajGen_config.xF;
duration =  trajGen_config.duration;
u_bnd  = trajGen_config.u_bnd;
x_bnd = trajGen_config.x_bnd; 
% include_dist_model = trajGen_config.include_dist_model;
% dist_model = trajGen_config.dist_model;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up function handles                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.func.dynamics = @(t,x,u) dyn_fcn(t,x,u,plant.f_fcn,plant.B,plant.df_dx_fcn); % add some disturbances later
problem.func.pathObj = @(t,x,u) pathObjective(u);  %Force-squared cost function
problem.func.bndObj = @(t0,x0,tF,xF) bndObjective(t0,x0,tF,xF); % Use cost function to define the terminal constraint


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up problem bounds                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = duration-10;
problem.bounds.finalTime.upp = duration+10;

problem.bounds.initialState.low = x0;
problem.bounds.initialState.upp = x0;
% problem.bounds.finalState.low = -inf*ones(6,1);
% problem.bounds.finalState.upp = inf*ones(6,1);
problem.bounds.finalState.low = xF-0.01;
problem.bounds.finalState.upp = xF+0.01;

problem.bounds.state.low = x_bnd(:,1);
problem.bounds.state.upp = x_bnd(:,2);

problem.bounds.control.low = u_bnd(:,1);
problem.bounds.control.upp = u_bnd(:,2);
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Initial guess at trajectory: a straightline          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.guess.time = initGuess.time;
problem.guess.state = initGuess.state;
problem.guess.control = initGuess.control;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Solver options                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.options.nlpOpt = optimset(...
    'Display','iter','DerivativeCheck','on',...
    'MaxFunEvals',1e6,'MaxIter',200,'TolFun',1e-5,'TolCon',1e-5,...
    'GradConstr','on','GradObj','on');
% optimoptions('fmincon',...
%  'SpecifyConstraintGradient',true,'SpecifyObjectiveGradient',true,...
%  'Display','iter','ConstraintTolerance',1e-6,'OptimalityTolerance',1e-4,...
%  'MaxFunctionEvaluations',1e6);


% problem.options.method = 'trapezoid'; 
% problem.options.trapezoid.nGrid = 50;

problem.options.method = 'hermiteSimpson';  
problem.options.hermiteSimpson.nSegment = 40;


% problem.options.method = 'chebyshev';  
% problem.options.verbose = 3; % How much to print out?
% problem.options.chebyshev.nColPts = 10;  %method-specific options

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve!                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
soln = optimTraj(problem);
% save('TrajOpt','soln','x0','xF','duration')

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Display Solution                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Unpack the simulation
dt = 0.001;
ts = soln.grid.time(1):dt:soln.grid.time(end); 
states = soln.interp.state(ts);
inputs = soln.interp.control(ts);
% %-----------------------------------------------------------------------
end
function [obj, objGrad] = pathObjective(u)
% [obj, objGrad] = pathObjective(u)
%
% Computes the objective function (and gradients)
obj = u.^2;
if nargout == 2  % Analytic gradients
    nTime = length(u);    
    objGrad = zeros(5,nTime); %5 = [time + 3 states + 1 input];    
    objGrad(5,:) = 2*u;  %gradient obj wrt u
end
end

function [dx, dxGrad] = dyn_fcn(t,x,u,f_fcn,B,df_dx_fcn)
if nargin<6
    include_dist_model = 0;
    dist_model = @(x) [0 0]';
end
dx = f_fcn(x)+B*u;
if nargout == 2   % Analytic gradients
    nTime = length(u);
    dxGrad = zeros(3,5,nTime); %5 = [time + 3 states + 1 input];
    for i = 1:nTime
        dxGrad(1:3,2:4,i) = df_dx_fcn(x(:,i));
        dxGrad(1:3,5,i) = B;
    end
end
end

function [J, JGrad] = bndObjective(t0,x0,tF,xF)
    J = tF*5;
    if nargout >=2        
        JGrad = zeros(1,8);  % 2 time instants + 6 states
        JGrad(5) =5;        
    end
end