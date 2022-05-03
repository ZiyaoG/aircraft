function [soln] = plan_traj_aircraft(plant,trajGen_config)
x0 = trajGen_config.x0;
xF = trajGen_config.xF;
duration = trajGen_config.duration;
% u_bnd = trajGen_config.u_bnd;
% x_bnd = trajGen_config.x_bnd;
% dist_model = trajGen_config.dist_model;
% include_dist_model = trajGen_config.include_dist_model;

% initial guess of the solution: important to find a feasible solution    
initGuess.time = [0,duration/2 duration];
initGuess.state = [x0, (x0+xF)/2 xF];
initGuess.control = ones(1,3)/2;
soln = trajOpt_aircraft(plant,trajGen_config,initGuess);   
fprintf(1,'cost = %.2f\n',soln.info.objVal);
