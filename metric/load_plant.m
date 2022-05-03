n = 3;                          % state dimension (ignoring psi)
nu = 1;                         % input dimension
nw = 1;                         % disturbance dimension
plant.g = 9.8066;                       % gravitational constant
x = msspoly('x',n); x_store = x;
u = msspoly('u',nu); plant.u= u;
deg2rad = pi/180;
      
% ------------------------- system dynamics -------------------------------
% x: theta (degree), alpha (degree), q (degree/s)

% use of Taylor series to approximate the sin function 
% SPOT does not support sin and cos function; so we use polynomials to
% approximate the sin function;
sin_fcn = @(x) x-x^3/6+x^5/120;

f = [x(3);x(3)-0.8*sin_fcn(2*x(2));-0.2*x(3)+0.8*sin_fcn(2*x(2))];

% f written as a function handle: can also work when x has multiple columns
% f_fcn = @(x)[x(3,:);x(3,:)-0.8*sin(2*x(2,:));-0.2*x(3,:)+0.8*sin(2*x(2,:))];
f_fcn = @(x)[x(3,:);x(3,:)-0.8*sin(2*x(2,:));-0.2*x(3,:)+0.8*sin(2*x(2,:))];  % k_q = 0.2, l_a = -1


B = [0;0;1]; 
B_perp = [1 0 0; 0 1 0]';

df_dx_fcn = @(x) [0 0 1;
                  0 -1.6*cos(2*x(2)) 1;
                  0 1.6*cos(2*x(2)) -0.2];

plant.n = n; plant.nu=nu; plant.nw = nw; 
plant.f_fcn = f_fcn; plant.B = B;
plant.B_fcn = @(x) B;
plant.dynamics = @(x,u) f_fcn(x)+ B*u;
plant.B_perp = B_perp;
plant.df_dx_fcn = df_dx_fcn;
plant.A_fcn = df_dx_fcn;
