params = [0.547, 0.0033, 0.0033, 0.0058, 9.81]; % numerical params from Dong et al
m = params(1);
g = params(5);
wind = zeros(6,1);

data = load("F.mat");
F = data.F;
observer = data.observer ;

%initial conditions
% dynamics are 12 states
x_0 = [0.5 1 1 ...
    0.1 -0.1 pi ...
    0 0 0 ...
    -0.001 0 0]*0; % slight curve
x_0(1:2) = [1,0]

if observer.tracking_flag
    ex_0 = zeros(1,7);
    xc_0 = zeros(1,26);
    x0 = [x_0 ex_0 xc_0];
    x0(13) = 1;
    x0(16) = 6.25/10;
else
% min obs is 3 states
z_0 = [0 0 0]; 
x0 = [x_0 z_0];
end

u_eq = [m*g 0 0 0]';
f = @(t,x) (quad_dynamics_inertial(t,x,u_eq,generate_wind(t),params,F,observer));

tend = 30;
opts = odeset('Stats','on');
[t,y] = ode45(f,[0, tend],x0,opts); % y in NED inertial coordinates

%% Visualize data
close all;
visualize(t,y)

%% local function
function [wind] = generate_wind(t)
    if (4 < t) && (t < 0)
        wind = [3 5 -5 0 0 0]';
    else
        wind = zeros(6,1);
    end
end