params = [0.547, 0.0033, 0.0033, 0.0058, 9.81]; % numerical params from Dong et al
m = params(1);
g = params(5);
wind = zeros(6,1);

% dynamics are 12 states
x_0 = [0.5 1 1 ...
    0.1 -0.1 1 ...
    0 0 0 ...
    -0.001 0 0]; % slight curve

% min obs is 3 states
z_0 = [0 0 0] 
x0 = [x_0 z_0];

data = load("F.mat");
F = data.F
observer = data.observer 
u_eq = [m*g 0 0 0]';
f = @(t,x) (quad_dynamics_inertial(t,x,u_eq,generate_wind(t),params,F,observer));

tend = 16;
opts = odeset('Stats','on');
[t,y] = ode45(f,[0, tend],x0,opts); % y in NED inertial coordinates

%% Visualize data
close all;
visualize(t,y)

%% local function
function [wind] = generate_wind(t)
    if t < 0
        wind = [0 0 -5 0 0 0]';
    else
        wind = zeros(6,1);
    end
end