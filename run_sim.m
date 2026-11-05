params = [0.547, 0.0033, 0.0033, 0.0058, 9.81]; % numerical params from Dong et al
m = params(1);
g = params(5);
wind = zeros(6,1);

% x_0 = [0 0 0 ...
%     0 -0.01 0 ...
%     0 0 0 ...
%     -0.001 0 0]; % slight curve

x_0 = [0 0 0 ...
    0 0 0 ...
    0 0 0 ...
    0 0 0];

% u = [m*g+0.1 0 0 0]';
u_eq = [m*g 0 0 0]';

% f = @(t,x) (quad_dynamics_inertial(x,u,wind,params));
F = load("F.mat").F;
f = @(t,x) (quad_dynamics_inertial(x,u_eq,generate_wind(t),params,F));

% [t,y] = ode45(f,[0, 1],x_0); % y in NED inertial coordinates
tend = 25;
opts = odeset('Stats','on');
[t,y] = ode45(f,[0, tend],x_0,opts); % y in NED inertial coordinates

%% Visualize data
close all;
visualize(t,y)

%% local function
function [wind] = generate_wind(t)
    if t < 5
        wind = [0.2 0.2 0 0 0 0]';
    else
        wind = zeros(6,1);
    end
end