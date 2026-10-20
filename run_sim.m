close all;
m = 1;
params = [m 1 1 1 9.81];
wind = zeros(6,1);
x_0 = [0 0 0 ...
    0 -0.1 0 ...
    0 0 0 ...
    -0.01 0 0];
u = [m*9.81+0.1 0 0 0]';

f = @(t,x) (quad_dynamics_inertial(x,u,wind,params));
[t,y] = ode45(f,[0, 10],x_0); % y in NED inertial coordinates
R = diag([1 -1 -1]);
y(:,1:3) = y(:,1:3)*R'; %convert to NWU

points = [y(1,1:3); y(end,1:3)];

plot3(y(:,1),y(:,2),y(:,3));
hold on;
plot3(points(:,1), points(:,2) , points(:,3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
hold off;
view(-45,45);
xlabel("x")
ylabel("y")
zlabel("z")
grid on;