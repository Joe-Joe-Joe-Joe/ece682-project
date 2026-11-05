function visualize(t,y)
% visualization functions 
R = diag([1 -1 -1]);
y(:,1:3) = y(:,1:3)*R'; %convert to NWU

endpoints = [y(1,1:3); y(end,1:3)];

plot3(y(:,1),y(:,2),y(:,3));
hold on;
% plot3(endpoints(1,1), endpoints(1,2) , endpoints(1,3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(endpoints(2,1), endpoints(2,2) , endpoints(2,3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot3([0],[0],[0], 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
hold off;
view(-45,45);
xlabel("x")
ylabel("y")
zlabel("z")
grid on;
lims = [-5 5];
xlim(lims)
ylim(lims)
zlim(lims)

figure;
plot(t,y(:,4))
hold on
plot(t,y(:,5))
plot(t,y(:,6))
hold off
xlabel("t [s]")
ylabel("angle [rad]")
legend("\phi","\theta","\psi")
figure(1)

end