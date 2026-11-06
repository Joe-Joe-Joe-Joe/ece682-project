function visualize(t,y)
% toggles
f1 = 1;
f2 = 1;
f3 = 0;
% visualization functions 
R = diag([1 -1 -1]);
y(:,1:3) = y(:,1:3)*R'; %convert to NWU

endpoints = [y(1,1:3); y(end,1:3)];

%% Set light theme
% Figure
set(groot,'defaultFigureColor','w');

% Axes
set(groot,'defaultAxesColor','w');
set(groot,'defaultAxesXColor','k');
set(groot,'defaultAxesYColor','k');
set(groot,'defaultAxesZColor','k');

% Gridlines
set(groot,'defaultAxesGridColor',[0.7 0.7 0.7]);
set(groot,'defaultAxesMinorGridColor',[0.85 0.85 0.85]);
set(groot,'defaultAxesGridAlpha',0.5);
set(groot,'defaultAxesMinorGridAlpha',0.3);

% Legends
set(groot,'defaultLegendColor','w');
set(groot,'defaultLegendTextColor','k');
set(groot,'defaultLegendEdgeColor','k');

% Font
set(groot,'defaultAxesXColor','k');
set(groot,'defaultAxesYColor','k');
set(groot,'defaultAxesZColor','k')

% Line thickness
set(groot,'defaultLineLineWidth',1.5);
set(groot,'defaultAxesLineWidth',1.0);

%% plotting
if f1
figure('Color','w','Position',[100 100 1750 900]);
plot3(y(:,1),y(:,2),y(:,3));
hold on;
% plot3(endpoints(1,1), endpoints(1,2) , endpoints(1,3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(endpoints(2,1), endpoints(2,2) , endpoints(2,3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot3([0],[0],[0], 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
hold off;
view(-45,45);
xlabel("x [m]")
ylabel("y [m]")
zlabel("z [m]")
grid on;
% lims = [-5 5];
% xlim(lims)
% ylim(lims)
% zlim(lims)
set(findall(gcf,'Type','ax'),'FontSize',20)
end

if f2
figure('Color','w','Position',[100 100 1750 900]);

tiledlayout(1,2, ...
    'TileSpacing','loose', ...
    'Padding','loose');

% Angles
nexttile
plot(t,y(:,4:6))
xlabel("t [s]")
ylabel("angle [rad]")
legend("\phi","\theta","\psi")
grid on

% Position
nexttile
plot(t,y(:,1:3))
xlabel("t [s]")
ylabel("position [m]")
legend("x","y","z")
grid on
end

if f3
% Z
figure('Color','w','Position',[100 100 1750 900]);
plot(t,y(:,13:15))
xlabel("t [s]")
legend("z_1","z_2","z_3")
grid on
end

set(findall(gcf,'Type','legend'),'FontSize',16)
set(findall(gcf,'Type','legend'),'Color','w','TextColor','k','EdgeColor','k')
set(findall(gcf,'Type','ax'),'FontSize',16)

%% export
% figure(1)
% exportgraphics(gcf,'/home/joe/Documents/ECE682/project/figures/full_state_fb_control_3d.png', 'Resolution',300);
% figure(2)
% exportgraphics(gcf,'/home/joe/Documents/ECE682/project/figures/full_state_fb_control.png','Resolution',300);
end