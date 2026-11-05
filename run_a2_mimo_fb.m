syms g Ix Iy Iz m s

% numerical params from Dong et al
% g = 9.81;
% Ix = 0.0033;
% Iy = 0.0033;
% Iz = 0.0058;
% m = 0.547;
% [m Ix Iy Iz g], [0.547, 0.0033, 0.0033, 0.0058, 9.81]

euler_to_uvw = [0 -g 0;
    g 0 0;
    0 0 0];

A = [zeros(3) zeros(3) eye(3) zeros(3,3);
    zeros(3) zeros(3) zeros(3) eye(3);
    zeros(3) euler_to_uvw zeros(3) zeros(3);
    zeros(3) zeros(3) zeros(3) zeros(3)];
C = [eye(6) zeros(6);
    zeros(6) [zeros(3) zeros(3);
    zeros(3) eye(3)]];
B = [zeros(8,4);
    -1/m 0 0 0;
    0 1/Ix 0 0;
    0 0 1/Iy 0;
    0 0 0 1/Iz];

A = double(vpa(subs(A,[m Ix Iy Iz g], [0.547, 0.0033, 0.0033, 0.0058, 9.81])))
B = double(vpa(subs(B,[m Ix Iy Iz g], [0.547, 0.0033, 0.0033, 0.0058, 9.81])))
Ldesired = [-3, -3, -2+0.5i, -2-0.5i, -2, -2, -1, -1, -1, -1, -1, -1]-randn(1,12)*0;
% Ldesired = [-1 -1.2 -1.4 ...
%     -3 -3.2 -3.4 ...
%     -4 -4.2 -4.4 ...
%     -5 -5.2 -5.4];

F = compute_fb_multi(A,B,Ldesired)
eig(A+B*F)

save F.mat F