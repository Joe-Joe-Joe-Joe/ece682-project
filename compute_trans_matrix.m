syms g Ix Iy Iz m s

% numerical params from Dong et al
% g = 9.81;
% Ix = 0.0033;
% Iy = 0.0033;
% Iz = 0.0058;
% m = 0.547;
% [g Ix Iy Iz m], [9.81, 0.0033, 0.0033, 0.0058, 0.547]

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
D = zeros(12,4);

I = eye(size(A));
G = C*inv(s*I-A)*B %transfer matrix
chp = det(s*I-A); %ch. polynomial = s^12

zer = [A B; C zeros(12,4)];
%compute transmission zeros somehow
