syms g Ix Iy Iz m s

lqr_flag = 1;

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
    zeros(3,9) eye(3)];
B = [zeros(8,4);
    -1/m 0 0 0;
    0 1/Ix 0 0;
    0 0 1/Iy 0;
    0 0 0 1/Iz];

% compute jordan form
J = jordan(A);

% numerical matrices for subsequent operations
A = double(vpa(subs(A,[m Ix Iy Iz g], [0.547, 0.0033, 0.0033, 0.0058, 9.81])));
B = double(vpa(subs(B,[m Ix Iy Iz g], [0.547, 0.0033, 0.0033, 0.0058, 9.81])));

% observer decomposition
I = eye(12);
Tinv = [I(:,1:6) I(:,10:12) I(:,7:9)];
T = inv(Tinv);
p = 9;
n = 12;
Ao = T*A*Tinv;
A_11 = Ao(1:p,1:p);
A_12 = Ao(1:p,p+1:end);
A_21 = Ao(p+1:end,1:p);
A_22 = Ao(p+1:end,p+1:end);
CT = C*Tinv;
C_1 = CT(:,1:p);
TB = T*B;
B_1 = TB(1:p,:);
B_2 = TB(p+1:end,:);
iC_1 = inv(C_1);


if ~lqr_flag
%% Do pole placement
% Compute pole placement controller
% Ldesired = [-1, -1, -1, -1, -2, -2, -1, -1, -1, -1, -1, -1]-rand(1,12);
Ldesired = [-1 -1.2 -1.4 -1.6 ...
    -2 -2.2 -2.4 -2.6 ...
    -3 -3.2 -3.4 -3.6];

% F = compute_fb_multi(A,B,Ldesired) %produces poorly conditioned
% eigenvectors
F = -place(A,B,Ldesired);
eig(A+B*F);

% Compute pole placement minimal observer
L2desired = [-3, -2, -2.5];
L_2 = -place(A_22',A_12',L2desired)';

else
%% Do LQR
% Compute LQR controller
Q = C'*C;
R = diag([1,1,1,1]);
F = -lqr(A,B,Q,R);
% Compute LQR observer
Q = eye(3);
R = diag(ones(1,9));
L_2 = -lqr(A_22',A_12',Q,R)';
end

observer = struct();
observer.M = A_22+L_2*A_12;
observer.N = B_2+L_2*B_1;
observer.P = (A_21 +L_2*A_11-A_22*L_2-L_2*A_12*L_2)*iC_1;
observer.Q = Tinv*[zeros(p,(n-p)); eye(n-p)];
observer.R = 0;
observer.S = Tinv*[iC_1;
    -L_2*iC_1];
observer.C = C;
save F.mat F observer

