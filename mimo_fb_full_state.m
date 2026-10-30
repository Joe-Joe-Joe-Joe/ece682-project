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
D = zeros(12,4);
I = eye(size(A));
chp = det(s*I-A); %ch. polynomial = s^12

%pole placement
F1 = [magic(4) zeros(4) zeros(4)] %step 2
A_ = A+B*F1;
B_ = B(:,1);
rank([A_ B_]); %=12, so (A_,B_) controllable

W = compute_W(A_,B_); %controllability matrix of (A_,B_)

%compute chp of A_
chp_ = det(s*I-A_);
coefficients = coeffs(chp_,s,"All");
c_reversed = flip(coefficients);
At = [zeros(11,1) eye(11); -c_reversed(1:12)];
Bt = [zeros(11,1); 1];
Wt = compute_W(At,Bt);

T = Wt*inv(W);
chp_desired = (s+1)^12; %desired closed-loop poles
cd_rev = flip(coeffs(chp_desired,s,"All"));
Ft = c_reversed(1:end-1) - cd_rev(1:end-1);
F2 = Ft*T;

% Af = A_ + B_*F2;
% simplify(det(s*eye(12)-Af)-(s+1)^12)
% Af = subs(Af, [g Ix Iy Iz m], [9.81, 0.0033, 0.0033, 0.0058, 0.547]);

F = F1 + [1 0 0 0]'*F2;
F_num = subs(F,[m Ix Iy Iz g], [0.547, 0.0033, 0.0033, 0.0058, 9.81]);
F_num = double(vpa(F_num));

save F.mat F_num