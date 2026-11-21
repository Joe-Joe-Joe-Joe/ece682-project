syms g Ix Iy Iz mass s

% numerical params from Dong et al
% g = 9.81;
% Ix = 0.0033;
% Iy = 0.0033;
% Iz = 0.0058;
% m = 0.547;

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
    -1/mass 0 0 0;
    0 1/Ix 0 0;
    0 0 1/Iy 0;
    0 0 0 1/Iz];

% numerical matrices for subsequent operations
A = double(vpa(subs(A,[mass Ix Iy Iz g], [0.547, 0.0033, 0.0033, 0.0058, 9.81])));
B = double(vpa(subs(B,[mass Ix Iy Iz g], [0.547, 0.0033, 0.0033, 0.0058, 9.81])));

mass = 0.547;
omega = 2*pi/10;
os = omega^2;
% tracking problem redefinition
A1 = A;
A3 = [zeros(6,7); 
    [0 0 0 0 1/mass 0 0; 
    0 0 0 0 0 1/mass 0; 
    0 0 0 0 0 0 1/mass];
    zeros(3,7)];
A2 = [0 1 0 0  0 0 0;
    -os 0 0 0 0 0 0;
    0 0 0 1 0 0 0;
    0 0 -os 0 0 0 0;
    0 0 0 0 0 0 0;
    0 0 0 0 0 0 0;
    0 0 0 0 0 0 0;];
C1 = C;
C2 = [eye(4) zeros(4,3)];
B1 = B;
D1 = [eye(2) zeros(2,10)];
D2 = [[-1 0 0; 0 0 -1] zeros(2,4)];
D = [D1 D2];

A = [A1 A3; zeros(7,12) A2];
C = [C1 zeros(9,7);
    zeros(4,12) C2];
B = [B1; zeros(7,4)];

% screw you matlab 
% non-resonance condition
r0 = rank([A1-eye(12)*0 B1; D1 zeros(2,4)]) == 14;
r1 = rank([A1-eye(12)*omega*1i B1; D1 zeros(2,4)]) == 14;
r2 =  rank([A1+eye(12)*omega*1i B1; D1 zeros(2,4)]);
assert (r0 && r1 && r2) 


% copy the code in the notes
% using lqr instead of pole placement :D
n1 = 12;
n2 = 7;
m = 4;
p = 13;
q = 2;

% step 0 - redefine output to y = [y0, e]
C1 = [C(1:9, 1:12); D1]; %turns out we dont need r_i_dot so drop those
C2 = [C(1:9, 13:end); D2];
%so the measurable exosystem state is captured entirely in the e part
%and the y0 part is the original measurable system states
p = 11; %since we drop 2 states, need to shorten p

% Step 1 − internal model 13 
pi2 = charpoly(A2);
Ac1 = compan(pi2); 
Ac1 = flip(Ac1, 1); 
Ac1 = flip(Ac1, 2);
Ac1 = kron(Ac1, eye(q));

Bce1 = [zeros(q*(n2-1), q);eye(q)]; % error feedback 
Bcy1 = zeros(q*n2, p-q); % remaining measured signals 
Bc1 = [Bcy1 Bce1];

% Step 2 − augmented system 
Aa = [A1 zeros(n1, n2*q); Bc1*C1 Ac1];
Ba = [B1;zeros(q*n2, m)];
Ca = [C1 zeros(p, n2*q); zeros(n2*q, n1) eye(n2*q)]; 

% observer−based stabilizer for augmented system
Qc = eye(n1+n2*q);
Rc = eye(m);
Fa = -lqr(Aa, Ba, Qc, Rc);
Fa1 = Fa(:,1:n1); 
Fa2 = Fa(:, n1+1:end);

Qo = eye(n1);
Ro = eye(p);
L1 = -lqr(A1', C1', Qo, Ro)';

Ac2 = A1 + B1*Fa1 + L1*C1;
Bc2 = -L1; 
Ac21 = B1*Fa2; %Ac3
Cc1 = Fa2; 
Cc2 = Fa1;
Dc = zeros(m,p);

%form final controller
Ac = [Ac1 zeros(q*n2,n1); Ac21 Ac2]; 
Bc = [Bc1;Bc2]; 
Cc = [Cc1 Cc2]; 
Dc = Dc;

% save into common format which I'm calling observer
% to not conflict with simple feedback controller F
observer = struct();
observer.Ac = Ac;
observer.Bc = Bc;
observer.Cc = Cc;
observer.Dc = Dc;
observer.QC = [C1 C2] % maps [x1 x2]' into [y0 e]'
observer.tracking_flag = 1;
observer.A2 = A2;
observer.QC(11,15) = 1; %TODO fix?
F = 0;

save F.mat observer F
