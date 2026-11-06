syms g Ix Iy Iz m s psi

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
     zeros(3,9) eye(3)];
B = [zeros(8,4);
    -1/m 0 0 0;
    0 1/Ix 0 0;
    0 0 1/Iy 0;
    0 0 0 1/Iz];
D = zeros(12,4);

I = eye(size(A));
G = C*inv(s*I-A)*B; %transfer matrix
chp = det(s*I-A); %ch. polynomial = s^12

N = s^4*G;
d_s = s^4;

m = size(N,2);
p = size(N,1);


%% zeros
% m=3;
% p=3;
% N = [0 1 0; 0 2 1;0 1 2]
psi = cell(min(m,p),1);

for k = 1:min(m,p)
    for i = 1:max(p-k+1,1)
        for j = 1:max(m-k+1,1)
            d = det(N(i:i+k-1,j:j+k-1));
            % psi{k-1} = [psi{k-1}; i, j, d];
            if ~isAlways(d == 0)
                psi{k} = [psi{k}; i, j, d];
            end
        end
    end
end

smith = [diag([1 1 s^2 s^6]); zeros(5,4)];
smithmcmillan = smith/d_s