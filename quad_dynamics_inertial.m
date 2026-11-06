function [x_] = quad_dynamics_inertial(t,x,ctl,wind,params,F,observer)
%x
% Eq. 2.24 in Sabatino with state vector as 
% x = [(pos(I), [roll, pitch, yaw] (I), vel(I), ,ang velocity(B)]
%   ctl = u = [ft tau_x tau_y tau_z]
%   params = [m Ix Iy Iz g]
%   wind(disturbance) = [fwx fwy fwz tau_wx tau_wy tau_wz] is ignored...
%
% we make the simplifying assumption that d[roll pitch yaw]/dt = [p q r]
% for calculating the angular accelerations

% observer is an additional 3 states tacked on

m = params(1);
Ix = params(2);
Iy = params(3);
Iz = params(4);
g = params(5);

[R, T] = body_to_inertial(x(4),x(5),x(6));

x_ = zeros([15 1]);

p = x(10);
q = x(11);
r = x(12);

x_(1:3) = x(7:9); %internal dynamics
x_(4:6) = T*x(10:12);
x_(7:9) = [0 0 g]';
x_(10:12) = [(Iy-Iz)/Ix*r*q;
            (Iz-Ix)/Iy*p*r;
            (Ix-Iy)/Iz*p*q];

g = zeros(12,4); %control input
g(7:9,1) = R*[0 0 -1/m]';
g(10,2) = 1/Ix;
g(11,3) = 1/Iy;
g(12,4) = 1/Iz;

d = zeros(12,6); %disturbance input
d(7:9,1:3) = diag([1/m,1/m,1/m]); %forces in inertial frame, dont need to be moved
d(10:12,4:6) = [1/Ix 0 0; %torques in body frame, dont need to be moved
                0 1/Iy 0;
                0 0 1/Iz];

y = observer.C*x(1:12);
z = x(13:15);
x_hat = observer.Q*z+observer.S*y;

% ctl_ = F*x + ctl; %full state feedback control
ctl_ = F*x_hat + ctl; % estimator feedback control

x_(1:12) = x_(1:12) + g*ctl_+d*wind;
x_(13:15) = observer.M*z+observer.N*(ctl_-ctl)+observer.P*y;

end


