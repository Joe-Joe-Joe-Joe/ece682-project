function [x_] = quad_dynamics_exact(x,ctl,wind,params)

% Eq. 2.19 in Sabatino with state vector as x= [(pos(I), ang. (I), vel(B), ,ang velocity(B)]
%   ctl = u = [ft tau_x tau_y tau_z]
%   params = [m Ix Iy Iz g]
%   wind(disturbance) = [fwx fwy fwz tau_wx tau_wy tau_wz]

m = params(1);
Ix = params(2);
Iy = params(3);
Iz = params(4);
g = params(5);

[R, T] = body_to_inertial(x(4),x(5),x(6));

x_ = zeros([12 1]);

x_(1:3) = R*x(7:9);
x_(4:6) = T*x(10:12);

u = x(7);
v = x(8);
w = x(9);
p = x(10);
q = x(11);
r = x(12);

x_(7:9) = [r*v-q*w-g*sin(x(2))+wind(1)/m;
            p*w-r*u+g*sin(x(1))*cos(x(2))+wind(2)/m;
            q*u-p*v+g*cos(x(2))*cos(x(1))+wind(3)/m-ctl(1)/m];

x_(10:12) = [(Iy-Iz)/Ix*r*q+(ctl(2)+wind(4))/Ix;
            (Iz-Ix)/Iy*p*r+(ctl(3)+wind(5))/Iy;
            (Ix-Iy)/Iz*p*q+(ctl(4)+wind(6))/Iz];

end