function [R,T] = body_to_inertial(roll,pitch,yaw)
%BODY_TO_EARTH Summary of this function goes here
%   R = R_z(yaw)*R_y(pitch)*R_x(roll)

R_x = [1 0 0;
    0 cos(roll) -sin(roll);
    0 sin(roll) cos(roll)];

R_y = [cos(pitch) 0 sin(pitch);
        0,       1,           0;
        -sin(pitch) 0 cos(pitch)];

R_z = [cos(yaw) -sin(yaw) 0;
        sin(yaw) cos(yaw) 0;
        0,       0,      1];

R = R_z*R_y*R_x;
T = [1 sin(roll)*tan(pitch) cos(roll)*tan(pitch);
    0 cos(roll) -sin(roll);
    0 sin(roll)/cos(pitch) cos(roll)/cos(pitch)];
end