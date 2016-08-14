function [u, phi] = rot2ang(R)
    % Converts rotation matrix to axis and angle of rotation
    phi = acos((trace(R) - 1)/2);
    uhat = 1/(2*sin(phi))*(R - R');
    u = [uhat(3,2); uhat(1,3); uhat(2,1)];
end