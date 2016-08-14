function [ R ] = rodriguez( u, phi )
    % Applies Roriguez rotation equation to get rotation matrix form
    %  axis and angle of rotation
    uhat = [0, -u(3), u(2);
            u(3), 0, -u(1);
            -u(2), u(1), 0];
    R = eye(3) * cos(phi) + u*u'*(1-cos(phi)) + uhat * sin(phi);
    
end

