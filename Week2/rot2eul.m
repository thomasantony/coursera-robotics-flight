function eul2 = rot2eu( R, varargin )
    R33 = R(3,3);
    R32 = R(3,2);
    R31 = R(3,1);
    R23 = R(2,3);
    R13 = R(1,3);
    
    theta = acos(R33);
    psi = atan2(R32/sin(theta), -R31/sin(theta));
    phi = atan2(R23/sin(theta),  R13/sin(theta));
   
    eul2 = [phi; theta; psi];
end

