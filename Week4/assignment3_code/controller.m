function [F, M] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
Kp = [10; 10; 800];
Kd = [10; 10; 25];

Kp_rot = 5*[300; 300; 300];
Kd_rot = [2; 2; 2];

g = params.gravity;
m = params.mass;

err_v = des_state.vel-state.vel;


ncap = des_state.vel/norm(des_state.vel); % Unit tangent to trajectory 
tcap = des_state.acc/norm(des_state.acc); % Unit normal to trajectory
bcap = cross(tcap, ncap);
delta_pos = (des_state.pos-state.pos);
if(any(isnan(bcap)))
    err_p = delta_pos;
else
    err_p = (delta_pos'*ncap)*ncap + (delta_pos'*bcap)*bcap;
end
rdotdot_c = des_state.acc + ...
           Kd.*(err_v) + ...
           Kp.*(err_p);

% if norm(delta_pos) > 1
%     keyboard;
% end
% Thrust (only for z)
F = m*(g + rdotdot_c(3));

% Moment
psi_des = des_state.yaw;
phi_des = (rdotdot_c(1)*sin(psi_des) - rdotdot_c(2)*cos(psi_des))/g;
theta_des = (rdotdot_c(1)*cos(psi_des) + rdotdot_c(2)*sin(psi_des))/g;

rot_des = [phi_des; theta_des; psi_des];
omega_des = [0; 0; des_state.yawdot];

M = Kp_rot.*(rot_des-state.rot) + ...
    Kd_rot.*(omega_des-state.omega);
% if abs(state.rot(2)) > pi/3
%     keyboard
% end
% =================== Your code ends here ===================

end
