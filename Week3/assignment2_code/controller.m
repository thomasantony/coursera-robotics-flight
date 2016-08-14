function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% FILL IN YOUR CODE HERE
Kp_z = 200;
Kv_z = 20;
Kp_y = 10;
Kv_y = 10;

% This loop takes phi_c and phidot_c and produces u2
Kp_phi = 1000; % Inner loop must be very fast compared to outer loop
Kv_phi = 50;

g = params.gravity;

y = state.pos(1);
z = state.pos(2);
ydot = state.vel(1);
zdot = state.vel(2);

y_c = des_state.pos(1);
z_c = des_state.pos(2);
ydot_c = des_state.vel(1);
zdot_c = des_state.vel(2);

yacc_c = des_state.acc(1);
zacc_c = des_state.acc(2);

phi = state.rot(1);
phidot = state.omega(1);

e_z = z_c - z;
e_zdot = zdot_c - zdot;

e_y = y_c - y;
e_ydot = ydot_c - ydot;

% Compute phi_c and phidot_c using position controller

phi_c = -(1/g)*(yacc_c + Kp_y*e_y + Kv_y*e_ydot);
phidot_c = 0;

%  Position controller computes u1
u1 = params.mass*(zacc_c + Kp_z*e_z + Kv_z*e_zdot + g);
% Use phi_c and phidot_c to compute u2
u2 = params.Ixx*(Kp_phi*(phi_c - phi) + Kv_phi*(phidot_c - phidot));

end

