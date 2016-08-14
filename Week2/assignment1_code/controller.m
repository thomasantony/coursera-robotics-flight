function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

% persistent last_err
% 
% if isempty(last_err)
%     last_err = 0;
% end

% FILL IN YOUR CODE HERE
Kp = 130;
Kv = 20;

err = s_des(1) - s(1);
d_err = s_des(2) - s(2);
u = params.mass*(Kp*err + Kv*d_err + params.gravity);

% last_err = err;
end

