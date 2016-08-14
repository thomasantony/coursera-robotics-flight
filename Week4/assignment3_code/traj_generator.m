function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

persistent waypoints0 traj_time d0 pos_coeffs vel_coeffs acc_coeffs
% if nargin > 2
%     d = waypoints(:,2:end) - waypoints(:,1:end-1);
%     d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
%     traj_time = [0, cumsum(d0)];
%     waypoints0 = waypoints;
% else
%     if(t > traj_time(end))
%         t = traj_time(end);
%     end
%     t_index = find(traj_time >= t,1);
% 
%     if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
%     if(t == 0)
%         desired_state.pos = waypoints0(:,1);
%     else
%         scale = t/d0(t_index-1);
%         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
%     end
%     desired_state.vel = zeros(3,1);
%     desired_state.acc = zeros(3,1);
%     desired_state.yaw = 0;
%     desired_state.yawdot = 0;
% end
% return;

    % 8n coefficients
    % [ [a07x, a07y, a07z],  % first segment
    %   [a06x, a06y, a06y],
    %          ....
    %   [a17x, a17y, a17y],  % second segment
    %   [a16x, a16y, a16y], 
    %          ....
    %  ]
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    traj_time(end)
    waypoints0 = waypoints;
    
    n = size(waypoints0,2)-1;
    A = zeros(8*n, 8*n);
    b = zeros(8*n, 3);
    
    % "Left" end of waypoint segment (t=0)
    %    p(t=0) = w_i
    %   -> a_i0 = w_i
    rowctr = 1;
    for i = 0:n-1
        A(rowctr,i*8+8) = 1;
        b(rowctr,:) = waypoints0(:,i+1)';
        rowctr = rowctr + 1;
    end
    coeffs_0 = ones(1,8);
    coeffs_1 = polyder(coeffs_0);
    coeffs_2 = polyder(coeffs_1);
    coeffs_3 = polyder(coeffs_2);
    coeffs_4 = polyder(coeffs_3);
    coeffs_5 = polyder(coeffs_4);
    coeffs_6 = polyder(coeffs_5);
    
    % List of polynomial coefficients up to 6th derivative
    dp_coeffs = {coeffs_1, coeffs_2, coeffs_3, ...
                 coeffs_4, coeffs_5, coeffs_6};
    
    % "Right" end of waypoint segment (t=1)
    %    p(t=1) = w_(i+1)
    for i = 0:n-1
        A(rowctr,i*8+1:i*8+8) = coeffs_0;
        b(rowctr,:) = waypoints0(:,i+2)';
        rowctr = rowctr + 1;
    end
    
    % pdot(S0) = pdot(Sn) = 0 -- boundary conditions
    A(rowctr,7) = 1; rowctr = rowctr + 1;
    A(rowctr,8*(n-1)+1:8*n) = [coeffs_1 0]; rowctr = rowctr + 1;
    % Acceleration = 0 at initial and terminal
    A(rowctr,6) = 1; rowctr = rowctr + 1;
    A(rowctr,8*(n-1)+1:8*n) = [coeffs_2 0 0]; rowctr = rowctr + 1;
    % Jerk = 0  at initial and terminal
    A(rowctr,5) = 1; rowctr = rowctr + 1;
    A(rowctr,8*(n-1)+1:8*n) = [coeffs_3 0 0 0]; rowctr = rowctr + 1;
    
    % Continuity of 1st-6th derivative
    % pdot_i(t=1) = pdot_(i+1) (t=0)
    for i = 0:n-2
        for k = 1:6
            A(rowctr,i*8+1:i*8+8) = [dp_coeffs{k} zeros(1,k)];
            A(rowctr,i*8+8 + (8-k)) = -1;
            rowctr = rowctr + 1;
        end
    end
    % Compute polynomial coefficients
    all_coeffs = A\b;
    
    pos_coeffs = cell(n,1);
    vel_coeffs = cell(n,1);
    acc_coeffs = cell(n,1);
    % Split coefficients into cell array
    % Also store velocity and acceleration coefficients
    for i=1:n
        cx = all_coeffs(((i-1)*8+1):i*8,1);
        cy = all_coeffs(((i-1)*8+1):i*8,2);
        cz = all_coeffs(((i-1)*8+1):i*8,3);
        pos_coeffs{i} = [cx, cy, cz];
        
        cx1 = polyder(cx)';
        cy1 = polyder(cy)';
        cz1 = polyder(cz)';
        vel_coeffs{i} = [cx1, cy1, cz1];
        
        cx2 = polyder(cx1)';
        cy2 = polyder(cy1)';
        cz2 = polyder(cz1)';
        acc_coeffs{i} = [cx2, cy2, cz2];
    end
else
    if(t > traj_time(end))
        t = traj_time(end)-0.001;
    end
    t_index = find(traj_time >= t,1)-1;
    if(t_index > 1)
        t = t - traj_time(t_index);
    end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
    else
        % Scaled time between 0 and 1
        scale = t/d0(t_index);
        
        % Get polynomial coefficients for current segment for x, y and z
        seg_coeffs = pos_coeffs{t_index};
        cx = seg_coeffs(:,1);
        cy = seg_coeffs(:,2);
        cz = seg_coeffs(:,3);
        
        desired_state.pos = [polyval(cx, scale);
                             polyval(cy, scale);
                             polyval(cz, scale);];
        % Compute value for polynomial for velocity
        seg_coeffs = vel_coeffs{t_index};
        cx1 = seg_coeffs(:,1);
        cy1 = seg_coeffs(:,2);
        cz1 = seg_coeffs(:,3);
        vel = [polyval(cx1, scale);
               polyval(cy1, scale);
               polyval(cz1, scale);];
        % because of chain rule, we have to divide by T
        desired_state.vel = vel.*(1/d0(t_index));

        % Compute value for polynomial for acceleration
        seg_coeffs = acc_coeffs{t_index};
        cx2 = seg_coeffs(:,1);
        cy2 = seg_coeffs(:,2);
        cz2 = seg_coeffs(:,3);
        
        acc = [polyval(cx2, scale);
               polyval(cy2, scale);
               polyval(cz2, scale);];
        % because of chain rule, we have to divide by T
        desired_state.acc = acc.*(1/d0(t_index)^2);
    end
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end

end
function [T] = polyT(n, k, t)
% One utility function we are going to build to help us with the above is 
% creating the polynom coefficient-coefficient vector (for lack of better 
% name, these are the actual values you would put into the matrix raws). 
% To understand what this mean here is an example: Lets say I want to get a
% vector of 8 variables (for a 7th order polynom) for the first derivative 
% when t=1. This utility function should return a vector of: 0 1 2 3 4 5 6 7.
% When we build matrix A we will use this utility function to create those 
% vector for us. n is the polynom number of coefficients, k is the requested 
% derivative and t is the actual value of t (this can be anything, not just 0 or 1).
T = zeros(n,1);
D = zeros(n,1);

% Init:
for i=1:n
    D(i) = i-1;
    T(i) = 1;
end

% Derivative:
for j=1:k
    for i=1:n
        T(i) = T(i) * D(i);
        if D(i) > 0
            D(i) = D(i) - 1;
        end
    end
end

% put t value
for i=1:n
    T(i) = T(i) * t^D(i);
end

T = T';

end
