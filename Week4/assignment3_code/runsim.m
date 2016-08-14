close all;
clear;

addpath('utils');
%% pre-calculated trajectories
% trajhandle = @traj_line;
trajhandle = @traj_helix;

%% Trajectory generation with waypoints
%% You need to implement this
waypoints = [0    0   0;
             1    1   1;
             2    0   2;
             3    -1  1;
             4    0   0
             ]';
trajhandle = @traj_generator;
trajhandle([],[],waypoints);

% 
% pos = zeros(101, 3);
% vel = zeros(101, 3);
% i = 1;
% tspan = linspace(0, 13.8564, 101);
% for t = tspan
%     des = trajhandle(t, []);
%     pos(i,:) = des.pos;
%     vel(i,:) = des.vel;
%     i = i + 1;
% end
% plot3(pos(:,1),pos(:,2),pos(:,3));
% hold on;
% grid on;
% for i = 1:size(waypoints,2)
%     plot3(waypoints(1,i), waypoints(2,i), waypoints(3,i), 'rx')
% end
% return;

%% controller
controlhandle = @controller;

% Run simulation with given trajectory generator and controller
% state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]
[t, state] = simulation_3d(trajhandle, controlhandle);
