%% Script to test discrete linear system rollout function
clear
close all
clc

initWorkspace();

%% Problem definition variables
k = 100; % number of paths to investigate
T = 2; % time horizon in seconds

%% Initialize all of our systems
pts = cell(1, k); % array of systems for each path
xs = cell(size(pts)); % all the state trajectories
us = cell(size(pts)); % all the input trajectories

for j = 1:k
    pts{j} = DiscreteLinearSystem(); % construct the object
    t_steps = floor(T/pts{j}.dt); % number of time steps

    pts{j}.x(3:4) = [0.25, 0.25]; % initial velocity
    xs{j} = zeros(length(pts{j}.x), t_steps); % state trajectories
    us{j} = zeros(length(pts{j}.u), t_steps); % input trajectories
end

%% Roll trajectories forwards in time
for j = 1:k
    % create an initial u trajectory or you can make this 
    % use the previous time steps (MPPI)
    u_init = zeros(length(pts{j}.u), floor(T/pts{j}.dt));

    % rollout the whole trajectory
    [xs{j}, us{j}] = pts{j}.rolloutTraj(u_init, T);
end

%% Plot state trajectories (a 'cone' because our initial state has  a velocity)
figure()
scatter(0, 0, 5, 'k', 'filled');
hold on
for j = 1:k
    plot(xs{j}(1, :), xs{j}(2,:));
end

%% Plot inputs (one color is one trajectory)
figure()
scatter(0, 0, 5, 'k', 'filled');
hold on
for j = 1:k
    scatter(us{j}(1, :), us{j}(2,:), 'filled');
end