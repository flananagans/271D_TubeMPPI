%% Script to test discrete linear system point
clear
close all
clc

%% Problem definition variables
k = 100; % number of paths to investigate
t = 25; % number of time steps

%% Initialize all of our systems
pts = cell(1, k); % array of systems for each path
xs = cell(size(pts)); % history of states for each system
for j = 1:k
    pts{j} = DiscreteLinearSystem(); % construct the object
    pts{j}.x(3:4) = [0.25, 0.25]; % initial velocity
    xs{j} = zeros(length(pts{j}.x), t); % state history
end

%% Roll trajectories forwards in time
for i = 1:t
    for j = 1:k
        pts{j}.addControlNoise(); % add control noise to explore the space
        pts{j}.updateState(); % update state to roll forward
        xs{j}(:, i) = pts{j}.x; % record the current state
    end
end

%% Plot trajectories
figure()
scatter(0, 0, 5, 'k', 'filled');
hold on
for j = 1:k
    plot(xs{j}(1, :), xs{j}(2,:));
end