%% Script to test discrete linear system point
% experimental for running on gpu

clear
close all
clc

%% Problem definition variables
k = 100; % number of paths to investigate
t = 25; % number of time steps

%% Initialize all of our systems
pts = cell(1, k); % array of systems for each path

for j = 1:k
    pts{j} = DiscreteLinearSystem(); % construct the object
    pts{j}.x(3:4) = [0.25, 0.25]; % initial velocity
end

%% Roll trajectories forwards in time
pts_gpu = gpuArray(pts); % ERRORS - GPU array only takes fundamental or logical data types
xs = arrayfun(@(x) rolloutTrajectory(x, t), pts_gpu, 'UniformOutput', false);

%% Plot trajectories
figure()
scatter(0, 0, 5, 'k', 'filled');
hold on
for j = 1:k
    plot(xs{j}(1, :), xs{j}(2,:));
end

%%%%%%%%%%%%%%% HELPER FUNCTIONS %%%%%%%%%%%%%%%%
function x = rolloutTrajectory(pt, t)
% Roll trajectory forwards in time
    x = zeros(size(pt{1}.x, 1), t);

    E = pt{1}.sampleControlNoise(t); % sample control noise for each time
    for i = 1:t
        pt{1}.setControl(pt{1}.u + E(:, i)); % add control noise to explore the space
        pt{1}.updateState(); % update state to roll forward
        x(:, i) = pt{1}.x; % record the current state
    end
end