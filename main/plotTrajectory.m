%% Script to plot a previously saved trajectory

clear
close all
clc

initWorkspace();

%% Load data
load('MPPI_lownoise_lqr.mat');

%% Plot the trajectory
figure();
track.plotTrack()

x_hist(x_hist == 0) = NaN;
plot(x_hist(1,:), x_hist(2,:))
scatter(0,0, 15, 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
xlabel('x position (m)')
ylabel('y position (m)')
xlim([-5, 1])
ylim([-3, 3])

%% High noise no ancillary, multiple starts

figure();
track.plotTrack()
for s_ind = 1:5
load(sprintf('MPPI_highnoise0%d.mat', s_ind));

%% Plot the trajectory
x_hist(x_hist == 0) = NaN;
plot(x_hist(1,:), x_hist(2,:))
end
scatter(0,0, 15, 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
xlabel('x position (m)')
ylabel('y position (m)')
xlim([-5, 1])
ylim([-3, 3])