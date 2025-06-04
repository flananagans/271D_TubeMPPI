%% Script to plot a previously saved trajectory

clear
close all
clc

initWorkspace();

%% Load data
load('MPPI_highnoise_lqr.mat');

%% Plot the trajectory
figure();
try
track.resetPlotter();
catch
end
track.plotTrack()

x_hist(x_hist == 0) = NaN;
plot(x_hist(1,:), x_hist(2,:))
scatter(0,0, 15, 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
xlabel('x position (m)')
ylabel('y position (m)')
xlim([-5, 1])
ylim([-3, 3])

%% Plot velocity over time
figure();

load('MPPI_highnoise_lqr.mat');
t = linspace(0, t, length(x_hist(1,:)));
speed = sqrt(x_hist(3,:).^2 + x_hist(4,:).^2);
plot(t, speed)
hold on;

load('MPPI_highnoise_iLQG.mat');
t = linspace(0, t, length(x_hist(1,:)));
speed = sqrt(x_hist(3,:).^2 + x_hist(4,:).^2);
plot(t, speed)

legend({'LQR', 'iLQG'}, 'Box', 'off');
yline(MPPI.v_des, '--', 'v_{des}', 'Alpha', 0.5);
xlabel('time (s)')
ylabel('speed (m/s)')
ylim([0, 3])

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