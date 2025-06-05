%% Script to plot a previously saved trajectory

clear
close all
clc

initWorkspace();

figure();

%% Low noise, MPPI
subplot(2, 2, 1);

% load data
load('MPPI_lownoise.mat');
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
title('MPPI')
xlim([-5, 1])
ylim([-3, 3])

%% Low noise, MPPI + LQR
subplot(2, 2, 2);

% load data
load('MPPI_lownoise_lqr.mat');
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
title('MPPI + LQR')
xlim([-5, 1])
ylim([-3, 3])

%% High noise no ancillary, multiple starts
subplot(2,2,3);

try
track.resetPlotter();
catch
end
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

%% High noise, MPPI + LQR
subplot(2, 2, 4);

% load data
load('MPPI_highnoise_lqr.mat');
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

%% LQR vs iLQG
% Plot velocity over time
figure();
tiledlayout(2,2);

%% high noise, MPPI + LQR
% load data
nexttile(1)
load('MPPI_highnoise_lqr.mat');
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
title('MPPI + LQR');
xlim([-5, 1])
ylim([-3, 3])
axis equal

%% high noise, MPPI + iLQG
% load data
nexttile(2)
load('MPPI_highnoise_iLQG.mat');
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
title('MPPI + iLQG');
xlim([-5, 1])
ylim([-3, 3])
axis equal

%% Compare speeds
nexttile([1 2]);
load('MPPI_highnoise_lqr.mat');
t_lqr = linspace(0, t, length(x_hist(1,:)));
speed_lqr = sqrt(x_hist(3,:).^2 + x_hist(4,:).^2);
mean(speed_lqr)
plot(t_lqr, speed_lqr)
hold on;

load('MPPI_highnoise_iLQG.mat');
t_ilqg = linspace(0, t, length(x_hist(1,:)));
speed_ilqg = sqrt(x_hist(3,:).^2 + x_hist(4,:).^2);
mean(speed_ilqg)
plot(t_ilqg, speed_ilqg)

legend({'LQR', 'iLQG'}, 'Box', 'off', 'Location', 'northwest');
yline(MPPI.v_des, '--', 'v_{des}', 'Alpha', 0.5, 'LineWidth', 1.5);
xlabel('time (s)')
ylabel('speed (m/s)')
ylim([0, 3])
set(gca(), 'Box', 'off')