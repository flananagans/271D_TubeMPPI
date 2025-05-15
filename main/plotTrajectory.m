%% Script to plot a previously saved trajectory

clear
close all
clc

initWorkspace();

%% Load data
load('MPPI_baseline.mat');

%% Plot the trajectory
figure();
track.plotTrack()

plot(x_hist(1,:), x_hist(2,:))
xlabel('x position (m)')
ylabel('y position (m)')