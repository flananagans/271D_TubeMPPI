%% Script to run for homework 2 simulation
%   simple implementation of our system


%% Controller setup
f_MPPI = 50; % frequency of MPPI controller
K_MPPI = 1200; % number of trajectories to rollout
T_MPPI = 2; % 2 second time-horizon

f_iLQG = 100; % frequency of the iLQG controller

%% Simulation setup
T_sim = 50; % time duration of simulation




%% Run the simulation
for t = 0:1/f_iLQG:T_sim








end