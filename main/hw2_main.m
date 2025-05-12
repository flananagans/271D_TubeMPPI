%% Script to run for homework 2 simulation
%   simple implementation of our system

%% Initialize the workspace and include folders
initWorkspace();

%% Controller setup
%MPPI
f_MPPI = 50; % frequency of MPPI controller
K_MPPI = 1200; % number of trajectories to rollout
T_MPPI = 2; % 2 second time-horizon
v_des = 2; % m/s desired velocity

% iLQG
f_iLQG = 100; % frequency of the iLQG controller



%% Simulation setup
T_sim = 50; % time duration of simulation

%% Create track
track = StraightTrack();

%% Run the simulation
for t = 0:1/f_iLQG:T_sim


    





end