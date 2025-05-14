%% Script to run for homework 2 simulation
%   simple implementation of our system

clear
close all
clc

%% Initialize the workspace and include folders
initWorkspace();

%% Simulation setup
T_sim = 50; % time duration of simulation
dt = 0.01; % sampling period of simulation

% create physical system model
car = DiscreteLinearSystem(); % right now this is a double integrator
car.setDt(dt);

% set the car's actual noise
%car.sigma_control = 10*car.sigma_control;

%% Create track
% ring track from paper point mass example
or = 2.125;
ir = 1.875;
track = OvalTrack(or-ir, ir, 0);

%% Controller setup
%MPPI
f_MPPI = 50; % frequency of MPPI controller
K_MPPI = 1200; % number of trajectories to rollout
T_MPPI = 2; % 2 second time-horizon
v_des = 2; % m/s desired velocity

% create MPPI controller instance and add control/environment noise
car_MPPI = DiscreteLinearSystem();
car_MPPI.setDt(1/f_MPPI); % set sampling time to match MPPI frequency

% MPPI controller
MPPI = MPPI_Controller(car_MPPI, track, K_MPPI, T_MPPI);
MPPI.v_des = v_des; % set desired speed

% iLQG
f_iLQG = 100; % frequency of the iLQG controller

% create iLQG controller instance
car_iLQG = DiscreteLinearSystem();
car_iLQG.setDt(1/f_iLQG); % set sampling time to match iLQG frequency

ilqg = iLQG(car_iLQG);

%% Run the simulation
figure('Name', 'HW2 Oval Track');
track.plotTrack();
for t = 0:1/f_iLQG:T_sim

    if(mod(t, floor(f_iLQG/f_MPPI)) == 0)
        %% Update MPPI at a slower rate to get the current input
        
        [u_mppi, x_mppi] = MPPI.RunMPPI(); % Shirley--> whatever this function
                                           % is called
    end

    %% Get input for this step from iLQG
    % NIKI - check this
    state_error = x_mppi(:, 1) - car.x; % x0 for iLQG
    u_ilqg = ilqg.solve(state_error, u_mppi(:, 1)); 

    %% Update state of the actual system given the inputs from MPPI and iLQG
    car.setControl(u_ilqg + u_mppi(:, 1))
    car.updateStateNoisy();

    %% Plot everything
    track.plotTrack();
    %MPPI.plot(); % function to plot all trajectories
    %car.plot(); % plot position and orientation of our car
    drawnow();
end