%% Script to run for homework 2 simulation
%   simple implementation of our system

clear
close all
clc

%% Settings
has_high_noise = false; % actual control noise 10x modeled
has_ancillary = true; % activate ancillary controller

%% Initialize the workspace and include folders
initWorkspace();

%% Filename to save to
fname = 'MPPI';
if(has_high_noise)
    fname = [fname, '_highnoise'];
else
    fname = [fname, '_lownoise'];
end
if(has_ancillary)
    fname = [fname, '_lqr'];
end

%% Simulation setup
T_sim = 100; % time duration of simulation
dt = 0.01; % sampling period of simulation

% create physical system model
car = DiscreteLinearSystem(); % right now this is a double integrator
car.setDt(dt);

% set the car's actual noise
if(has_high_noise)
    car.sigma_control = 10*car.sigma_control;
end

%% Create track
% ring track from paper point mass example
or = 2.125;
ir = 1.875;
track = OvalTrack(or-ir, ir, 0);

%% Controller setup
%MPPI
f_MPPI = 1/(2*dt); % frequency of MPPI controller
K_MPPI = 100; % number of trajectories to rollout
T_MPPI = 2; % 2 second time-horizon
v_des = 2; % m/s desired velocity

% create MPPI controller instance and add control/environment noise
car_MPPI = DiscreteLinearSystem();
car_MPPI.setDt(1/f_MPPI); % set sampling time to match MPPI frequency

% MPPI controller
MPPI = MPPI_Controller(car_MPPI, track, K_MPPI, T_MPPI);
MPPI.v_des = v_des; % set desired speed

% iLQG
f_iLQG = 1/dt; % frequency of the iLQG controller

% create iLQG controller instance
car_iLQG = DiscreteLinearSystem();
car_iLQG.setDt(1/f_iLQG); % set sampling time to match iLQG frequency

ilqg = iLQG_hw(car_iLQG);

% create PID instance
Kp = eye(4);
Kd = eye(4);
Ki = eye(4); %zeros(4)
PID = PID_Controller(Kp,Ki,Kd,dt);

%create LQR instance
Q = diag([1,1,1,1]);
R = diag([0.1,0.1]);
A = car_iLQG.A;
B = car_iLQG.B;
K = dlqr(A,B,Q,R);

%% Run the simulation
figure('Name', 'HW2 Oval Track');
track.plotTrack();
mppi_run_flag = 0;
xlim([-5, 1]);
ylim([-3, 3]);

% keep track of trajectories (t = 0 -> t = N)
x_hist = zeros(length(car.x), length(0:1/f_iLQG:T_sim) + 1);
u_ilqg_hist = zeros(length(car.u), length(x_hist(1,:)));
u_mppi_hist = zeros(length(car.u), length(x_hist(1,:)));
u_tot_hist = zeros(length(car.u), length(x_hist(1,:)));
x_mppi_hist = zeros(length(car.x), length(x_hist(1,:)));
x_mppi_traj_hist = cell(1, length(x_hist(1,:))); % the chosen rollout
t_step = 1;
for t = 0:1/f_iLQG:T_sim
    if(mod(floor(1000*t), floor(1000*MPPI.dt)) == 0)
        %% Update MPPI at a slower rate to get the current input
        
        [u_mppi, x_mppi] = MPPI.RunMPPI(car.x); % Shirley--> whatever this function
                                                % is called
        MPPI.plotController(); % function to plot all trajectories
    end

    %% Get input for this step from iLQG
    if(has_ancillary)
        state_error = x_mppi(:, 1) - car.x; % x0 for iLQG
        u_ilqg = K*state_error; %ilqg.solve(state_error, u_mppi(:, 1));
    else
        u_ilqg = 0;
    end

    %% Update state of the actual system given the inputs from MPPI and iLQG
    car.setControl(u_ilqg + u_mppi(:, 1) + car.sampleControlNoise(1));
    x_hist(:, t_step + 1) = car.updateState();

    % Save our trajectories
    x_mppi_hist(:, t_step + 1) = x_mppi(:, 1);
    x_mppi_traj_hist{:, t_step + 1} = MPPI.x_traj;
    u_mppi_hist(:, t_step) = u_mppi(:, 1);
    u_ilqg_hist(:, t_step) = u_ilqg(:, 1);
    u_tot_hist(:, t_step) = car.u;

    %% Plot car
    %track.plotTrack();
    car.plotSystem(); % plot position and orientation of our car
    drawnow();

    %% Stop if way outside
    if(car.x(1) > 1 || car.x(1) < -5 || abs(car.x(2)) > 3)
        break;
    end


xlim([-5, 1]);
ylim([-3, 3]);
    t_step = t_step + 1;
end

%% Save everything 
save(fname);