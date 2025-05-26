%% Script to run for homework 2 simulation
%   simple implementation of our system

clear
close all
clc

%% Settings
has_high_noise = true; % actual control noise 10x modeled
has_ancillary = true; % activate ancillary controller

captureVideo = true;

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
car.x(4) = 0.1; % small positive Y velocity

% set the car's actual noise
if(has_high_noise)
    car.sigma_control = 10*car.sigma_control;
end

%% Create track
% ring track from paper point mass example
or = 2.25;
ir = 1.75;
track = OvalTrack(or-ir, ir, 1);

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

% Ancillary controller
f_anc = 1/dt; % frequency of the ancillary controller

% create iLQG controller instance
car_anc = DiscreteLinearSystem();
car_anc.setDt(1/f_anc); % set sampling time to match iLQG frequency

% create iLQG instance
%ilqg = iLQG_hw(car_iLQG);

% create PID instance
%Kp = eye(4);
%Kd = eye(4);
%Ki = eye(4); %zeros(4)
%PID = PID_Controller(Kp,Ki,Kd,dt);

%create LQR instance
Q = diag([1,1,1,1]);
R = diag([0.1,0.1]);
A = car_anc.A;
B = car_anc.B;
K = dlqr(A,B,Q,R);

%% Video writer to save video
if(captureVideo)
    v = VideoWriter([fname, '.avi']);
    v.FrameRate = f_anc;
    open(v);
end

%% Run the simulation
figure('Name', 'HW2 Oval Track');
track.plotTrack();
mppi_run_flag = 0;
xlim([-5, 1]);
ylim([-3, 5]);

% keep track of trajectories (t = 0 -> t = N)
x_hist = zeros(length(car.x), length(0:1/f_anc:T_sim) + 1);
u_anc_hist = zeros(length(car.u), length(x_hist(1,:)));
u_mppi_hist = zeros(length(car.u), length(x_hist(1,:)));
u_tot_hist = zeros(length(car.u), length(x_hist(1,:)));
x_mppi_hist = zeros(length(car.x), length(x_hist(1,:)));
x_mppi_traj_hist = cell(1, length(x_hist(1,:))); % the chosen rollout
t_step = 1;

% TO ADD:
% object position
% if object active
% if outside track
% if inside object
%
%   LOOP 50 times and save to each

for t = 0:1/f_anc:T_sim
    if(mod(floor(1000*t), floor(1000*MPPI.dt)) == 0)
        %% Update MPPI at a slower rate to get the current input
        
        [u_mppi, x_mppi] = MPPI.RunMPPI(car.x); % Shirley--> whatever this function
                                                % is called
        MPPI.plotController(); % function to plot all trajectories
    end

    %% Get input for this step from ancillary controller
    if(has_ancillary)
        state_error = x_mppi(:, 1) - car.x; % x0 for ancillary
        u_anc = K*state_error; 
    else
        u_anc = 0;
    end

    %% Update state of the actual system given the inputs from MPPI and iLQG
    car.setControl(u_anc + u_mppi(:, 1) + car.sampleControlNoise(1));
    x_hist(:, t_step + 1) = car.updateState();

    % Save our trajectories
    x_mppi_hist(:, t_step + 1) = x_mppi(:, 1);
    x_mppi_traj_hist{:, t_step + 1} = MPPI.x_traj;
    u_mppi_hist(:, t_step) = u_mppi(:, 1);
    u_anc_hist(:, t_step) = u_anc(:, 1);
    u_tot_hist(:, t_step) = car.u;

    %% Plot car/track
    track.spawnObstacles(car.x(1:2)); % Check for obstacle spawning
    car.plotSystem(); % plot position and orientation of our car
    track.plotTrack();
    drawnow();

    if(captureVideo)
        frame = getframe(gcf());
        writeVideo(v, frame);
    end

    %% Stop if way outside
    if(car.x(1) > 1 || car.x(1) < -5 || abs(car.x(2)) > 3)
        break;
    end

    xlim([-5, 1]);
    ylim([-3, 5]);
    t_step = t_step + 1;
end

%% Save everything 
save(fname);

if(captureVideo)
    close(v);
end