%MPPI Controller Code 

classdef MPPI_Controller < handle
    %%
    properties
        lambda = 1
        N_traj = 1; % number of trajectories 
        T = 2; % time horizon
        dt = 0.01; % sampling period

        % System properties
        system = []; % system to act on
        x_traj = []; % nominal state trajectory
        u_traj = []; % control trajectory

        % Track used for constraints
        track = [];

        % Cost function properties
        v_des = 2.0; % desired velocity
    end

    methods
        % Constructor
        function obj = MPPI_Controller(system, track, N_traj, time_horizon)
            % Set our properties
            obj.N_traj = N_traj; 
            obj.T = time_horizon;
            obj.system = system;
            obj.track = track;
            obj.dt = system.dt;
            
            % Initialize state and control trajectory to zeros
            obj.x_traj = zeros(length(system.x), obj.T/obj.dt);
            obj.u_traj = zeros(length(system.u), obj.T/obj.dt);
        end
    
        % Function to run the core MPPI optimization
        function [U,X,S] = MPPI(obj, initial_state)

            % SHIRLEY: shift control trajectory over one step (u_traj)


            % array of costs for each trajectory
            cost_list = zeros(1, obj.N_traj);

            % cell array of each trajectory's noise trajectory
            e_traj_list = cell(1, obj.N_traj);

            % simulate N_traj -> acquire costs
            for k = 1:obj.N_traj

                % reset system to the initial state for start of rollout
                obj.system.x = initial_state; 
                
                % Rollout a trajectory
                [x_traj_k, u_traj_k, e_traj_list{k}] = obj.system.rolloutTraj(obj.u_traj, obj.T);
                
                % Compute the cost for this trajectory
                % SHIRLEY: iterate over x_traj_k, u_traj_k for each time step
                %
                % constraints will come from the track (Will added the
                % track as a property of MPPI)
                %    obj.track.checkTrackLimits(x_traj_k(i)(1:2)) % pass in this state's position
                

            end
            min_cost = min(cost_list);

            eta = 0; 
            w = zeros(1, obj.N_traj);
            for k = 1:obj.N_traj
                curr_w = exp((-1/obj.lambda)*(cost_list(k) - min_cost));
                w(k) = curr_w;
                eta = eta + curr_w;
            end

            % Find Optimal Perturbation Sequence
            sum = 0;
            for k = 1:obj.N_traj
                sum = sum + w(k)*e_traj_list{k};
            end

            % Update Control Sequence
            obj.u_traj = obj.u_traj + (1/eta)*sum;
            U = obj.u_traj;
    
            % Simulate the system with U to get X
            N = obj.T/obj.dt;
            X = zeros(size(A,2),N);
            X(:, 1) = initial_state;
            obj.system.x = initial_state;
            S = 0;
            for t = 1:N - 1
                obj.system.setControl(U(:, t));
                X(:, t+1) = obj.system.updateState();
                
                % SHIRLEY: update for MPPI cost
                S = obj.system.Cost(X) + S;
            end
            % SHIRLEY: update for MPPI cost
            S = obj.system.terminal_cost() + S;
    
        end
    
        function [U,X] = RunMPPI(obj, actual_state, safety)

            % Run MPPI starting from nominal state at this time
            [U_nominal, X_nominal, S_nominal] = obj.MPPI(obj.x_traj(:, 1));

            % Run MPPI starting from actual state at this time
            [U_actual, X_actual, S_actual] = obj.MPPI(actual_state);

            if S_actual <= S_nominal + safety
                U = U_actual;
                X = X_actual;
            else
                U = U_nominal;
                X = X_nominal;
            end
        end

        % SHIRLEY: Bring over cost methods from DLS
    end
end


