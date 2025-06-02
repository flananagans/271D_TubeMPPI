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

        x_traj_list = {}; % all trajectories generated for this time point

        % Track used for constraints
        track = [];

        % Cost function properties
        v_des = 2.0; % desired velocity

        % Plotting
        x_traj_plothandle = [];
        x_traj_list_plothandle = [];
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
        function [U,X,S, x_traj_list] = MPPI(obj, initial_state, C0)


            % Shift control trajectory over one step (u_traj)
            obj.u_traj = [obj.u_traj(:, 2:end), zeros(length(obj.system.u),1)];


            % array of costs for each trajectory
            cost_list = zeros(1, obj.N_traj);

            % cell array of each trajectory's noise trajectory
            e_traj_list = cell(1, obj.N_traj);

            % cell array of to track all the generated state trajectories
            x_traj_list = cell(1, obj.N_traj);

            % simulate N_traj -> acquire costs
            for k = 1:obj.N_traj

                % reset system to the initial state for start of rollout
                obj.system.x = initial_state; 
                
                % Rollout a trajectory
                [x_traj_k, u_traj_k, e_traj_k] = obj.system.rolloutTraj(obj.u_traj, obj.T);
                %initlize the trajectory cost
                traj_cost = 0;
                
                % Compute the cost for this trajectory
                % SHIRLEY: iterate over x_traj_k, u_traj_k for each time step
                %
                % constraints will come from the track (Will added the
                % track as a property of MPPI)
                % obj.track.checkTrackLimits(x_traj_k(i)(1:2)) % pass in this state's position
                % the function
                % Note From Shirley: Please check if I'm using the
                % constraint function correctly
                N = obj.T/obj.dt;
                for t = 1:N-1
                    %compute the running cost 
                    cc = obj.track.checkTrackLimits(x_traj_k(1:2,t)); % do we go out of the track
                    co = obj.track.checkObstacles(x_traj_k(1:2,t)); % do we hit an obstacle
                    traj_cost = traj_cost + obj.runningCost(x_traj_k(:,t+1), u_traj_k(:,t), e_traj_k(:,t), cc, co);
                end
                traj_cost = traj_cost + obj.terminalCost(x_traj_k(:, end)); 
                
                % store our trajectory info
                cost_list(k) = traj_cost;
                e_traj_list{k} = e_traj_k;
                x_traj_list{k} = x_traj_k;
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
            E = (1/eta)*sum; % optimal control perturbation

            % Update Control Sequence
            U = obj.u_traj + E;
    
            % Simulate the system with U to get X
            N = obj.T/obj.dt;
            X = zeros(size(obj.system.x,1),N); % this is states for t = 2:N
            obj.system.x = initial_state;
            S = 0;
            for t = 1:N
                obj.system.setControl(U(:, t));
                X(:, t) = obj.system.updateState();
                X_curr = X(:, t);
                CC = obj.track.checkTrackLimits(X_curr(1:2));
                CO = obj.track.checkObstacles(X_curr(1:2));
                
                % SHIRLEY: update for MPPI cost
                S = obj.runningCost(X(:, t), U(:,t), E(:, t), CC, CO) + S;
            end
            % SHIRLEY: update for MPPI cost
            S = obj.terminalCost(X(:, end)) + S;
    
        end
    
        function [U,X] = RunMPPI(obj, actual_state)

            % Run MPPI starting from nominal state at this time
            [U_nominal, X_nominal, S_nominal, x_traj_list_nominal] = obj.MPPI(obj.x_traj(:, 1));

            % Run MPPI starting from actual state at this time
            [U_actual, X_actual, S_actual, x_traj_list_actual] = obj.MPPI(actual_state);

            % Set safety threshold
            safety = 0;

            if S_actual <= S_nominal + safety
                U = U_actual;
                X = X_actual;
                obj.x_traj_list = x_traj_list_actual;
            else
                U = U_nominal;
                X = X_nominal;
                obj.x_traj_list = x_traj_list_nominal;
            end

            % update the object's trajectories
            obj.u_traj = U;
            obj.x_traj = X;
        end

        % SHIRLEY: Bring over cost methods from DLS
        %{
        function rc = runningCost(obj, x_t, u_t, e_t, cc)
            %exract velocities
            vx = x_t(3);
            vy = x_t(4);
            
            rc_state = (sqrt(vx^2+vy^2)-vdes)^2 + 1000*cc;
            rc_control = obj.lambda*(u_t'/obj.system.sigma_control)*e_t;
            rc = rc_state + rc_control;            
        end
        %}

        % Referring to algorithm in the paper and equations 13 & 14
        function cost = runningCost(obj, x_t, u_t, e_t,  CC, CO)

            % Get velocity of system
            vx = x_t(3); 
            vy = x_t(4);

            %calculate the cost of the states and control
            cost_states = (sqrt(vx^2 + vy^2) - obj.v_des)^2 + 1000*CC + 5*CO;
            cost_control = obj.lambda*(u_t'/obj.system.sigma_control)*e_t;
            cost = cost_states + cost_control;
        end

        % For Linear Discrete Time System, this is 0
        function tc = terminalCost(obj, final_state)
            tc = 0;
        end
       
        %% Plotting trajectories
        function plotController(obj)

            if(isempty(obj.x_traj_list_plothandle))
                obj.x_traj_list_plothandle = cell(size(obj.x_traj_list));
            end

            % Plot all of our tested trajectories
            for k = 1:length(obj.x_traj_list_plothandle)
                if(~isempty(obj.x_traj_list_plothandle{k}))
                    % update the trajectory's data
                    obj.x_traj_list_plothandle{k}.XData = obj.x_traj_list{k}(1,:);
                    obj.x_traj_list_plothandle{k}.YData = obj.x_traj_list{k}(2,:);
                else
                    % plot the trajectory for the first time
                    obj.x_traj_list_plothandle{k} = plot(obj.x_traj_list{k}(1,:), obj.x_traj_list{k}(2,:), 'LineWidth', 0.5, 'Color', '#AAAAAA');
                end
            end

            % Plot our nominal trajectory
            if(~isempty(obj.x_traj_plothandle))
                % update the trajectory's data
                obj.x_traj_plothandle.XData = obj.x_traj(1, :);
                obj.x_traj_plothandle.YData = obj.x_traj(2, :);
            else
                % plot the trajectory for the first time
                obj.x_traj_plothandle = plot(obj.x_traj(1, :), obj.x_traj(1, :), 'k', 'LineWidth', 1);
            end
        end
        
    end
end


