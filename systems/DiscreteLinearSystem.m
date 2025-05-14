% Class for describing a discrete linear system with dynamics of the form
%   x_{k+1} = Ax_{k} + Bu_{k}
%
%
%   This currently only has the methods to update the state and add noise 
%   to the control input from the control noise distribution 

% TODO: 

classdef DiscreteLinearSystem < handle

    properties
        %% System description
        % Right now this is the double integrator system from the paper
        % with delta_t = 0.01
        dt = 0.01;
        A = [eye(2), eye(2)*0.01;
            zeros(2,2), eye(2)];
        x = zeros(4, 1);
        B = [zeros(2, 2);
             eye(2)*0.01];
        u = zeros(2, 1);

        %% System limits
        % min (col 1) and max (col 2) allowable control for each channel
        u_lims = [-1, 1;
                  -1, 1] * 10;

        %% Noise definitions
        sigma_control = eye(2); % control noise
        sigma_environment = eye(4); % environmental noise

        %% Cost matrices
        % NIKI: Add Q and R definitions
    end

    methods
        % Constructor
        function obj=DiscreteLinearSystem()
            % nothing for now
        end

        % Function to set dt. Set this to match the controller frequency
        % of MPPI or iLQR
        function setDt(obj, dt)
            obj.dt = dt;

            % Update state-space matrices
            obj.A = [eye(2), eye(2)*dt;
                     zeros(2,2), eye(2)];
            obj.B = [zeros(2, 2);
                     eye(2)*dt];
        end

        
        % Function to update the state of the system based on current x
        % and u
        function x_new = updateState(obj)
            obj.x = obj.A*obj.x + obj.B*obj.u;

            x_new = obj.x;
        end

        % Function to update state in the presence of environmental noise
        function x_new = updateStateNoisy(obj)

            obj.updateState();
            
            % sample environmental noise
            W = transpose(mvnrnd(zeros(size(obj.x)), ...
                             obj.sigma_environment, ...
                             1));
            obj.x = obj.x + W;
            x_new = obj.x;
        end

        % Set the current control input
        function setControl(obj, u_new)
            obj.u = obj.clipControl(u_new);
        end

        % Function to clip control inputs to control limits
        function u_clipped = clipControl(obj, u)

            % apply control limits to the signal
            u_clipped = u;
            for m = 1:length(obj.u)
                % check for each row individually based on that row's lims
                u_clipped(m, u(m, :) < obj.u_lims(m, 1)) = obj.u_lims(m, 1);
                u_clipped(m, u(m, :) > obj.u_lims(m, 2)) = obj.u_lims(m, 2);
            end
        end

        % Function to sample control noise num_samples amount of times
        function E = sampleControlNoise(obj, num_samples)
            % add noise from the control noise distribution
            E = transpose(mvnrnd(zeros(size(obj.u)), ...
                                 obj.sigma_control, ...
                                 num_samples));
        end

        function [x_traj, u_traj, e_traj] = rolloutTraj(obj, u_init, T)
        %% Rollout a single trajectory given initial trajectory of inputs
        %   and the time horizon T
        %
        %   u_init: 2xN matrix where columns are each time step's input.
        %           N needs to equal t/obj.dt so each time step of the
        %           rollout has an input
        %   T: how many seconds into the future to simulate
        %
        %   x_traj: state trajectory over the horizon (t+1 -> N)
        %   u_traj: input trajectory over the horizon (t -> N-1)

            % how many time steps
            N = size(u_init, 2);
            if(N ~= T/obj.dt)
                error("incorrect size of u_init! You need one control input column for each of the N = t/obj.dt time steps\n");
            end

            % MOVED TO MPPI
            %lam = 1
            %traj_cost = 0

            x_traj = zeros(length(obj.x), N); % states along the trajectory
            e_traj = obj.sampleControlNoise(N);
            u_traj = u_init + e_traj; % input trajectory is the initial control trajectory + sampling of control noise
            u_traj = obj.clipControl(u_traj);

            for i = 1:N
                obj.setControl(u_traj(:, i)); % set system control
                obj.updateState(); % update state to roll forward
                x_traj(:, i) = obj.x; % record the current state

                % MOVED TO MPPI traj_cost = traj_cost +running_cost(lam,e_traj(:,i))
            end
            % MOVED TO MPPI traj_cost = traj_cost+terminal_cost()
        end

        %% Computing Costs and Contraints

        % NIKI: add calculateCost method

       
       
    end
end