%MPPI Controller Code 

classdef MPPI_Controller_Class < handle
    %%
    properties
        lambda = 1
        N_traj = 0; % #number of trajecotories 
        T = 0;


    end

    methods
        % Constructor
        function obj=MPPI_Controller(varargin,system,N_traj, time_horizon)
                obj.N_traj = N_traj; 
                obj.T = time_horizon;
                obj.system = system;
                obj.dt = system.dt;
                obj.u_MPPI = zeros(len(obj.system.u),time_horizon/obj.dt);
        end
    
        function [U,X,S] = MPPI(obj, state)
            cost_list = zeros(1,N_traj)
            N = obj.T/obj.dt;
            %simulate N_traj -> aquire costs
            for k = 1:N_traj
                [~, ~,e_traj,traj_cost] = rolloutTraj(obj, obj.u_MPPI, obj.time_horizon)
                cost_list(k) = traj_cost
            end
            min_cost = min(cost_list);
            eta = 0 
            w = zeros(1,N_traj)
            for k = 1:N_traj
                curr_w = exp((-1/lambda)*(cost_list(k)-min_cost));
                w(k) = curr_w
                eta = eta +curr_w;
                
    
            end
            % Find Optimal Perturbation Sequence
            sum = 0
            for k = 1:N_traj
                sum = sum+ w(k)*E(k);
            end
            % Update Control Sequence
            obj.u_MPPI = obj.u_MPPI + (1/eta)*sum;
            U = obj.u_MPPI;
    
            %Simulate U to get X
            A = obj.system.A;
            B = obj.system.B;
            X = zeros(size(A,2),N);
            X(1) = state;
            S = 0;
            for t=1:N
                X(t+1) = A*X(t)+B.U(t);
                S = system.Cost(X) + S;
            end
            S = system.terminal_cost()+S;
    
        end
    
        function [U,X] = RunMPPI(obj, nominal_state, actual_state, safety)
            [U_nominal, X_nominal, S_nominal] = MPPI(nominal_state);
            [U_actual, X_actual, S_actual] = MPPU(actual_state);
            if S_actual <= S_nominal + safety
                U = U_actual
                X = X_actual
            else
                U = U_nominal
                X = X_nominal
            end
        end
    end
end


