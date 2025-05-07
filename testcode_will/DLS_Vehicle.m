% Class for describing a discrete linear system with dynamics of the form
%   x_{k+1} = Ax_{k} + Bu_{k}
%
%
%   This currently only has the methods to update the state and add noise 
%   to the control input from the control noise distribution 


classdef VehicleDLS < handle

    properties
        %% System description
        A = [[-(cr+cf)/(m*vx) 0 -1-(cf*lf-cr*lr)/(m*vx^2)]
            [0 0 1 0]]


        %% System limits
        u_lims = [-5, 5;
                  -5, 5]; % min and max allowable control

        %% Noise definitions
        sigma_control = eye(2)/10; % control noise
        sigma_environment = eye(4)/10; % environmental noise
    end

    methods
        % Constructor
        function params=DiscreteLinearSystem(varargin,cr,cf,m,vx,cr,lr,Iz)
            % nothing for now
            params = []
        end

        % Function to update the state of the system based on current x
        % and u
        function updateState(obj)
            obj.x = obj.A*obj.x + obj.B*obj.u;
        end

        % Set the current control input
        function setControl(obj, u_new)
            obj.u = u_new;

            % apply control limits to the signal
            for m = 1:length(obj.u)
                obj.u(m) = max([min([obj.u_lims(m, 2), obj.u(m)]), ...
                                                obj.u_lims(m, 1)]);
            end
        end

        % Function to sample control noise num_samples amount of times
        function E = sampleControlNoise(obj, num_samples)
            % add noise from the control noise distribution
            E = transpose(mvnrnd(zeros(size(obj.u)), ...
                                 obj.sigma_control, ...
                                 num_samples));
        end
    end
end



