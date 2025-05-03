% Class for describing a discrete linear system with dynamics of the form
%   x_{k+1} = Ax_{k} + Bu_{k}
%
%
%   This currently only has the methods to update the state and add noise 
%   to the control input from the control noise distribution 


classdef DiscreteLinearSystem < handle

    properties
        %% System description
        % Right now this is the double integrator system from the paper
        % with delta_t = 0.01
        A = [eye(2), eye(2)*0.01;
            zeros(2,2), eye(2)];
        x = zeros(4, 1);
        B = [zeros(2, 2);
             eye(2)*0.01];
        u = zeros(2, 1);

        %% System limits
        u_lims = [-5, 5;
                  -5, 5]; % min and max allowable control

        %% Noise definitions
        sigma_control = eye(2)/10; % control noise
        sigma_environment = eye(4)/10; % environmental noise
    end

    methods
        % Constructor
        function obj=DiscreteLinearSystem(varargin)
            % nothing for now
        end

        % Function to update the state of the system based on current x
        % and u
        function updateState(obj)
            obj.x = obj.A*obj.x + obj.B*obj.u;
        end

        % Function to add control noise
        function addControlNoise(obj)

            % add noise from the control noise distribution
            obj.u = obj.u + transpose(mvnrnd(zeros(size(obj.u)), ...
                                             obj.sigma_control));

            % apply control limits to the signal
            for m = 1:length(obj.u)
                obj.u(m) = max([min([obj.u_lims(m, 2), obj.u(m)]), ...
                                                obj.u_lims(m, 1)]);
            end
        end
    end
end



