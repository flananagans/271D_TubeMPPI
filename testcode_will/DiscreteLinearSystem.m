% Class for describing a discrete linear system with dynamics of the form
%   x_{k+1} = Ax_{k} + Bu_{k}

classdef DiscreteLinearSystem < handle

    properties
        A = [eye(2), eye(2)*0.01;
            zeros(2,2), eye(2)];
        x = zeros(4, 1);
        B = [zeros(2, 2);
             eye(2)*0.01];
        u = zeros(2, 1);

        u_lims = [-5, 5;
                  -5, 5]; % min and max allowable control

        sigma_control = eye(2)/10; % control noise
        sigma_environment = eye(4)/10; % environmental noise
    end

    methods
        % Constructor
        function obj=DiscreteLinearSystem(varargin)
            
        end

        % Function to update the state of the system based on current x
        % and u
        function updateState(obj)
            obj.x = obj.A*obj.x + obj.B*obj.u;
        end

        % Function to add control noise
        function addControlNoise(obj)
            obj.u = obj.u + transpose(mvnrnd(zeros(size(obj.u)), obj.sigma_control));

            for m = 1:length(obj.u)
                obj.u(m) = max([min([obj.u_lims(m, 2), obj.u(m)]), obj.u_lims(m, 1)]);
            end
        end

    end

end



