classdef OvalTrack < Track
    %OVALTRACK
    %   track with a specified width, turn radius, and straight length
    %
    %   the track will have the origin at the end of the bottom curve
    %   (beginning of the straight), then the radius is from the center of
    %   the turn to the inside edge of the track
    
    properties
        boundaries = struct('width', 5, 'radius', 5, 'straightlength', 5); % limits of track
        obstacle = struct('active', false, 'xlim', [-0.5, 0.5], 'ylim', [2, 3]);
        obstacle_spawn_ylim = 1; % spawn obstacle once state passes this line
    end

    properties (Access=private)
        obstacle_plothandle = [];
    end
    
    methods
        function obj = OvalTrack()
            %OVALTRACK Construct an instance of this class
        end
        
        function outside_bounds = checkTrackLimits(obj, system_state)
            % true if system_state outside limits


            outside_bounds = (system_state(1) <= obj.boundaries.xlim(1)) || ...
                             (system_state(1) >= obj.boundaries.xlim(2));
        end

        function in_obstacle = checkObstacles(obj, system_state)
            % true is system_state inside obstacle

            if(obj.obstacle.active)
                % check if state is inside boundary
                in_obstacle = ((system_state(1) >= obj.obstacle.xlim(1)) && ...
                              (system_state(1) <= obj.obstacle.xlim(2))) && ...
                              ((system_state(2) >= obj.obstacle.ylim(1)) && ...
                              (system_state(2) <= obj.obstacle.ylim(2)));
            else
                % no obstacles
                in_obstacle = false;
            end
        end

        function spawnObstacles(obj, system_state)
            % updates the track's obstacles

            if(system_state(2) > obj.obstacle_spawn_ylim)
                obj.obstacle.active = true;
            end
        end

        function plotTrack(obj)
            % plots the current track state
        
            % Plot boundaries
            hold on
            plot(obj.boundaries.xlim(1), 'k', 'LineWidth', 1.5);
            plot(obj.boundaries.xlim(2), 'k', 'LineWidth', 1.5);

            % Plot spawn limit for obstacle
            plot(obj.boundaries.xlim, [obj.obstacle_spawn_ylim, obj.obstacle_spawn_ylim], 'k--');

            % set axis limits
            xlim([obj.boundaries.xlim(1) - 1, obj.boundaries.xlim(2) + 1]);
            ylim([-1, 10]);
        
            if(obj.obstacle.active && isempty(obj.obstacle_plothandle))
                % plot obstacle
    
                patch_x = [obj.obstacle.xlim(1), obj.obstacle.xlim(2), ...
                           obj.obstacle.xlim(2), obj.obstacle.xlim(1)];
                patch_y = [obj.obstacle.ylim(1), obj.obstacle.ylim(1), ...
                           obj.obstacle.ylim(2), obj.obstacle.ylim(2)];

                obj.obstacle_plothandle = patch(patch_x, patch_y, 'k');
            elseif(~isempty(obj.obstacle_plothandle) && ~obj.obstacle.active)
                obj.obstacle_plothandle.Visible = false;
            end
        end
    end
end

