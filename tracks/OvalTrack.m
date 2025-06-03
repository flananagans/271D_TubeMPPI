classdef OvalTrack < Track
    %OVALTRACK
    %   track with a specified width, turn radius (of inside line), 
    %   and straight length
    %
    %   the track will have the origin at the end of the bottom curve
    %   (beginning of the straight), then the radius is from the center of
    %   the turn to the inside edge of the track
    
    properties
        boundaries = struct('width', 5, 'radius', 3, 'straightlength', 10); % limits of track
        
        % circular obstacle
        obstacle = struct('active', false, 'center', [-0.625; 2], 'radius', 0.25);
        obstacle_spawn_ylim = 1;
        obstacle_spawn_mean = 1; % spawn obstacle once state passes this line
        obstacle_spawn_var = 0.05;
        obstacle_clearance = 0.005;
    end

    properties (Access=private)
        obstacle_plothandle = [];
        track_plotted = false;
    end
    
    methods
        function obj = OvalTrack(w, r, sl)
            %OVALTRACK Construct an instance of this class

            % default track width
            if(nargin < 1 )
                w = 5;
            end

            %default track radius
            if(nargin < 2 )
                r = 3;
            end

            % default track straight length
            if(nargin < 3)
                sl = 10;
            end

            % Set boundaries
            obj.boundaries = struct('width', w, 'radius', r, 'straightlength', sl);

            % Set obstacle spawn ylim
            obj.obstacle_spawn_ylim = mvnrnd(obj.obstacle_spawn_mean, obj.obstacle_spawn_var, 1);
        end

        % set straight length of track
        function setStraightLength(obj, sl)
            obj.boundaries.straightlength = sl;
        end
        
        function outside_bounds = checkTrackLimits(obj, system_pos)
        % Function to check if we have are outside track limits
        %   returns true if system_pos outside the track
        %
        %   system_pos: position  of system in [x;y]

            if((system_pos(2) > 0) && ...
               (system_pos(2) < obj.boundaries.straightlength))
            
                % check if in (0,0) side
                inside_bounds = (system_pos(1) <= obj.boundaries.width/2) && ...
                                (system_pos(1) >= -obj.boundaries.width/2);

                % check if in other side
                inside_bounds = inside_bounds || ...
                                (system_pos(1) <= -2*obj.boundaries.radius - obj.boundaries.width/2) && ...
                                (system_pos(1) >= -2*obj.boundaries.radius - 3*obj.boundaries.width/2);

            else % we are in the curves

                % by default say it is in the bottom curve
                center_pt = [-obj.boundaries.radius - obj.boundaries.width/2; 
                             0];                

                if(system_pos(2) >= obj.boundaries.straightlength)
                    % system is in the top curve
                    center_pt(2) = obj.boundaries.straightlength;
                end

                r = vecnorm(system_pos(1:2) - center_pt);

                inside_bounds = (r >= obj.boundaries.radius) && ...
                                (r <= obj.boundaries.radius + obj.boundaries.width);
            end

            outside_bounds = ~inside_bounds;
        end

        function in_obstacle = checkObstacles(obj, system_pos)
        % Function to check if we have hit an obstacle
        %   returns true if system_pos inside obstacle
        %
        %   system_pos: position of system in [x;y]

            if(obj.obstacle.active)
                % check if state is inside boundary
                
                in_obstacle = obj.getObstacleDistance(system_pos) < obj.obstacle_clearance;
            else
                % no obstacles
                in_obstacle = false;
            end
        end

        function d = getObstacleDistance(obj, system_pos)
        % Function to get the distance of a system to the edge of the
        %   obstacle
        %
        %   system_pos: position of system in [x;y]

            if(obj.obstacle.active)
                d = vecnorm(obj.obstacle.center - system_pos) ...
                            - obj.obstacle.radius;
            else
                d = NaN;
            end
        end

        function spawnObstacles(obj, system_state)
            % updates the track's obstacles

            if(system_state(2) > obj.obstacle_spawn_ylim)
                obj.obstacle.active = true;
            end
        end

        function resetPlotter(obj)
        % call this to reset plotting flags so you can replot a track later
            obj.track_plotted = false;
            obj.obstacle_plothandle = [];
        end

        function plotTrack(obj)
            % plots the current track state
        
            %% Track
            % Plot boundaries
            hold on

            % Only plot track once
            if(~obj.track_plotted)
                obj.track_plotted = true;

                if(obj.boundaries.straightlength > 0)
                % We may not have a straight if the track is a ring
                    % plot first straight
                    plotStraight([0;0], obj.boundaries.width, obj.boundaries.straightlength, [0;1]);
        
                    % plot second straight
                    plotStraight([-1*(obj.boundaries.width + 2*obj.boundaries.radius);0], obj.boundaries.width, obj.boundaries.straightlength, [0;1]);
                end
    
                % plot top curve
                plotCurve([0; obj.boundaries.straightlength], obj.boundaries.width, obj.boundaries.radius, [-1;0]);
    
                % plot bottom curve
                plotCurve([-1*(obj.boundaries.width + 2*obj.boundaries.radius); 0], obj.boundaries.width, obj.boundaries.radius, [1;0]);
            end

            %% Obstacle
            if(obj.obstacle.active && isempty(obj.obstacle_plothandle))
                % plot obstacle
    
                thetas = linspace(0, 2*pi, 100);

                patch_x = obj.obstacle.center(1) + obj.obstacle.radius*cos(thetas);
                patch_y = obj.obstacle.center(2) + obj.obstacle.radius*sin(thetas);

                obj.obstacle_plothandle = patch(patch_x, patch_y, 'k');
            elseif(~isempty(obj.obstacle_plothandle) && ~obj.obstacle.active)
                obj.obstacle_plothandle.Visible = false;
            end

        end
    end
end

function plotStraight(pos_start, width, length, straight_dir)
%% Plot a straight section of the track
%
%   pos_start: coordinates of start position (this is in the midline of the
%              straight)
%   width: width of the straight
%   length: length of the straight
%   straight_dir: column vector that defines the straight direction
    
    straight_dir = straight_dir/vecnorm(straight_dir); % get unit vector

    rot_90 = [0, -1;
              1,  0]; % rotate +90 deg about Z
    perp_dir = rot_90*straight_dir;

    % "left" side line
    leftside_bot = pos_start + perp_dir*width/2;
    leftside_top = leftside_bot + straight_dir*length;
    leftside = [leftside_bot, leftside_top]; % [xs; ys]

    % "right side line
    rightside_bot = pos_start - perp_dir*width/2;
    rightside_top = rightside_bot + straight_dir*length;
    rightside = [rightside_bot, rightside_top]; % [xs; ys]

    % plot it up
    plot(leftside(1,:), leftside(2,:), 'k', 'LineWidth', 1.5);
    plot(rightside(1,:), rightside(2,:), 'k', 'LineWidth', 1.5);
end

function plotCurve(pos_start, width, radius, center_dir)
%% Plot a 180deg curved section of the track
%
%   pos_start: coordinates of start position (this is in the midline of the
%              curve)
%   width: width of the straight
%   radius: radius of the curve OF THE INSIDE LINE
%   center_dir: column vector defines the direction to the center from
%               pos_start
%               the curve will start perpendicular to this and this vector
%               will go counterclockwise by 180deg

    thetas = linspace(0, pi, 1000);

    % Find the center of the curve
    center_dir = center_dir/vecnorm(center_dir); % get the unit vector
    center_pt = pos_start + center_dir*(radius + width/2);

    inside_line = zeros(2, length(thetas));
    outside_line = zeros(2, length(thetas));
    for th_ind = 1:length(thetas)
        th = thetas(th_ind);
        rot_mat = [cos(th), -sin(th);
                   sin(th),  cos(th)]; % rotation about z

        % Make the inside line
        inside_line(:, th_ind) = center_pt - radius*rot_mat*center_dir;

        % Make the outside line
        outside_line(:, th_ind) = center_pt - (radius + width)*rot_mat*center_dir;
    end
    
    % plot it up
    plot(inside_line(1,:), inside_line(2,:), 'k', 'LineWidth', 1.5);
    plot(outside_line(1,:), outside_line(2,:), 'k', 'LineWidth', 1.5);
end