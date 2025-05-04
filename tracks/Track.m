%% Abstract class for all of our tracks
% Individual tracks will be derived from this

classdef (Abstract) Track < handle
    %TRACK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Abstract)
        boundaries;
        obstacle;
    end

    methods (Abstract)
        % Methods that will be defined for each track type
        checkTrackLimits(obj, system_state); % true if system_state outside limits
        checkObstacles(obj, system_state); % true is system_state inside obstacle
        spawnObstacles(obj, system_state); % updates the track's obstacles
        plotTrack(obj); % plots the current track state
    end
end

