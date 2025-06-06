%% Example function to test the OvalTrack class
clear
close all
clc

%% Set up track and system (ring track from paper example)
or = 2.25;
ir = 1.75;
track = OvalTrack(or-ir, ir, 1);
track.clearance = 0.07;

system_state = [0;0;0;0];

% Iterate over time steps
figure();
scatter(system_state(1), system_state(2), 15, 'b', 'filled');
for t = 0:50

    % update system state
    system_state(1) = system_state(1) - 0.01;
    system_state(2) = system_state(2) + 0.05;
    
    % update/check track
    track.spawnObstacles(system_state);
    if(track.hitTrackLimits(system_state(1:2)))
        system_color = 'r';
    elseif(track.checkTrackLimits(system_state(1:2)))
        system_color = 'c';
    elseif(track.hitObstacles(system_state(1:2)))
        system_color = 'm';
    elseif(track.checkObstacles(system_state(1:2)))
        system_color = 'g';
    else
        system_color = 'b';
    end
    
    % plot track and system
    track.plotTrack();
    axis equal;
    scatter(system_state(1), system_state(2), 15, system_color, 'filled');
    drawnow
    pause(0.05)
end