%% Example function to test the OvalTrack class
clear
close all
clc

%% Set up track and system (ring track from paper example)
track = OvalTrack();

system_state = [0;0;0;0];

% Iterate over time steps
figure();
scatter(system_state(1), system_state(2), 15, 'b', 'filled');
for t = 0:50

    % update system state
    system_state(1) = system_state(1);
    system_state(2) = system_state(2) + 0.35;
    
    % update/check track
    track.spawnObstacles(system_state);
    if(track.checkTrackLimits(system_state))
        system_color = 'r';
    elseif(track.checkObstacles(system_state))
        system_color = 'm';
    else
        system_color = 'b';
    end
    
    % plot track and system
    track.plotTrack();
    scatter(system_state(1), system_state(2), 15, system_color, 'filled');
    drawnow
    pause(0.05)
end