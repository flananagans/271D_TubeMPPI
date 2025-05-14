%% Example function to test the OvalTrack class as a ring
clear
close all
clc

%% Set up track and system (ring track from paper example)
or = 2.125;
ir = 1.875;
track = OvalTrack(or-ir, ir, 0);

system_state = [0;0;0;0];

% Iterate over time steps
figure();
scatter(system_state(1), system_state(2), 15, 'b', 'filled');
for t = 0:50

    % update system state
    system_state(1) = system_state(1) - 0.15;
    system_state(2) = system_state(2);
    
    % update/check track
    if(track.checkTrackLimits(system_state))
        system_color = 'r';
    else
        system_color = 'b';
    end
    
    % plot track and system
    track.plotTrack();
    scatter(system_state(1), system_state(2), 15, system_color, 'filled');
    drawnow
    pause(0.05)
end