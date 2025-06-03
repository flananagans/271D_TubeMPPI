%% Classify collisions and avoidances

clear
close all
clc

initWorkspace();

%% Load data
fnames = dir("CalibrationData\Run5\*.mat");

%% Collect data
load([fnames(1).folder, filesep, fnames(1).name]);

x_at_appear = zeros(length(car.x), length(fnames));
dist_at_appear = zeros(size(fnames));
hit_obs_all = zeros(size(fnames));
for f_ind = 1:length(fnames)
    
    load([fnames(f_ind).folder, filesep, fnames(f_ind).name]);

    % Skip trials if they went the wrong way
    if(any(x_hist(2,:) < -0.5))

        xs_appear_all(f_ind) = NaN;
        dist_at_appear(f_ind) = NaN;
        hit_obs_all(f_ind) = NaN;
        continue;
    else % Collect data
        xs_all(:, f_ind) = 
        
    end
    
    if(f_ind == 1)
        %% Plot the track
        figure();
        try(track.resetPlotter());
        catch
        end
        track.plotTrack()
        axis equal
    end

    % plot the trajectory
    if(any(obs_hit))
        traj_col = 'm';
    elseif(any(outside_track))
        traj_col = '#AA2222';
    else
        traj_col = '#AAAAAA';
    end

    % Skip any bad trajectories
    if(~strcmp(traj_col, '#AAAAAA'))
        continue;
    end

    x_hist(x_hist == 0) = NaN;
    plot(x_hist(1,:), x_hist(2,:), 'Color', traj_col);
end

%% Plot bad trajectories
for f_ind = 1:length(fnames)
    
    load([fnames(f_ind).folder, filesep, fnames(f_ind).name]);

    % Skip trials if they went the wrong way
    if(any(x_hist(2,:) < -0.5))
        continue;
    end

    % plot the trajectory
    if(any(obs_hit))
        traj_col = 'm';
    elseif(any(outside_track))
        traj_col = '#AA2222';
    else
        traj_col = '#AAAAAA';
    end
    
    % Skip any good trajectories
    if(strcmp(traj_col, '#AAAAAA'))
        continue;
    end

    x_hist(x_hist == 0) = NaN;
    plot(x_hist(1,:), x_hist(2,:), 'Color', traj_col);
end

%% Plot scatter points for when obstacle is active and clean up plot
for f_ind = 1:length(fnames)
    
    load([fnames(f_ind).folder, filesep, fnames(f_ind).name]);

    % Skip trials if they went the wrong way
    if(any(x_hist(2,:) < -0.5))
        continue;
    end

    % plot unfilled circle for where the obstacle was first activated
    obs_act = find(obs_isactive);
    scatter(x_hist(1, obs_act(1)), x_hist(2, obs_act(1)), 15, 'MarkerEdgeColor', 'k');
    
    if(f_ind == length(fnames))
        % plot filled circle for starting position
        scatter(0,0, 15, 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');

        % set axes
        xlabel('x position (m)')
        ylabel('y position (m)')
        xlim([-5, 1]);
        ylim([-3, 5]);
    end
end