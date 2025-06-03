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
dist_at_appear = zeros(1, length(fnames));
hit_obs_all = zeros(1, length(fnames));
for f_ind = 1:length(fnames)
    
    load([fnames(f_ind).folder, filesep, fnames(f_ind).name]);

    % Skip trials if they went the wrong way
    if(any(x_hist(2,:) < -0.5))

        x_at_appear(:, f_ind) = nan(size(car.x));
        dist_at_appear(f_ind) = NaN;
        hit_obs_all(f_ind) = NaN;
        continue;
    else % Collect data

        % find when obstacle appears
        t_ind = find(obs_isactive, 1);

        x_at_appear(:, f_ind) = x_hist(:, t_ind);
        dist_at_appear(f_ind) = track.getObstacleDistance(x_hist(1:2, t_ind));
        hit_obs_all(f_ind) = any(obs_hit);
    end
end

% Remove NaNs
x_at_appear(:, isnan(x_at_appear(1,:))) = [];
dist_at_appear(isnan(dist_at_appear)) = [];
hit_obs_all(isnan(hit_obs_all)) = [];


%% Train classifier
train_set = rand(size(hit_obs_all)) <= 0.7; % split into 70% train and 30% test
test_set = ~train_set;
x_train = [x_at_appear(:, train_set); dist_at_appear(train_set)];
y_train = hit_obs_all(train_set);
mdl = fitcdiscr(transpose(x_train), transpose(y_train));

x_test = [x_at_appear(:, test_set); dist_at_appear(test_set)];
y_test = hit_obs_all(test_set);
y_pred = transpose(predict(mdl, transpose(x_test)));

acc = sum(y_pred == y_test) / length(y_test);
fprintf('accuracy = %0.2f\n', acc);

% Plot 
figure()
scatter(dist_at_appear, x_at_appear(4, :), 15, hit_obs_all, 'filled');