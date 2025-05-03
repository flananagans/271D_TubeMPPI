%% Script to test discrete linear system point
clear
close all
clc

figure()
scatter(0, 0, 5, 'k', 'filled');
hold on

k = 100; % number of paths to investigate
t = 25; % number of time steps

pts = cell(1, k);
xs = cell(size(pts));
for j = 1:k
    pts{j} = DiscreteLinearSystem();
    pts{j}.x(3:4) = [0.25, 0.25]; % initial velocity
    xs{j} = zeros(length(pts{j}.x), t);
end
for i = 1:t

    for j = 1:k
        pts{j}.addControlNoise();
        pts{j}.updateState();
        xs{j}(:, i) = pts{j}.x;
    end
end

for j = 1:k
    plot(xs{j}(1, :), xs{j}(2,:));
end