%% Determine Conformity score (min distance) of each run
clear all;
%% Indicate the Runs that will be included in the distrobution
run_num = [7,8];
min_dist_lst = [];
% Main loop
for N = run_num
   path  = fullfile(strcat('Run', int2str(N)));
   files = dir(fullfile(path,'*.mat'));
   len = length(files);
   %%
   for i=1:len
       data = load(files(i).name,'outside_track', 'obs_isactive', 'obs_hit', 'track', 'x_hist', 't_step');
       if(any(data.x_hist(2,:) < -0.5))
           continue;
       end
       dist_lst = zeros(1,data.t_step);
       for j = 1:data.t_step
           x_curr = data.x_hist(1, j);
           y_curr = data.x_hist(2, j);
           dist_curr = data.track.getObstacleDistance([x_curr;y_curr]);
           dist_lst(j) = dist_curr;
       end
       if isnan(min(dist_lst))
           display("Hit a NaN")
       end
       min_dist_lst = [min_dist_lst, min(dist_lst)];
   end
end
%%
min_dist_lst_inv = zeros(1, length(min_dist_lst));
for h = 1:length(min_dist_lst)
   if(min_dist_lst(h)<= 0)
       min_dist_lst_inv(h) = 300;
   else
       min_dist_lst_inv(h) = 1/min_dist_lst(h);
   end
end
%%
clear figure;
%edges = round(min(min_dist_lst_inv),2):0.005:round(max(min_dist_lst_inv),2);
histogram(min_dist_lst_inv)
grid on;
xlabel("Minimum Distance")
alpha = 0;
hold on
xline(alpha, "--",'linewidth',2)
hold off;
%%
%% Plotting Histogram
clear figure;
%edges = round(min(min_dist_lst),2):0.005:round(max(min_dist_lst),2);
histogram(min_dist_lst)
grid on;
xlabel("Minimum Distance")
alpha = 0.005;
hold on
xline(alpha, "--",'linewidth',2)
hold off;
percent_within_safety = sum(min_dist_lst>=alpha)/length(min_dist_lst);
num_obstacle_hits = sum(min_dist_lst<=0);

