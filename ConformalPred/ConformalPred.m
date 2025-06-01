
clear all; 

%%
run_num = 1;

path  = fullfile(strcat('Run', int2str(run_num)));
files = dir(fullfile(path,'*.mat'));

len = length(files);
%%


for i=1:len
    data = load(files(i).name,'outside_track', 'obs_isactive', 'obs_hit');
    if any(obs_hit)
        i
    end

end

