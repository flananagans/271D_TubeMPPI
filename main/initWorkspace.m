%% Initialize the workspace
%   Mainly adding dependencies to path and adding plotting formats

% Base directory
base_dir = '..'; % the parent directory of the repo

% add tracks
addpath(genpath(fullfile(base_dir, 'tracks')));

% add systems
addpath(genpath(fullfile(base_dir, 'systems')));

% add MPPI
addpath(genpath(fullfile(base_dir, 'MPPI')));

% add iLQG
addpath(genpath(fullfile(base_dir, 'iLQG')));

% add PID
addpath(genpath(fullfile(base_dir, 'PID')));

% add LQR
addpath(genpath(fullfile(base_dir, 'LQR')));

% Plotting things
ax_fmt = struct('Box', 'off', 'TickLabelInterpreter', 'none', 'FontSize', 8, 'FontName', 'Arial');
fig_fmt = struct('Units', 'inches');
lw_line = 1.5; % linewidth for plotted traces
lw_bar = 1; % linewidth for error bars
%colors = {'#007BFF', '#9955B4', '#B80028', '#FF4747', '#5BD75B', '#FFD700', '#A0A0A0'};
%colors = {'#B23A3A', '#D97D50', '#4C8C4A', '#A1D3A1', '#4A90E2', '#7C4D92', '#9E9E9E'}; 
%colors = {'#D15B5B', '#FF8A4C', '#558B45', '#A1D85D', '#4B90C6', '#6A3F8D', '#A1A1A1'};
%colors = {'#000000', '#D15B5B', '#558B45', '#A1D85D', '#4B90C6', '#6A3F8D', '#A1A1A1'};
%colors = {'#212121', '#FF7043', '#388E3C', '#81C784', '#1976D2', '#7B1FA2', '#B0BEC5'};
%colors = {'#292929', '#FF453F', '#50722F', '#4CA951', '#368FE7', '#BC66E1', '#ADBBC2'}; % black, red, green, green, blue, purple, grey
%colors = {'#f98177', '#b979a1', '#3382d7', '#00274c', '#2a7f62', '#525252', '#b8b8b8'};
%colors = {'#992929', '#C06A35', '#74AAAA', '#8ED6C1', '#C2D1FF', '#6F6F6F', '#CCCCCC'};
colors = {'#005587', '#7f5da7', '#dc5b96', '#ff7860', '#ffb81c', '#6F6F6F', '#CCCCCC'};

