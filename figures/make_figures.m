clear
close all
clc

ax_fmt = struct('Box', 'off', 'XGrid', 'off', 'YGrid', 'off', 'TickLabelInterpreter', 'none', 'FontSize', 8, 'FontName', 'Arial');
fig_fmt = struct('Units', 'inches');

%% Track
open('ring_track.fig');

set(gcf(), fig_fmt, 'Position', [5.5, 1.5, 3.3, 3.3]);
set(gca(), ax_fmt);

%% Ancillary Controller Comparison

open('tube_comparison.fig');
set(gcf(), fig_fmt, 'Position', [5.5, 1.5, 3.3, 3.4]);
subplot(2,2,1)
set(gca(), ax_fmt);
subplot(2,2,2)
set(gca(), ax_fmt);
subplot(2,2,3)
set(gca(), ax_fmt);
subplot(2,2,4)
set(gca(), ax_fmt);

open('ancillary_comparison.fig');
set(gcf(), fig_fmt, 'Position', [5.5, 1.5, 3.3, 3.125]);
set(gca(), ax_fmt);
nexttile(1)
set(gca(), ax_fmt);
nexttile(2)
set(gca(), ax_fmt);


%% Obstacle Track
open('oval_track.fig');

ylim([-2.5, 3.5]);
set(gcf(), fig_fmt, 'Position', [5.5, 1.5, 3.3, 3.15]);
set(gca(), ax_fmt);

%% Monte Carlo runs
open("run08_noise10.fig")

xlim([-1, 1]);
ylim([-0.125, 3]);
set(gcf(), fig_fmt, 'Position', [5.5, 1.5, 3.3, 3.5]);
set(gca(), ax_fmt);

%% Tube Size
open("run14_noise01_histo.fig")
set(gcf(), fig_fmt, 'Position', [5.5, 1.5, 3.3, 1.3]);
set(gca(), ax_fmt);

open("run08_noise10_histo.fig")
set(gcf(), fig_fmt, 'Position', [5.5, 1.5, 3.3, 1.3]);
set(gca(), ax_fmt);

open("run13_noise20_histo.fig")
set(gcf(), fig_fmt, 'Position', [5.5, 1.5, 3.3, 1.3]);
set(gca(), ax_fmt);

%% Obstacle avoidance with clearance
open("run16_clearance.fig")

xlim([-1, 1]);
ylim([-0.125, 3]);
set(gcf(), fig_fmt, 'Position', [5.5, 1.5, 3.3, 3.5]);
set(gca(), ax_fmt);
