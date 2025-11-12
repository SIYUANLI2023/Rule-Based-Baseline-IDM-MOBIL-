%% ============================================================
%  Baseline Simulation Script for Case 2 (IDM + MOBIL)
%  ------------------------------------------------------------
%  Description:
%    This script runs the complete baseline simulation for
%    the multi-lane driving scenario (Case 2), where:
%      - The Ego Vehicle (EV) uses IDM for longitudinal control
%        and MOBIL for lane-change decisions.
%      - The first surrounding vehicle (SV1) reacts to the EV’s
%        lane change using an event-triggered acceleration and
%        a hysteretic PD-based following policy.
%      - Other vehicles maintain constant-speed cruising.
%
%  Outputs (all stored in ./results/):
%    - baseline_case2.mat   : full simulation data structure
%    - playback_case2.gif   : lane-change playback animation
%    - velocity_case2.png   : EV and SV speed profiles
%    - trajectory_case2.png : compressed x–y trajectory plot
%
%  Author: Siyuan Li from Loughborough University.
%  Date  : November 2025
% ============================================================

clear;
close all;
clc;
set(groot,'defaultLegendAutoUpdate','off'); % Prevent legends from auto-updating


%% ========== 1) Run Baseline Simulation ==========
disp('>>> Step 1: running baseline simulation (IDM + MOBIL)...');

% Run the rule-based simulation (IDM + MOBIL)
% The function returns a structured dataset "baseline"
baseline = run_case2_baseline_IDM_MOBIL();

% Create ./results folder if not existing
if ~exist('results','dir')
    mkdir results;
end

% Save the simulation results
save(fullfile('results','baseline_case2.mat'),'baseline');
disp('>>>   baseline saved to results/baseline_case2.mat');

%% ========== 2) Playback Visualization ==========
% disp('>>> Step 2: creating playback GIF...');
% 
% % Define output GIF path (stored directly in ./results/)
% gifFile = fullfile('results','playback_case2.gif');
% 
% % Generate playback animation
% % Arguments:
% %   baseline : simulation result structure
% %   gifFile  : output GIF file path
% %   false    : whether to display animation in real-time (false = export only)
% playback_lanechange_from_baseline(baseline, gifFile, false);
% 
%% ========== 3) Plot Speed Profiles ==========
disp('>>> Step 3: plotting speed curves...');

% Define key timestamps (s) and reference speeds (m/s)
key.T = [0 5.85 7.8 12.6 18 29.25];  % time markers [s]
key.V = [30 28.25 30 30 30 30];     % corresponding EV speeds [m/s]

% Plot longitudinal speed profiles with key markers
% The function returns a figure handle for export
hSpd = plot_velocity_from_baseline(baseline, key);

% Save figure under results/
saveas(hSpd, fullfile('results','velocity_case2.png'));

%% ========== 4) Plot Trajectory ==========
disp('>>> Step 4: plotting whole trajectory...');

% Plot the full x–y trajectory 
hTraj = plot_traj_compressed_from_baseline(baseline);

% Save trajectory plot under results/
saveas(hTraj, fullfile('results','trajectory_case2.png'));

disp('>>> Baseline simulation and visualization completed successfully.');
