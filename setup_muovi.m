%% setup_muovi.m — Configure and save experiment parameters
% Run this once before experiment_muovi.m
close all; clear; clc;

% =========================================================================
%% USER OPTIONS
% =========================================================================
dryrun   = 0;               % 1 = no hardware, fake data
datapath = uigetdir([], 'Select folder to save data');
side     = 'right';         % reserved for future bilateral support

% =========================================================================
%% HARDWARE PARAMETERS
% =========================================================================
TCPPort  = 54320;           % SyncStation TCP port (match QA script)
sampFreq = 2000;            % Hz
num_muovi = 2;              % 1 or 2 Muovi+ devices

% SyncStation command encoding
channels_per_muovi = 70;    % 64 EMG + 6 AUX
channels_sync      = 6;
total_channels     = num_muovi * channels_per_muovi + channels_sync;


% =========================================================================
%% CHANNEL LAYOUT
% =========================================================================
% Force channels — absolute indices in full data matrix (from QA script)
force_left  = 141;
force_right = 142;
force_sum   = 144;

% EMG channels — absolute indices for both Muovi+ devices
emg_channels = [1:64, 71:134];   % 128 channels total

% =========================================================================
%% ACQUISITION PARAMETERS
% =========================================================================
block_samples = 200;        % samples per read loop iteration
ds            = 20;         % EMG display downsample factor
offsettime    = 0.5;        % seconds of baseline to collect for offset

% =========================================================================
%% MVC PARAMETERS
% =========================================================================
mvc_duration = 3;           % seconds

% =========================================================================
%% DISPLAY PARAMETERS
% =========================================================================
force_ylim  = [-30000 5000];   % pre-offset raw units (will rescale after offset)
emg_offset  = 20000;        % vertical spacing between EMG channels (raw units)
emg_ylim_std = [0 25000];      % for activity bar chart

% =========================================================================
%% TASK PARAMETERS
% =========================================================================
% Target profile types: 'trapezoid', 'mvc', 'ramp', 'constant'
% These are used by experiment_muovi.m to build the target trace
% trap_ramp_s   = 2;          % trapezoid ramp duration (s)
% trap_hold_s   = 5;          % trapezoid hold duration (s)
% trap_level    = 0.5;        % hold level as fraction of MVC (0–1)

% =========================================================================
%% SAVE SETUP
% =========================================================================
save(fullfile(datapath, 'setup.mat'), ...
    'dryrun', 'datapath', 'side', ...
    'TCPPort', 'sampFreq', 'num_muovi', ...
    'channels_per_muovi', 'channels_sync', 'total_channels', ...
    'force_left', 'force_right', 'force_sum', ...
    'emg_channels', ...
    'block_samples', 'ds', 'offsettime', ...
    'mvc_duration', ...
    'force_ylim', 'emg_offset', 'emg_ylim_std');
% 
% , ...
%     'trap_ramp_s', 'trap_hold_s', 'trap_level');

disp('Setup saved. Run experiment_muovi.m to begin.');