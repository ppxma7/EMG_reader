%% setup_muovi.m
close all; clear; clc;

%% === USER OPTIONS ===
dryrun      = 0;                % 1 = no hardware, fake data
datapath    = uigetdir([], 'Select folder to save setup');
side        = 'right';          % reserved for future bilateral support

%% === MUOVI+ PARAMETERS ===
TCPPort     = 54321;
ProbeEN     = 1;                % enable probe
EMGmode     = 1;                % EMG = 2000 Hz
Mode        = 0;                % 64ch monopolar
NumChan     = 70;               % 64 EMG + 6 IMU
sampFreq    = 2000;             % Hz

%% === FORCE CHANNEL ===
% Your load cell amplifier is wired into AUX → appears as channel 65
force_channel = 65;

%% === EMG CHANNELS ===
% Muovi+ EMG channels = 1:64
emg_channels = 1:64;

%% === FEEDBACK PARAMETERS ===
rate        = 1/15.625;         % update rate
offsettime  = 0.5;              % seconds for offset collection

%% === PLOT SETTINGS ===
force_ylim   = [-500 500];       % adjust as needed
force_title  = 'Force Feedback';

emg_offset   = 200;              % vertical spacing between EMG channels
emg_ylim     = [0, 64*emg_offset];

%% === SAVE SETUP ===
save(fullfile(datapath,'setup.mat'), ...
    'dryrun','datapath','side', ...
    'TCPPort','ProbeEN','EMGmode','Mode','NumChan','sampFreq', ...
    'force_channel','emg_channels','rate','offsettime', ...
    'force_ylim','force_title','emg_offset','emg_ylim');

disp('Setup saved successfully.');
