%% experiment_muovi.m — Force tracing + EMG recording via SyncStation
% Requires: setup_muovi.m to have been run first
close all; clear; clc;

% =========================================================================
%% LOAD SETUP
% =========================================================================
[fname, fpath] = uigetfile('*.mat', 'Select setup.mat');
load(fullfile(fpath, fname));

% =========================================================================
%% BUILD SYNCSTATION COMMAND STRING
% =========================================================================
DeviceEN = zeros(1,16);
EMG      = zeros(1,16);
Mode     = zeros(1,16);

if num_muovi >= 1, DeviceEN(5)=1; EMG(5)=1; Mode(5)=0; end
if num_muovi == 2, DeviceEN(6)=1; EMG(6)=1; Mode(6)=0; end

ConfString    = zeros(18,1);
ConfString(1) = sum(DeviceEN)*2 + 1;
idx = 2;
for i = 1:16
    if DeviceEN(i)
        ConfString(idx) = (i-1)*16 + EMG(i)*8 + Mode(i)*2 + 1;
        idx = idx + 1;
    end
end
ConfString(idx) = CRC8(ConfString, idx-1);
ConfLen = idx;

% =========================================================================
%% CONNECT TO SYNCSTATION
% =========================================================================
if dryrun
    disp('Dry-run mode: no hardware.');
    t = [];
else
    disp('Connecting to SyncStation...');
    try
        t = tcpclient('192.168.76.1', TCPPort, 'Timeout', 5);
    catch ME
        error('Connection failed: %s', ME.message);
    end

    t.InputBufferSize = total_channels * sampFreq * 3;
    flush(t);

    write(t, ConfString(1:ConfLen), 'uint8');
    pause(0.05);

    % Wait for first packet
    expected_bytes = total_channels * sampFreq * 2;
    timeout = tic;
    while t.NumBytesAvailable < expected_bytes
        if toc(timeout) > 5
            error('Timeout waiting for SyncStation data.');
        end
        pause(0.005);
    end

    % Flush the startup packet
    read(t, total_channels * sampFreq, 'int16');
    disp('SyncStation connected and streaming.');
end

% =========================================================================
%% MVC — collect before experiment
% =========================================================================
% Will be populated during MVC routine (key: m)
mvc_value = NaN;
mvc_force = [];
mvc_emg   = [];

% =========================================================================
%% TARGET TRACE — build trapezoid (in MVC fraction; scaled after MVC)
% =========================================================================
% Build in update-steps (each step = block_samples)
updates_per_sec = sampFreq / block_samples;

ramp_steps = round(trap_ramp_s  * updates_per_sec);
hold_steps = round(trap_hold_s  * updates_per_sec);

target_trace = [linspace(0, trap_level, ramp_steps), ...
                trap_level * ones(1, hold_steps),    ...
                linspace(trap_level, 0, ramp_steps)];
n_target = numel(target_trace);

% =========================================================================
%% FIGURE: FORCE
% =========================================================================
N_disp = round(10 * updates_per_sec);   % 10-second rolling window

force_buf_L = zeros(1, N_disp);
force_buf_R = zeros(1, N_disp);
force_buf_S = zeros(1, N_disp);

force_fig = figure('Color','w', 'WindowState','maximized', 'Name','Force');
ax_force = axes(force_fig);
hold(ax_force, 'on');

force_left_line  = plot(ax_force, 1:N_disp, force_buf_L, 'r',  'LineWidth', 1.5);
force_right_line = plot(ax_force, 1:N_disp, force_buf_R, 'b',  'LineWidth', 1.5);
force_sum_line   = plot(ax_force, 1:N_disp, force_buf_S, 'k',  'LineWidth', 2);
target_line      = plot(ax_force, 1:N_disp, NaN(1,N_disp), 'g--', 'LineWidth', 2);

legend(ax_force, {'Left','Right','Sum','Target'}, 'Location','northwest');
title(ax_force, 'Force — press M for MVC, T for task, Q to quit');
xlabel(ax_force, 'Updates');
ylabel(ax_force, 'Force (a.u.) / MVC fraction');
ylim(ax_force, force_ylim);
xlim(ax_force, [1 N_disp]);

% =========================================================================
%% FIGURE: EMG (indicator only — channels active = nonzero std)
% =========================================================================
n_emg = numel(emg_channels);
emg_std_buf = zeros(n_emg, 1);

emg_fig = figure('Color','w', 'Position',[50 50 400 600], 'Name','EMG activity');
ax_emg = axes(emg_fig);
emg_bar = bar(ax_emg, 1:n_emg, emg_std_buf, 'FaceColor','flat');
title(ax_emg, 'EMG channel activity (std)');
xlabel(ax_emg, 'Channel');
ylabel(ax_emg, 'Std (a.u.)');
%ylim(ax_emg, [0 200]);
ylime(ax_emg, emg_ylim_std);

% =========================================================================
%% KEY HANDLER & STATE
% =========================================================================
keyPressed = '';
set(force_fig, 'KeyPressFcn', @(~,e) assignin('base','keyPressed', e.Key));
set(emg_fig,   'KeyPressFcn', @(~,e) assignin('base','keyPressed', e.Key));

state = 'idle';     % 'idle' | 'mvc' | 'task'
k     = 0;          % update counter

% Storage (pre-allocated when task starts)
task_force = [];
task_emg   = [];
task_target = [];

disp('Ready. Keys: M = MVC, T = start task, Q = quit');

% =========================================================================
%% MAIN LOOP
% =========================================================================
while ~strcmp(keyPressed, 'q')

    %% READ BLOCK
    if dryrun
        data = randn(total_channels, block_samples) * 20;
        data(force_left,  :) = 200 + 20*randn(1, block_samples);
        data(force_right, :) = 180 + 20*randn(1, block_samples);
        data(force_sum,   :) = 380 + 20*randn(1, block_samples);
    else
        while t.NumBytesAvailable < total_channels * block_samples * 2
            pause(0.001);
        end
        Temp = read(t, total_channels * block_samples, 'int16');
        data = reshape(Temp, total_channels, block_samples);
    end

    %% EXTRACT SIGNALS
    force_L = mean(data(force_left,  :));
    force_R = mean(data(force_right, :));
    force_S = mean(data(force_sum,   :));
    emg_block = data(emg_channels, :);   % n_emg × block_samples

    %% NORMALISE FORCE IF MVC KNOWN
    if ~isnan(mvc_value) && mvc_value > 0
        disp_L = force_L / mvc_value;
        disp_R = force_R / mvc_value;
        disp_S = force_S / mvc_value;
    else
        disp_L = force_L;
        disp_R = force_R;
        disp_S = force_S;
    end

    %% UPDATE FORCE ROLLING BUFFER
    force_buf_L = [force_buf_L(2:end), disp_L];
    force_buf_R = [force_buf_R(2:end), disp_R];
    force_buf_S = [force_buf_S(2:end), disp_S];

    set(force_left_line,  'YData', force_buf_L);
    set(force_right_line, 'YData', force_buf_R);
    set(force_sum_line,   'YData', force_buf_S);

    %% UPDATE EMG ACTIVITY BARS
    emg_std_buf = std(double(emg_block), 0, 2);
    set(emg_bar, 'YData', emg_std_buf);

    %% KEY: START MVC
    if strcmp(keyPressed, 'm') && ~strcmp(state, 'mvc')
        keyPressed = '';
        state = 'mvc';
        disp('MVC started — recording...');

        mvc_n     = mvc_duration * sampFreq;   % total samples
        mvc_force_raw = zeros(1, mvc_n);
        mvc_emg_raw   = zeros(n_emg, mvc_n);
        col = 1;

        while col <= mvc_n
            if dryrun
                chunk_f = 200 + 20*randn(1, block_samples);
                chunk_e = randn(n_emg, block_samples) * 20;
            else
                while t.NumBytesAvailable < total_channels * block_samples * 2
                    pause(0.001);
                end
                Temp = read(t, total_channels * block_samples, 'int16');
                d    = reshape(Temp, total_channels, block_samples);
                chunk_f = d(force_sum, :);
                chunk_e = d(emg_channels, :);
            end

            idx_end = min(col + block_samples - 1, mvc_n);
            len     = idx_end - col + 1;
            mvc_force_raw(col:idx_end) = chunk_f(1:len);
            mvc_emg_raw(:, col:idx_end) = chunk_e(:, 1:len);
            col = col + len;
        end

        mvc_value = max(mvc_force_raw);
        mvc_force = mvc_force_raw;
        mvc_emg   = mvc_emg_raw;

        fprintf('MVC = %.1f\n', mvc_value);
        save(fullfile(datapath, 'mvc.mat'), 'mvc_value', 'mvc_force', 'mvc_emg');
        disp('MVC saved.');

        % Update y-axis to fraction scale
        ylim(ax_force, [0 1.2]);
        state = 'idle';
    end

    %% KEY: START TASK
    if strcmp(keyPressed, 't') && ~strcmp(state, 'task')
        if isnan(mvc_value)
            disp('Run MVC first (press M).');
            keyPressed = '';
        else
            keyPressed = '';
            state = 'task';
            task_k = 1;

            % Pre-allocate storage
            task_force  = zeros(3, n_target);    % L, R, Sum
            task_emg    = zeros(n_emg, block_samples * n_target);
            task_target = target_trace;

            disp('Task started.');
        end
    end

    %% TASK STATE: store data + show target
    if strcmp(state, 'task')
        if task_k <= n_target
            % Show current target on force plot
            target_buf = NaN(1, N_disp);
            t_pos = max(1, N_disp - task_k + 1);
            len   = min(task_k, N_disp);
            target_buf(t_pos:end) = target_trace(max(1,task_k-len+1):task_k);
            set(target_line, 'YData', target_buf);

            % Store
            task_force(1, task_k) = disp_L;
            task_force(2, task_k) = disp_R;
            task_force(3, task_k) = disp_S;

            col_start = (task_k-1)*block_samples + 1;
            task_emg(:, col_start:col_start+block_samples-1) = emg_block;

            task_k = task_k + 1;
        else
            % Task complete
            disp('Task complete. Saving...');
            save(fullfile(datapath, sprintf('task_%s.mat', datestr(now,'HHMMSS'))), ...
                'task_force', 'task_emg', 'task_target', 'mvc_value');
            disp('Task data saved.');
            set(target_line, 'YData', NaN(1,N_disp));
            state = 'idle';
        end
    end

    k = k + 1;
    drawnow limitrate;
end

% =========================================================================
%% STOP SYNCSTATION
% =========================================================================
if ~dryrun
    ConfStop = [0; CRC8(0, 1)];
    write(t, ConfStop, 'uint8');
    flush(t);
    clear t;
end

close all;
disp('Experiment finished.');