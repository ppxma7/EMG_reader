%% qa_syncstation.m — Diagnostic Tool for Muovi+ via SyncStation
close all; clear; clc;

% =========================================================================
%% USER OPTIONS
% =========================================================================
dryrun    = 0;          % 1 = no hardware
num_muovi = 2;          % 1 or 2 Muovi+ devices

TCPPort  = 54320;       % SyncStation TCP port
sampFreq = 2000;        % EMG sampling rate (Hz)

% =========================================================================
%% CHANNEL LAYOUT
% =========================================================================
channels_per_muovi = 70;    % 64 EMG + 6 AUX per Muovi+
channels_sync      = 6;     % SyncStation aux channels
total_channels     = num_muovi * channels_per_muovi + channels_sync;

% Force channels (absolute indices in full data matrix)
force_left  = 141;
force_right = 142;
force_sum   = 144;

% EMG channel indices (absolute, both Muovi+ devices)
emg_idx = [1:64, 71:134];

% =========================================================================
%% BUILD SYNCSTATION COMMAND STRING
% =========================================================================
DeviceEN = zeros(1,16);
EMG      = zeros(1,16);
Mode     = zeros(1,16);

% Muovi+ devices occupy slots 5 and 6 in SyncStation protocol
if num_muovi >= 1
    DeviceEN(5) = 1; EMG(5) = 1; Mode(5) = 0;
end
if num_muovi == 2
    DeviceEN(6) = 1; EMG(6) = 1; Mode(6) = 0;
end

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
    disp('Dry-run mode: no hardware connected.');
    t = [];
else
    disp('Connecting to SyncStation...');
    try
        t = tcpclient('192.168.76.1', TCPPort, 'Timeout', 1);
    catch ME
        error('Could not connect to SyncStation: %s', ME.message);
    end
    disp('TCP connection opened.');

    t.InputBufferSize = total_channels * sampFreq * 3;
    flush(t);

    % Send config
    write(t, ConfString(1:ConfLen), 'uint8');
    pause(0.05);

    % Wait for first packet (1 second of data)
    expected_bytes = total_channels * sampFreq * 2;   % int16 = 2 bytes
    timeout = tic;
    while t.NumBytesAvailable < expected_bytes
        if toc(timeout) > 5
            error('Timeout: SyncStation not sending data after 5s.');
        end
        pause(0.005);
    end
    disp(['First packet received: ', num2str(t.NumBytesAvailable), ' bytes.']);

    % Read and validate
    Temp = read(t, total_channels * sampFreq, 'int16');
    if numel(Temp) ~= total_channels * sampFreq
        error('Packet size mismatch. Check device count or Mode.');
    end
 
end

% =========================================================================
%% FIGURE SETUP
% =========================================================================
%fig = figure('Color','w', 'WindowState','maximized');

fig = figure;
tiledlayout(2, 1, 'TileSpacing','compact', 'Padding','compact');

%% Force axis
N_force = 500;      % rolling window length (samples/updates)
force_buf_L = zeros(1, N_force);
force_buf_R = zeros(1, N_force);
force_buf_S = zeros(1, N_force);

ax_force = nexttile;
hold(ax_force, 'on');
force_left_line  = plot(ax_force, 1:N_force, force_buf_L, 'r', 'LineWidth', 2);
force_right_line = plot(ax_force, 1:N_force, force_buf_R, 'b', 'LineWidth', 2);
force_sum_line   = plot(ax_force, 1:N_force, force_buf_S, 'k', 'LineWidth', 2);
legend(ax_force, {'Left','Right','Sum'}, 'Location','northwest');
title(ax_force, 'Force Tap Test');
xlabel(ax_force, 'Updates');
ylabel(ax_force, 'Force (a.u.)');
xlim(ax_force, [1 N_force]);

%% EMG axis
ds     = 20;        % downsample factor for display
offset = 20000;     % vertical spacing between channels
n_emg  = 64 * num_muovi;
n_disp = sampFreq / ds;    % display samples per buffer

emg_buffer = zeros(n_emg, n_disp);

ax_emg = nexttile;
hold(ax_emg, 'on');
title(ax_emg, 'EMG Tap Test (64 channels per Muovi+)');
xlabel(ax_emg, 'Samples (downsampled)');
ylabel(ax_emg, 'Channel');
ylim(ax_emg, [0, n_emg * offset]);
xlim(ax_emg, [1, n_disp]);

emg_lines = gobjects(n_emg, 1);
for ch = 1:n_emg
    emg_lines(ch) = plot(ax_emg, 1:n_disp, zeros(1, n_disp), ...
        'Color', [0.5 0.5 0.5], 'LineWidth', 0.5);
end

% =========================================================================
%% LATENCY TRACKING
% =========================================================================
lat_buf = zeros(1, 500);
lat_idx = 1;

% =========================================================================
%% MAIN LOOP
% =========================================================================
block    = 200;     % samples to read per update
n_shift  = block / ds;

disp('QA running — press Q to quit.');
keyPressed = '';
set(fig, 'KeyPressFcn', @(~,e) assignin('base','keyPressed', e.Key));

while ~strcmp(keyPressed, 'q')
    loop_start = tic;

    %% Read data block
    if dryrun
        data = randn(total_channels, block) * 20;
        data(force_left,  :) = 200 + 20*randn(1, block);
        data(force_right, :) = 180 + 20*randn(1, block);
        data(force_sum,   :) = 380 + 20*randn(1, block);
    else
        while t.NumBytesAvailable < total_channels * block * 2
            pause(0.001);
        end
        Temp = read(t, total_channels * block, 'int16');
        data = reshape(Temp, total_channels, block);

        % chans = [65:70, 135:140, 141:146];
        % stds  = arrayfun(@(i) std(double(data(i,:))), chans);
        % [sorted_stds, sort_idx] = sort(stds, 'descend');
        % fprintf('ch %d: std=%.0f\n', [chans(sort_idx); round(sorted_stds)]);
    end

    %% Force: rolling buffer update
    force_buf_L = [force_buf_L(2:end),  mean(data(force_left,  :))];
    force_buf_R = [force_buf_R(2:end),  mean(data(force_right, :))];
    force_buf_S = [force_buf_S(2:end),  mean(data(force_sum,   :))];

    set(force_left_line,  'YData', force_buf_L);
    set(force_right_line, 'YData', force_buf_R);
    set(force_sum_line,   'YData', force_buf_S);

    %% EMG: extract, downsample, scroll buffer
    emg_chunk = zeros(n_emg, n_shift);
    for d = 1:num_muovi
        src = (d-1)*channels_per_muovi + 1;
        dst = (d-1)*64 + 1;
        emg_chunk(dst:dst+63, :) = data(src:src+63, 1:ds:end);
    end

    emg_buffer          = circshift(emg_buffer, -n_shift, 2);
    emg_buffer(:, end-n_shift+1:end) = emg_chunk;

    %% Update EMG plot
    for ch = 1:n_emg
        set(emg_lines(ch), 'YData', emg_buffer(ch,:) + (ch-1)*offset);
    end

    %% Latency tracking
    lat_buf(lat_idx) = toc(loop_start) * 1000;   % ms
    lat_idx = mod(lat_idx, numel(lat_buf)) + 1;

    drawnow limitrate;
end

%find_force_live(t, [141, 142, 144], total_channels, ["A", "B", "C"]);

find_force_live(t, [145], total_channels);


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
fprintf('QA finished. Mean loop latency: %.1f ms\n', mean(lat_buf));