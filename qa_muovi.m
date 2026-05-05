%% qa_muovi.m — Muovi+ Diagnostic Tool
close all; clear; clc;

%% === USER OPTIONS ===
dryrun = 0;                 % 1 = no hardware
TCPPort = 54321;
NumChan = 70;               % 64 EMG + 6 IMU
sampFreq = 2000;            % Hz
force_channel = 65;         % load cell amplifier
emg_channels = 1:64;

%% === MUOVI+ COMMAND ===
ProbeEN = 1;
EMGmode = 1;                % EMG = 2000 Hz
Mode = 0;                   % 64ch monopolar
Command = EMGmode*8 + Mode*2 + ProbeEN;

blockData = 2 * NumChan * sampFreq;

%% === INITIALISE MUOVI OR DRYRUN ===
if dryrun
    disp('Dry-run mode: no hardware connected.');
else
    t = tcpserver(TCPPort,"ByteOrder","big-endian");
    t.InputBufferSize = 500000;

    % Wait for connection
    while t.Connected < 1, pause(0.05); end

    % Flush buffer
    flush(t);

    % Send command
    fwrite(t, Command, "uint8");
end

%% === SETUP FIGURE ===
fig = figure('Color','w','WindowState','maximized');
tiledlayout(2,1);

%% FORCE AXIS
ax_force = nexttile;
hold(ax_force,'on');
force_line = plot(ax_force, 0, 0, 'r', 'LineWidth', 2);
title(ax_force,'Force (Tap Test)');
xlabel(ax_force,'Samples');
ylabel(ax_force,'Force');
ylim(ax_force,[-500 500]);

%% EMG AXIS (downsampled for speed)
ax_emg = nexttile;
hold(ax_emg,'on');
title(ax_emg,'EMG Tap Test (64 channels, downsampled)');
xlabel(ax_emg,'Samples');
ylabel(ax_emg,'Channels');

ds = 20;                    % downsample factor for display
offset = 200;               % vertical spacing
emg_lines = gobjects(64,1);

for ch = 1:64
    emg_lines(ch) = plot(ax_emg, zeros(1, sampFreq/ds), ...
        'LineWidth', 1);
end

ylim(ax_emg, [0 64*offset]);
xlim(ax_emg, [1 sampFreq/ds]);

%% === LATENCY MEASUREMENT ===
latency_history = zeros(1,2000);
lat_idx = 1;

%% === MAIN LOOP ===
disp('QA running: press q to quit.');
keyPressed = '';
set(fig,'KeyPressFcn',@(src,event) assignin('base','keyPressed',event.Key));

k = 0;

while ~strcmp(keyPressed,'q')

    loop_start = tic;

    %% --- READ DATA ---
    if dryrun
        data = randn(NumChan, sampFreq)*20;
        data(force_channel,:) = 200 + 20*randn(1,sampFreq);
    else
        while t.BytesAvailable < blockData, end
        Temp = fread(t, NumChan*sampFreq, "int16");
        data = reshape(Temp, NumChan, sampFreq);
    end

    %% --- FORCE ---
    force_raw = mean(data(force_channel,:));

    %% --- EMG (downsample for display) ---
    emg_block = data(1:64, 1:ds:end);

    %% --- UPDATE FORCE PLOT ---
    k = k + 1;
    set(force_line, 'XData', [get(force_line,'XData') k], ...
                    'YData', [get(force_line,'YData') force_raw]);

    %% --- UPDATE EMG PLOT ---
    for ch = 1:64
        set(emg_lines(ch), ...
            'YData', emg_block(ch,:) + (ch-1)*offset);
    end

    %% --- LATENCY ---
    latency_history(lat_idx) = toc(loop_start);
    lat_idx = lat_idx + 1;
    if lat_idx > length(latency_history)
        lat_idx = 1;
    end

    drawnow limitrate
end

%% === STOP MUOVI ===
if ~dryrun
    fwrite(t, Command-1, "uint8");
    pause(0.2);
    flush(t);
    clear t;
end

close all;
disp('QA finished.');
