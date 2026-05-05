%% experiment_muovi.m
close all; clear; clc;

%% === LOAD SETUP ===
[filename, pathname] = uigetfile('*.mat', 'Select setup.mat');
load(fullfile(pathname, filename));

%% === INITIALISE MUOVI OR DRYRUN ===
Command = EMGmode*8 + Mode*2 + ProbeEN;
blockData = 2 * NumChan * sampFreq;

if dryrun
    disp('Dry-run mode: no hardware connected.');
else
    t = tcpserver(TCPPort,"ByteOrder","big-endian");
    t.InputBufferSize = 500000;
    while t.Connected < 1, pause(0.05); end
    fwrite(t, Command, "uint8");
end

%% === FORCE PLOT ===
force_fig = figure('Color','w','WindowState','maximized');
force_ax = axes(force_fig);
hold(force_ax,'on');
force_line = plot(force_ax, 0, 0, 'r', 'LineWidth', 2);
title(force_ax, force_title);
xlabel(force_ax, 'Samples');
ylabel(force_ax, 'Force (a.u.)');
ylim(force_ax, force_ylim);

%% === EMG PLOT (64 channels stacked) ===
emg_fig = figure('Color','w','WindowState','maximized');
emg_ax = axes(emg_fig);
hold(emg_ax,'on');
title(emg_ax,'EMG (64 channels)');
xlabel(emg_ax,'Samples');
ylabel(emg_ax,'Channels');

emg_lines = gobjects(64,1);
for ch = 1:64
    emg_lines(ch) = plot(emg_ax, zeros(1,sampFreq), 'LineWidth', 1);
end

ylim(emg_ax, emg_ylim);
xlim(emg_ax, [1 sampFreq]);

%% === MVC STORAGE ===
mvc_duration = 3;                       % seconds
mvc_frames   = mvc_duration * sampFreq;
mvc_force    = zeros(1, mvc_frames);
mvc_emg      = zeros(64, sampFreq, mvc_frames);

%% === MAIN LOOP ===
disp('Press m to start MVC, q to quit.');
keyPressed = '';
set(force_fig,'KeyPressFcn',@(src,event) assignin('base','keyPressed',event.Key));

k = 0;

while ~strcmp(keyPressed,'q')

    %% --- READ DATA ---
    if dryrun
        data = randn(NumChan, sampFreq)*20;
        data(force_channel,:) = 200 + 20*randn(1,sampFreq);
    else
        while t.BytesAvailable < blockData, end
        Temp = fread(t, NumChan*sampFreq, "int16");
        data = reshape(Temp, NumChan, sampFreq);
    end

    %% --- EXTRACT FORCE ---
    force_raw = mean(data(force_channel,:));

    %% --- EXTRACT EMG ---
    emg_block = data(1:64, :);   % 64 × sampFreq

    %% --- UPDATE FORCE PLOT ---
    k = k + 1;
    set(force_line, 'XData', [get(force_line,'XData') k], ...
                    'YData', [get(force_line,'YData') force_raw]);

    %% --- UPDATE EMG PLOT ---
    for ch = 1:64
        set(emg_lines(ch), ...
            'YData', emg_block(ch,:) + (ch-1)*emg_offset);
    end

    drawnow limitrate

    %% === START MVC ===
    if strcmp(keyPressed,'m')
        keyPressed = '';
        disp('Starting MVC...');
        mvc_force = zeros(1, mvc_frames);
        mvc_emg   = zeros(64, sampFreq, mvc_frames);

        for i = 1:mvc_frames
            if dryrun
                force_raw = 200 + 20*randn;
                emg_block = randn(64, sampFreq)*20;
            else
                while t.BytesAvailable < blockData, end
                Temp = fread(t, NumChan*sampFreq, "int16");
                data = reshape(Temp, NumChan, sampFreq);
                force_raw = mean(data(force_channel,:));
                emg_block = data(1:64,:);
            end

            mvc_force(i) = force_raw;
            mvc_emg(:,:,i) = emg_block;
        end

        mvc_value = max(mvc_force);
        disp(['MVC = ', num2str(mvc_value)]);

        save(fullfile(datapath,'mvc_value.mat'), ...
            'mvc_value','mvc_force','mvc_emg');

        disp('MVC saved.');
    end
end

%% === STOP MUOVI ===
if ~dryrun
    fwrite(t, Command-1, "uint8");
    pause(0.2);
    clear t;
end

close all;
disp('Experiment finished.');
