%% experiment_muovi.m — Force tracing + EMG recording via SyncStation
% Requires: setup_muovi.m to have been run first
function experiment_muovi(varargin)

% parser
close all
clc

% Default parameters
mvc_mode     = 'bilateral';   % 'left' | 'right' | 'bilateral'
mvc_override = [];            % numeric MVC value

trap_ramp_s = 2;
trap_hold_s = 10;
trap_level  = 0.2;
lead_in = 5;
task_shape   = 'sombrero';    % 'trap' | 'sombrero'

histLen = 20;

% Parse name/value pairs
for k = 1:2:numel(varargin)
    key = lower(varargin{k});
    val = varargin{k+1};
    switch key
        case 'mvc'
            mvc_mode = lower(val);
        case 'trap'
            trap_level = val / 100;   % user gives percent
        case 'mvcvalue'
            mvc_override = val;
        case 'shape'
            task_shape = lower(val);
        otherwise
            error('Unknown parameter: %s', key);
    end
end


% =========================================================================
%% LOAD SETUP
% =========================================================================
fpath = 'C:\Users\masgh\data\emgReaderData\';
fname = 'setup.mat';
%[fname, fpath] = uigetfile('*.mat', 'Select setup.mat');
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

[b_force, a_force] = butter(2, 50/(sampFreq/2), 'low');

% Before main loop — initialise filter states
zi_L = zeros(max(length(a_force), length(b_force)) - 1, 1);
zi_R = zeros(max(length(a_force), length(b_force)) - 1, 1);
zi_S = zeros(max(length(a_force), length(b_force)) - 1, 1);

force_hist_L = zeros(1, histLen);
force_hist_R = zeros(1, histLen);
force_hist_S = zeros(1, histLen);

% =========================================================================
%% COLLECT FORCE BASELINE OFFSET
% =========================================================================
fprintf('Collecting baseline — keep force at rest (%.1fs)...\n', offsettime);

offset_n     = round(offsettime * sampFreq);
offset_buf_L = zeros(1, offset_n);
offset_buf_R = zeros(1, offset_n);
offset_buf_S = zeros(1, offset_n);

if dryrun
    offset_buf_L(:) = -6790;
    offset_buf_R(:) = -24227;
    %offset_buf_S(:) = -8294;
    offset_buf_S(:) = offset_buf_L + offset_buf_R;
else
    col = 1;
    while col <= offset_n
        while t.NumBytesAvailable < total_channels * block_samples * 2
            pause(0.001);
        end
        Temp = read(t, total_channels * block_samples, 'int16');
        d    = reshape(Temp, total_channels, block_samples);
        len  = min(block_samples, offset_n - col + 1);
        offset_buf_L(col:col+len-1) = d(force_left,  1:len);
        offset_buf_R(col:col+len-1) = d(force_right, 1:len);
        offset_buf_S(col:col+len-1) = d(force_sum,   1:len);
        col = col + len;
    end
end

force_offset_L = mean(offset_buf_L);
force_offset_R = mean(offset_buf_R);
force_offset   = mean(offset_buf_S);
fprintf('Offsets — L: %.0f  R: %.0f  Sum: %.0f\n', force_offset_L, force_offset_R, force_offset);



% =========================================================================
%% MVC & TARGET — initialise
% =========================================================================
mvc_value    = 0;
mvc_best     = 0;      
mvc_force    = [];
mvc_emg      = [];
mvc_done     = false;

% If user supplied an MVC value, skip MVC acquisition
if ~isempty(mvc_override)
    mvc_value = mvc_override;
    mvc_done  = true;
    fprintf('Using user-supplied MVC = %.1f\n', mvc_value);
end

if ~isempty(trap_level)
    updates_per_sec = sampFreq / block_samples;
    ramp_steps      = round(trap_ramp_s * updates_per_sec);
    hold_steps      = round(trap_hold_s * updates_per_sec);
    lead_steps = round(lead_in * updates_per_sec);   % 3s flat lead-in and lead-out

    % target_trace = [linspace(0, trap_level, ramp_steps), ...
    %     trap_level * ones(1, hold_steps), ...
    %     linspace(trap_level, 0, ramp_steps)];

    % target_trace = [zeros(1, lead_steps),              ...
    %     linspace(0, trap_level, ramp_steps), ...
    %     trap_level * ones(1, hold_steps),    ...
    %     linspace(trap_level, 0, ramp_steps), ...
    %     zeros(1, lead_steps)];

    switch task_shape
        case 'trap'
            target_trace = [zeros(1, lead_steps), ...
                linspace(0, trap_level, ramp_steps), ...
                trap_level * ones(1, hold_steps), ...
                linspace(trap_level, 0, ramp_steps), ...
                zeros(1, lead_steps)];

        case 'sombrero'
            brim_level  = trap_level * 0.4;
            brim_steps  = round(1.5 * updates_per_sec);
            dip_steps   = round(1.0 * updates_per_sec);
            target_trace = [zeros(1, lead_steps), ...
                linspace(0, brim_level, ramp_steps), ...
                brim_level * ones(1, brim_steps), ...
                linspace(brim_level, trap_level, dip_steps), ...
                trap_level * ones(1, hold_steps), ...
                linspace(trap_level, brim_level, dip_steps), ...
                brim_level * ones(1, brim_steps), ...
                linspace(brim_level, 0, ramp_steps), ...
                zeros(1, lead_steps)];
    end

    n_target       = numel(target_trace);
    target_display = target_trace;
end




% =========================================================================
%% FIGURE: FORCE
% =========================================================================
N_disp = round(10 * updates_per_sec);   % 10-second rolling window
cursor_pos = round(0.7 * N_disp);

force_buf_L = zeros(1, N_disp);
force_buf_R = zeros(1, N_disp);
force_buf_S = zeros(1, N_disp);

%force_fig = figure('Color','w', 'WindowState','maximized', 'Name','Force');
force_fig = figure;
ax_force  = axes(force_fig);
hold(ax_force, 'on');

force_left_line  = plot(ax_force, 1:N_disp, force_buf_L, 'r',     'LineWidth', 1.5);
force_right_line = plot(ax_force, 1:N_disp, force_buf_R, 'b',     'LineWidth', 1.5);
force_sum_line   = plot(ax_force, 1:N_disp, force_buf_S, 'k',     'LineWidth', 2);
target_line      = plot(ax_force, 1:N_disp, NaN(1,N_disp), 'g', 'LineWidth', 5);

tracker_ball = plot(ax_force, cursor_pos, 0, 'ko', ...
    'MarkerSize', 16, 'MarkerFaceColor', 'k', 'Visible', 'off');

%cursor_line = xline(ax_force, round(0.7 * N_disp), 'k:', 'LineWidth', 1.5);

legend(ax_force, {'Left','Right','Sum','Target'}, 'Location','northwest');
title(ax_force, 'Force — press M for MVC, T for task, Q to quit');
xlabel(ax_force, 'Updates');
ylabel(ax_force, 'Force (raw units)');
ylim(ax_force, [-80000 80000]);
ax_force.YLimMode = 'manual';
xlim(ax_force, [1 N_disp]);

set(ax_force, 'XMinorGrid', 'on', 'YMinorGrid', 'on');

% =========================================================================
%% FIGURE: EMG ACTIVITY
% =========================================================================
n_emg       = numel(emg_channels);
emg_std_buf = zeros(n_emg, 1);

emg_fig = figure('Color','w', 'Position',[50 50 400 600], 'Name','EMG activity');
ax_emg  = axes(emg_fig);
emg_bar = bar(ax_emg, 1:n_emg, emg_std_buf, 'FaceColor','flat');
title(ax_emg, 'EMG channel activity (std)');
xlabel(ax_emg, 'Channel');
ylabel(ax_emg, 'Std (a.u.)');
ylim(ax_emg, emg_ylim_std);

% =========================================================================
%% FIGURE: SATURATION GAUGE
% =========================================================================
SAT_LIMIT = 32000;   % int16 max is 32767; flag at 32000 (98%)

sat_fig = figure('Color','w','Position',[500 50 200 500],'Name','Saturation');
ax_sat  = axes(sat_fig);
hold(ax_sat,'on');

% One vertical bar per force channel (L, R, Sum)
sat_bars = bar(ax_sat, 1:3, [0;0;0], 'FaceColor','flat');
sat_bars.CData = [0.2 0.6 1; 1 0.3 0.3; 0 0 0];  % blue, red, black

% Saturation threshold lines
yline(ax_sat,  SAT_LIMIT, 'r--', 'LineWidth', 1.5, 'Label', 'SAT+');
yline(ax_sat, -SAT_LIMIT, 'r--', 'LineWidth', 1.5, 'Label', 'SAT-');

ylim(ax_sat, [-32768, 32768]);
ax_sat.YLimMode = 'manual';
xticklabels(ax_sat, {'L','R','Sum'});
title(ax_sat, 'Raw ADC (saturation monitor)');
ylabel(ax_sat, 'int16 value');
set(ax_sat, 'YGrid', 'on');

% Colour the axes background red if saturated
sat_saturated = false;

% =========================================================================
%% KEY HANDLER & STATE
% =========================================================================
% keyPressed = '';
% set(force_fig, 'KeyPressFcn', @(~,e) assignin('base','keyPressed', e.Key));
% set(emg_fig,   'KeyPressFcn', @(~,e) assignin('base','keyPressed', e.Key));

guidata(force_fig, struct('pressed', ''));
set(force_fig, 'KeyPressFcn', @(src,e) guidata(src, setfield(guidata(src),'pressed',e.Key)));
set(emg_fig,   'KeyPressFcn', @(~,e)   guidata(force_fig, setfield(guidata(force_fig),'pressed',e.Key)));



state  = 'idle';
k      = 0;
task_k = 1;

task_force  = [];
task_emg    = [];
task_target = [];

%disp('Ready. Keys: M = MVC, T = start task, Q = quit');

disp('Ready. Keys: M = MVC, T = start task, O = recalibrate offset, Q = quit');




% =========================================================================
%% MAIN LOOP
% =========================================================================
%while ~strcmp(keyPressed, 'q')
while ~strcmp(guidata(force_fig).pressed, 'q')

    keyPressed = guidata(force_fig).pressed;

    %% READ BLOCK
    if dryrun
        % --- FORCE SIMULATION ---
        % Baseline noise
        fL = -7000  + 200*randn(1, block_samples);
        fR = -24000 + 200*randn(1, block_samples);

        % MVC behaviour: smooth Gaussian push
        if strcmp(state,'mvc')
            bump = 3000 * exp(-((1:block_samples)-block_samples/2).^2 / (2*(block_samples/4)^2));
            fL = fL - bump;
            fR = fR - bump;

            % Task behaviour: more irregular force
        elseif strcmp(state,'task')
            fL = fL - 1500*rand(1, block_samples);
            fR = fR - 1500*rand(1, block_samples);
        end

        % Bilateral sum (this is what was missing)
        fS = fL + fR;

        % Fill data matrix
        data = zeros(total_channels, block_samples);
        data(force_left,  :) = fL;
        data(force_right, :) = fR;
        data(force_sum,   :) = fS;   % <-- FIXED: now included

        % --- EMG SIMULATION ---
        emg_noise = 50 * randn(n_emg, block_samples);

        if strcmp(state,'mvc')
            emg_noise = emg_noise + 300 * randn(n_emg, block_samples);
        elseif strcmp(state,'task')
            emg_noise = emg_noise + 150 * randn(n_emg, block_samples);
        end

        data(emg_channels, :) = emg_noise;

    else
        while t.NumBytesAvailable < total_channels * block_samples * 2
            pause(0.001);
        end
        Temp = read(t, total_channels * block_samples, 'int16');
        data = reshape(Temp, total_channels, block_samples);
    end


    %% EXTRACT FORCE (offset-corrected, negated so push = positive)

    % fL = filtfilt(b_force, a_force, double(data(force_left,  :)));
    % fR = filtfilt(b_force, a_force, double(data(force_right, :)));
    % fS = filtfilt(b_force, a_force, double(data(force_sum,   :)));

    % Replace filtfilt lines in main loop with:
    % [fL, zi_L] = filter(b_force, a_force, double(data(force_left,  :)), zi_L);
    % [fR, zi_R] = filter(b_force, a_force, double(data(force_right, :)), zi_R);
    % [fS, zi_S] = filter(b_force, a_force, double(data(force_sum,   :)), zi_S);

    raw_mean_L = mean(double(data(force_left,  :)));
    raw_mean_R = mean(double(data(force_right, :)));
    raw_mean_S = mean(double(data(force_sum,   :)));

    force_hist_L = [force_hist_L(2:end), raw_mean_L - force_offset_L];
    force_hist_R = [force_hist_R(2:end), raw_mean_R - force_offset_R];
    force_hist_S = [force_hist_S(2:end), -(raw_mean_S - force_offset)];

    % force_hist_L = [force_hist_L(2:end), (mean(fL) - force_offset_L)];
    % force_hist_R = [force_hist_R(2:end), (mean(fR) - force_offset_R)];
    % force_hist_S = [force_hist_S(2:end), -(mean(fS) - force_offset)];

    force_L = mean(force_hist_L);
    force_R = mean(force_hist_R);
    force_S = mean(force_hist_S);

    % fL = double(data(force_left,  :));
    % fR = double(data(force_right, :));
    % fS = double(data(force_sum,   :));

    % THIS ONE
    % force_L = -(mean(fL) - force_offset_L);
    % force_R = -(mean(fR) - force_offset_R);
    % force_S = -(mean(fS) - force_offset);

    % force_L = -(mean(double(data(force_left,  :))) - force_offset_L);
    % force_R = -(mean(double(data(force_right, :))) - force_offset_R);
    % force_S = -(mean(double(data(force_sum,   :))) - force_offset);

    %% UPDATE SATURATION GAUGE
    raw_L   = mean(double(data(force_left,  :)));
    raw_R   = mean(double(data(force_right, :)));
    raw_S   = -mean(double(data(force_sum,   :)));
    set(sat_bars, 'YData', [raw_L; raw_R; raw_S]);

    % Flash background red if any channel is saturated
    is_sat = any(abs([raw_L raw_R raw_S]) >= SAT_LIMIT);
    if is_sat ~= sat_saturated
        sat_saturated = is_sat;
        set(ax_sat, 'Color', repmat(~is_sat, 1, 3) + is_sat*[1 0.8 0.8]);
    end

    %% NORMALISE IF MVC DONE
    if mvc_done
        disp_L = force_L / mvc_value;
        disp_R = force_R / mvc_value;
        disp_S = force_S / mvc_value;
    else
        disp_L = force_L;
        disp_R = force_R;
        disp_S = force_S;
    end

    % Add this after disp_L/R/S are calculated:
    switch mvc_mode
        case 'left',      disp_track = disp_L;
        case 'right',     disp_track = disp_R;
        case 'bilateral', disp_track = disp_S;
    end

    %% EXTRACT EMG
    emg_block = data(emg_channels, :);

    %% UPDATE FORCE PLOT
    force_buf_L = [force_buf_L(2:end), disp_L];
    force_buf_R = [force_buf_R(2:end), disp_R];
    % force_buf_S = [force_buf_S(2:end), disp_S];
    %
    set(force_left_line,  'YData', force_buf_L);
    set(force_right_line, 'YData', force_buf_R);
    % set(force_sum_line,   'YData', force_buf_S);

    if ~strcmp(state, 'task')
        force_buf_S = [force_buf_S(2:end), disp_S];
        set(force_sum_line, 'YData', force_buf_S);
    end


    if strcmp(state, 'task')
        set(tracker_ball, 'XData', cursor_pos, 'YData', disp_track);
    end

    %% UPDATE EMG BARS
    emg_std_buf = std(double(emg_block), 0, 2);
    set(emg_bar, 'YData', emg_std_buf);

    % =====================================================================
    %% KEY: MVC
    % =====================================================================
    if strcmp(keyPressed, 'm') && ~strcmp(state, 'mvc')
        guidata(force_fig, setfield(guidata(force_fig),'pressed',''));
        keyPressed = '';

        % Countdown
        for ct = 3:-1:1
            title(ax_force, sprintf('GET READY... %d', ct));
            set(ax_force, 'Color', [1 1 0.8]);
            drawnow;
            pause(1);
        end
        title(ax_force, '*** PUSH NOW ***');
        set(ax_force, 'Color', [1 0.85 0.85]);
        drawnow;

        state      = 'mvc';
        mvc_value  = 0;
        %disp('MVC started — push as hard as possible...')


        % Default offset for MVC — overridden by mvc_mode switch in hardware branch
        switch mvc_mode
            case 'left',     offset = force_offset_L;
            case 'right',    offset = force_offset_R;
            case 'bilateral', offset = force_offset;
        end

        mvc_n         = mvc_duration * sampFreq;
        mvc_force_raw = zeros(1, mvc_n);
        mvc_emg_raw   = zeros(n_emg, mvc_n);
        col = 1;

        % Flush stale data accumulated during countdown
        if ~dryrun
            flush(t);
        end

        while col <= mvc_n
            %% READ BLOCK
            if dryrun
                chunk_f = -8000 - 3000*rand(1, block_samples);
                chunk_e = randn(n_emg, block_samples) * 500;
                d_mvc   = zeros(total_channels, block_samples);
                d_mvc(force_left,  :) = chunk_f;
                d_mvc(force_right, :) = chunk_f;
                d_mvc(force_sum,   :) = chunk_f;
            else
                while t.NumBytesAvailable < total_channels * block_samples * 2
                    pause(0.001);
                end
                Temp  = read(t, total_channels * block_samples, 'int16');
                d_mvc = reshape(Temp, total_channels, block_samples);

                switch mvc_mode
                    case 'left',      chunk_f = double(d_mvc(force_left,:));
                    case 'right',     chunk_f = double(d_mvc(force_right,:));
                    case 'bilateral', chunk_f = double(d_mvc(force_sum,:));
                end
                chunk_e = d_mvc(emg_channels, :);
            end

            %% STORE RAW MVC DATA
            idx_end = min(col + block_samples - 1, mvc_n);
            len     = idx_end - col + 1;
            mvc_force_raw(col:idx_end)  = chunk_f(1:len);
            mvc_emg_raw(:, col:idx_end) = chunk_e(:, 1:len);
            col = col + len;

            %% UPDATE DISPLAY
            raw_L =  mean(double(d_mvc(force_left,  :))) - force_offset_L;
            raw_R =  mean(double(d_mvc(force_right, :))) - force_offset_R;
            raw_S = -(mean(double(d_mvc(force_sum,  :))) - force_offset);

            force_hist_L = [force_hist_L(2:end), raw_L];
            force_hist_R = [force_hist_R(2:end), raw_R];
            force_hist_S = [force_hist_S(2:end), raw_S];

            force_buf_L = [force_buf_L(2:end), mean(force_hist_L)];
            force_buf_R = [force_buf_R(2:end), mean(force_hist_R)];
            force_buf_S = [force_buf_S(2:end), mean(force_hist_S)];

            set(force_left_line,  'YData', force_buf_L);
            set(force_right_line, 'YData', force_buf_R);
            set(force_sum_line,   'YData', force_buf_S);

            set(sat_bars, 'YData', [mean(double(d_mvc(force_left,:))); ...
                mean(double(d_mvc(force_right,:))); ...
                mean(double(d_mvc(force_sum,:)))]);
            drawnow limitrate;
        end

        %% FINAL MVC VALUE

        % mvc_value = max(-(mvc_force_raw - offset));
        % mvc_force = mvc_force_raw;
        % mvc_emg   = mvc_emg_raw;
        % mvc_done  = true;
        % 
        % set(ax_force, 'Color', 'w');
        % title(ax_force, sprintf('MVC = %.1f  |  M=MVC  T=task  Q=quit', mvc_value));
        % 
        % fprintf('MVC = %.1f\n', mvc_value);
        % save(fullfile(datapath, 'mvc.mat'), 'mvc_value', 'mvc_force', 'mvc_emg');
        % disp('MVC saved.');
        % 
        % y_max = trap_level * 2;
        % ylim(ax_force, [-y_max * 0.5, y_max * 1.5]);
        % ax_force.YLimMode = 'manual';
        % ylabel(ax_force, 'Force (MVC fraction)');

        mvc_value = max(-(mvc_force_raw - offset));
        mvc_force = mvc_force_raw;
        mvc_emg   = mvc_emg_raw;

        % Plot MVC trace for review
        mf = figure;
        plot(mvc_force_raw, 'b', 'LineWidth', 1.5);
        %yline(-(mvc_value + offset), 'r--', sprintf('Peak: %.0f', mvc_value));
        plot([1 mvc_n], [-(mvc_value+offset) -(mvc_value+offset)], 'r--');
        text(mvc_n*0.7, -(mvc_value+offset), sprintf('Peak: %.0f', mvc_value));

        title('MVC trace — review before accepting');
        xlabel('Samples'); ylabel('Raw ADC');

        keepLooping = true;
        while keepLooping
            choice = questdlg( ...
                sprintf('New MVC = %.1f   Best so far = %.1f\nKeep this attempt?', mvc_value, mvc_best), ...
                'MVC Confirmation', 'Yes', 'No', 'Select range', 'Yes');

            switch choice
                case 'Yes'
                    keepLooping = false;
                    if mvc_value > mvc_best
                        mvc_best = mvc_value;
                        fprintf('New best MVC = %.1f\n', mvc_best);
                    else
                        fprintf('MVC %.1f did not beat best (%.1f) — best kept\n', mvc_value, mvc_best);
                    end
                    mvc_done = true;

                case 'No'
                    keepLooping = false;
                    disp('MVC attempt discarded.');
                    mvc_done = (mvc_best > 0);
                    mvc_value = mvc_best;   % reset to best (or 0 if none accepted)

                case 'Select range'
                    disp('Click two points on the figure to select range.');
                    [x, ~] = ginput(2);
                    new_start = max(1,          round(min(x)));
                    new_end   = min(mvc_n, round(max(x)));
                    fprintf('Range selected: [%d, %d]\n', new_start, new_end);
                    mvc_value = max(-(mvc_force_raw(new_start:new_end) - offset));

                    % Update plot
                    hold on;
                    xline(new_start, 'g--');
                    xline(new_end,   'g--');
                    plot([new_start new_end], [-(mvc_value+offset) -(mvc_value+offset)], 'g--');

                    %yline(-(mvc_value + offset), 'g--', sprintf('Range peak: %.0f', mvc_value));
                    hold off;
            end
        end
        close(mf);
        drawnow;
        pause(0.1);
        figure(force_fig);
        drawnow;


        % Always use best accepted value for normalisation
        mvc_value = mvc_best;

        set(ax_force, 'Color', 'w');
        if mvc_done
            title(ax_force, sprintf('Best MVC = %.1f  |  M=MVC  T=task  Q=quit', mvc_best));
            y_max = trap_level * 2;
            ylim(ax_force, [-y_max * 0.1, y_max * 1.1]);
            ax_force.YLimMode = 'manual';
            ylabel(ax_force, 'Force (MVC fraction)');
            save(fullfile(datapath, 'mvc.mat'), 'mvc_best', 'mvc_force', 'mvc_emg');
            fprintf('MVC saved. Best = %.1f\n', mvc_best);
        else
            title(ax_force, 'No MVC accepted yet  |  M=MVC  T=task  Q=quit');
        end


        state = 'idle';
    end

    % =====================================================================
    %% KEY: RECALIBRATE OFFSET
    % =====================================================================
    if strcmp(keyPressed, 'o')
        guidata(force_fig, setfield(guidata(force_fig),'pressed',''));
        keyPressed = '';

        fprintf('Recalibrating offsets — release force (%.1fs)...\n', offsettime);
        title(ax_force, 'Recalibrating offset — release force...');
        drawnow;

        reoffset_n = round(offsettime * sampFreq);
        rebuf_L = zeros(1, reoffset_n);
        rebuf_R = zeros(1, reoffset_n);
        rebuf_S = zeros(1, reoffset_n);
        col = 1;

        if dryrun
            rebuf_L(:) = mean(force_buf_L) * mvc_value;  % approximate from display
            rebuf_R(:) = mean(force_buf_R) * mvc_value;
            rebuf_S(:) = rebuf_L + rebuf_R;
        else
            while col <= reoffset_n
                while t.NumBytesAvailable < total_channels * block_samples * 2
                    pause(0.001);
                end
                Temp = read(t, total_channels * block_samples, 'int16');
                d    = reshape(Temp, total_channels, block_samples);
                len  = min(block_samples, reoffset_n - col + 1);
                rebuf_L(col:col+len-1) = d(force_left,  1:len);
                rebuf_R(col:col+len-1) = d(force_right, 1:len);
                rebuf_S(col:col+len-1) = d(force_sum,   1:len);
                col = col + len;
            end
        end

        force_offset_L = mean(rebuf_L);
        force_offset_R = mean(rebuf_R);
        force_offset   = mean(rebuf_S);
        fprintf('New offsets — L: %.0f  R: %.0f  Sum: %.0f\n', ...
            force_offset_L, force_offset_R, force_offset);
        title(ax_force, 'Force — press M for MVC, T for task, O for offset, Q to quit');
    end


    % =====================================================================
    %% KEY: START TASK
    % =====================================================================
    if strcmp(keyPressed, 't') && ~strcmp(state, 'task')
        if ~mvc_done
            disp('Run MVC first (press M).');
            guidata(force_fig, setfield(guidata(force_fig),'pressed',''));
            keyPressed = '';
        else
            guidata(force_fig, setfield(guidata(force_fig),'pressed',''));
            keyPressed = '';

            % Countdown
            for ct = 3:-1:1
                title(ax_force, sprintf('Starting in %d...', ct));
                drawnow;
                pause(1);
            end
            if ~dryrun
                flush(t);
            end

            title(ax_force, 'Force — press M for MVC, T for task, Q to quit');
            state      = 'task';
            task_k     = 1;

            set(tracker_ball, 'Visible', 'on');
            switch mvc_mode
                case 'left',      set(tracker_ball, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
                case 'right',     set(tracker_ball, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
                case 'bilateral', set(tracker_ball, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
            end
            set(force_sum_line, 'Visible', 'off');

            task_force  = zeros(4, n_target);
            task_emg    = zeros(n_emg, block_samples * n_target);
            task_target = target_display;

            disp('Task started.');
        end
    end

    % =====================================================================
    %% TASK STATE
    % =====================================================================
    if strcmp(state, 'task')
        if task_k <= n_target
            % % Build scrolling target buffer
            % target_buf = NaN(1, N_disp);
            % t_pos      = max(1, N_disp - task_k + 1);
            % len        = min(task_k, N_disp);
            % target_buf(t_pos:end) = target_display(max(1, task_k-len+1):task_k);
            % set(target_line, 'YData', target_buf);


            lookahead  = round(0.3 * N_disp);   % current position 30% from right
            cursor_pos = N_disp - lookahead;    % x position of "now"

            % Fill target buffer centred on cursor_pos
            target_buf = NaN(1, N_disp);
            for x = 1:N_disp
                t_idx = task_k + (x - cursor_pos);
                if t_idx >= 1 && t_idx <= n_target
                    target_buf(x) = target_display(t_idx);
                end
            end
            set(target_line, 'YData', target_buf);

            % Store
            task_force(1, task_k) = disp_track;   % tracked channel (L, R, or bilateral)
            task_force(2, task_k) = disp_L;
            task_force(3, task_k) = disp_R;
            task_force(4, task_k) = disp_S;

            col_start = (task_k-1)*block_samples + 1;
            task_emg(:, col_start:col_start+block_samples-1) = emg_block;

            task_k = task_k + 1;
        else
            disp('Task complete. Saving...');
            save(fullfile(datapath, sprintf('task_%s.mat', datestr(now,'HHMMSS'))), ...
                'task_force', 'task_emg', 'task_target', 'mvc_value', 'mvc_mode');
            disp('Task data saved.');
            set(target_line, 'YData', NaN(1, N_disp));
            state = 'idle';

            set(tracker_ball, 'Visible', 'off');
            set(force_sum_line, 'Visible', 'on');
        end
    end
    %pause(block_samples / sampFreq);   % slow dry-run to real-time
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


end