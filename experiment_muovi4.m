%% experimentMuoviDraft.m
% Live force plot + EMG recording via SyncStation
% uint8 read

close all; clear all; clc;

subject = 'sub01';   % set per participant

force_dir = 'push'; % set this to push or pull

preComputedMVC = 3.102;   % set to a value e.g. 9000 to skip MVC, [] to require MVC

mvc_duration = 3;

task_shape  = 'trap';   % 'trap' | 'sombrero' | 'mcon'
task_level  = 0.5;          % target as fraction of MVC
task_leg    = 'bilateral';  % 'left' | 'right' | 'bilateral'
trap_ramp_s = 2;
trap_hold_s = 10;
lead_in_s   = 5;

mcon_cycles = 8;

% Force channels are 16-bit signed integers (-32768 to +32767 counts)
% Hardware input range is ±2.5V (5V total span)
% Scale factor converts raw counts back to volts: 5V / 2^16 counts
% hardware measures a voltage (say -2.5V to +2.5V), 
% converts it to a 16-bit signed integer (-32768 to +32767), 
% and sends it over TCP as two bytes. 
% readBlock reassembles those two bytes back into the integer, and 
% force_scale converts that integer back to volts.
force_scale = 5.0 / 65536;


% true to see EMG signals
show_emg_traces = false;
% =========================================================================
% USER OPTIONS
% =========================================================================
TCPPort      = 54320;
sampFreq     = 2000;
blockSamples = 50; % drop this lower to make things smoother
% DISPLAY RATE MATHS:
% Hardware streams at sampFreq = 2000 samples/sec continuously.
% We read in chunks of blockSamples at a time.
% Time to accumulate one block = blockSamples / sampFreq
%   blockSamples=200 → 200/2000 = 0.100s → max 10 updates/sec
%   blockSamples=100 → 100/2000 = 0.050s → max 20 updates/sec
%   blockSamples=50  →  50/2000 = 0.025s → max 40 updates/sec
% Actual update rate will be lower due to MATLAB processing overhead.
% For smooth MCON display, aim for >20 updates/sec → blockSamples <= 100.
% Tradeoff: smaller blockSamples = more TCP reads per second.

TotNumChan    = 146; % total number of channels 64+6 AUX (Empty) per Muovi)x2 = 140. Then 6 AUX Syncstation
TotNumByte    = 292;
bytesPerBlock = TotNumByte * blockSamples;

% this took weeks to figure out
force_left  = 141;
force_right = 142;
force_sum   = 143;

n_emg         = 128;          % 64 per Muovi+
emg_channels  = [1:64, 71:134];
datapath      = 'C:\Users\masgh\data\emgReaderData\';

ConvFact = 0.000286;   % converts raw ADC to mV for EMG

emg_ylim_std = [0 500];   % ignored - this for EMG activity std figure
mvc_value = 0;

% =========================================================================
% CONFIG STRING
% =========================================================================
ConfString    = zeros(18,1);
ConfString(1) = 2*2 + 1;
ConfString(2) = (5-1)*16 + 1*8 + 0*2 + 1;
ConfString(3) = (6-1)*16 + 1*8 + 0*2 + 1;
ConfString(4) = CRC8(ConfString, 3);

% =========================================================================
% CONNECT
% =========================================================================
tcpSocket = tcpclient('192.168.76.1', TCPPort);
tcpSocket.InputBufferSize = TotNumChan * sampFreq * 3;
write(tcpSocket, ConfString(1:4), 'uint8');
disp('Connected. Collecting baseline — keep force at rest...');

% =========================================================================
% BASELINE OFFSET (2 seconds)
% =========================================================================
baseline_samples = sampFreq * 2;
baseline_buf = zeros(3, baseline_samples);
col = 1;
while col <= baseline_samples
    while tcpSocket.BytesAvailable < bytesPerBlock, pause(0.001); end
    D = readBlock(tcpSocket, TotNumByte, blockSamples);
    len = min(blockSamples, baseline_samples - col + 1);
    baseline_buf(1, col:col+len-1) = D(force_left,  1:len);
    baseline_buf(2, col:col+len-1) = D(force_right, 1:len);
    %baseline_buf(3, col:col+len-1) = D(force_sum,   1:len);
    baseline_buf(3, col:col+len-1) = D(force_left, 1:len) + D(force_right, 1:len);
    col = col + len;
end
offset_L = mean(baseline_buf(1,:));
offset_R = mean(baseline_buf(2,:));
offset_S = mean(baseline_buf(3,:));
fprintf('Offsets — L:%.0f  R:%.0f  Sum:%.0f\n', offset_L, offset_R, offset_S);

% =========================================================================
% FIGURES
% =========================================================================
%% FORCE FIGURE
N = round(10 * sampFreq / blockSamples);   % 10s rolling window
buf_L = zeros(1,N);
buf_R = zeros(1,N);
buf_S = zeros(1,N);

force_fig = figure('Color','w','Name','Force');
ax = axes(force_fig); hold(ax,'on');
set(ax, 'XGrid','on','YGrid','on','XMinorGrid','on','YMinorGrid','on');

hl = plot(ax, 1:N, buf_L, 'r', 'LineWidth',1.5);
hr = plot(ax, 1:N, buf_R, 'b', 'LineWidth',1.5);
hs = plot(ax, 1:N, buf_S, 'k', 'LineWidth',2);
legend(ax,{'Left','Right','Sum'},'Location','northwest');
title(ax,'Force — M=MVC  T=task  O=offset  Q=quit');
xlabel(ax,'Updates'); ylabel(ax,'Force (offset-corrected ADC)');
ax.YLimMode = 'auto';
xlim(ax,[1 N]);

guidata(force_fig, struct('pressed',''));
set(force_fig,'KeyPressFcn',@(src,e) guidata(src,setfield(guidata(src),'pressed',e.Key)));

if ~isempty(preComputedMVC)
    mvc_value = preComputedMVC;
    ylabel(ax, 'Force (MVC fraction)');
    fprintf('Using precomputed MVC = %.0f\n', mvc_value);
end

% FIGURE: EMG ACTIVITY
% =========================================================================
% n_emg       = numel(emg_channels);
% emg_std_buf = zeros(n_emg, 1);
% 
% emg_fig = figure('Color','w', 'Position',[50 50 400 600], 'Name','EMG activity');
% ax_emg  = axes(emg_fig);
% emg_bar = bar(ax_emg, 1:n_emg, emg_std_buf, 'FaceColor','flat');
% title(ax_emg, 'EMG channel activity (std)');
% xlabel(ax_emg, 'Channel');
% ylabel(ax_emg, 'Std (a.u.)');
% ylim(ax_emg, emg_ylim_std);
%% EMG 
emg_lines   = [];
emg_buf    = [];
emg_offset = 200;

if show_emg_traces
    [emg_fig2, emg_lines, emg_buf] = setup_emg_figure(n_emg, blockSamples*2, emg_offset);
end

%% MAIN LOOP
% =========================================================================
% MAIN LOOP
% =========================================================================
disp('Running. Press M=MVC, Q=quit.');

while ~strcmp(guidata(force_fig).pressed, 'q')

    while tcpSocket.NumBytesAvailable < bytesPerBlock, pause(0.001); end
    D = readBlock(tcpSocket, TotNumByte, blockSamples);

    % Force
    if strcmp(force_dir, 'push')
        fL = -(mean(double(D(force_left, :))) - offset_L) * force_scale;
        fR = -(mean(double(D(force_right,:))) - offset_R) * force_scale;
    else  % pull
        fL =  (mean(double(D(force_left, :))) - offset_L) * force_scale;
        fR =  (mean(double(D(force_right,:))) - offset_R) * force_scale;
    end
    %fS = -(mean(double(D(force_sum,  :))) - offset_S);
    fS = -(mean(double(D(force_left,:) + D(force_right,:))) - offset_S) * force_scale;

    if mvc_value > 0
        buf_L = [buf_L(2:end), fL/mvc_value];
        buf_R = [buf_R(2:end), fR/mvc_value]; 
        buf_S = [buf_S(2:end), fS/mvc_value];
    else
        buf_L = [buf_L(2:end), fL];
        buf_R = [buf_R(2:end), fR];
        buf_S = [buf_S(2:end), fS];
    end

    set(hl,'YData',buf_L);
    set(hr,'YData',buf_R);
    set(hs,'YData',buf_S);

    % EMG activity display
    % emg_block   = double(D(emg_channels,:));
    % emg_std_buf = std(emg_block, 0, 2);
    % set(emg_bar, 'YData', emg_std_buf);

    if show_emg_traces
        for k = 1:n_emg
            emg_buf(k,:) = [emg_buf(k, blockSamples+1:end), double(D(emg_channels(k),:))];
            set(emg_lines(k), 'YData', emg_buf(k,:) + (k-1)*emg_offset);
        end
    end

    % MVC
    if strcmp(guidata(force_fig).pressed, 'm')
        guidata(force_fig, setfield(guidata(force_fig),'pressed',''));

        if ~isempty(preComputedMVC)
            mvc_value = preComputedMVC;
            buf_L = zeros(1,N); buf_R = zeros(1,N); buf_S = zeros(1,N);
            set(hl,'YData',buf_L); set(hr,'YData',buf_R); set(hs,'YData',buf_S);
            ylabel(ax, 'Force (MVC fraction)');
            fprintf('MVC set to precomputed value: %.0f\n', mvc_value);

        else

            
          
            [mvc_value, mvc_value_L, mvc_value_R, mvc_emg, mvc_force_raw, mvc_force_L, mvc_force_R, buf_L, buf_R, buf_S, emg_buf] = run_MVC(tcpSocket, ax, hl, hr, hs, ...
                buf_L, buf_R, buf_S, TotNumByte, blockSamples, bytesPerBlock, ...
                force_left, force_right, force_sum, offset_L, offset_R, offset_S, ...
                force_dir, sampFreq, mvc_duration, emg_channels, n_emg, ConvFact,...
                emg_lines, emg_buf, emg_offset, show_emg_traces, force_scale);
        end

        flush(tcpSocket);


        fprintf('>>> MVC = %.0f <<<\n', mvc_value);

        % append to log

        mvc_csv = fullfile(datapath, 'mvc_log.csv');
        if ~isfile(mvc_csv)
            fid = fopen(mvc_csv, 'w');
            fprintf(fid, 'subject,datetime,MVC_Sum,MVC_L,MVC_R\n');
            fclose(fid);
        end
        fid = fopen(mvc_csv, 'a');
        fprintf(fid, '%s,%s,%.0f,%.0f,%.0f\n', subject, datestr(now,'yyyymmdd_HHMMSS'), mvc_value, mvc_value_L, mvc_value_R);
        fclose(fid);

        % fid = fopen(fullfile(datapath, 'mvc_log.txt'), 'a');
        % fprintf(fid, '%s  MVC_Sum = %.0f  MVC_L = %.0f  MVC_R = %.0f\n', datestr(now,'yyyymmdd_HHMMSS'), mvc_value, mvc_value_L, mvc_value_R);
        % fclose(fid);

        if mvc_value > 0
            buf_L = zeros(1,N);
            buf_R = zeros(1,N);
            buf_S = zeros(1,N);
            set(hl,'YData',buf_L);
            set(hr,'YData',buf_R);
            set(hs,'YData',buf_S);
            ylabel(ax, 'Force (MVC fraction)');
            title(ax, 'Force — M=MVC  T=task  O=offset  Q=quit');
            save_mvc(datapath, mvc_emg, mvc_force_raw, mvc_force_L, mvc_force_R, mvc_value, sampFreq, n_emg, emg_channels,subject);

            % % saving
            % SamplingFrequency = sampFreq;
            % Force = mvc_force_raw;
            % save(fullfile(datapath, sprintf('mvc_%s.mat', datestr(now,'yyyymmdd_HHMMSS'))), ...
            %     'mvc_value', 'mvc_emg', 'Force', 'SamplingFrequency', 'emg_channels');
            % 
            % 
            % disp('MVC saved.');
        end
    end


    % TASK
    if strcmp(guidata(force_fig).pressed, 't')
        guidata(force_fig, setfield(guidata(force_fig),'pressed',''));
        if mvc_value == 0
            disp('Run MVC first (press M).');
        else
            [task_force, task_emg, emg_buf] = run_task(tcpSocket, ax, hl, hr, hs, ...
                buf_L, buf_R, buf_S, TotNumByte, blockSamples, bytesPerBlock, ...
                force_left, force_right, force_sum, offset_L, offset_R, offset_S, ...
                force_dir, sampFreq, mvc_value, task_leg, task_shape, task_level, mcon_cycles, ...
                trap_ramp_s, trap_hold_s, lead_in_s, emg_channels, n_emg, ConvFact,...
                emg_lines, emg_buf, emg_offset, show_emg_traces, force_scale);


            flush(tcpSocket);

            if ~isempty(task_force)
                save_task(datapath, task_emg, task_force, mvc_value, task_leg, task_shape, sampFreq, n_emg, subject);

                % % saving
                % SamplingFrequency = sampFreq;
                % Force             = task_force;
                % Target            = task_level;
                % save(fullfile(datapath, sprintf('task_%s.mat', datestr(now,'yyyymmdd_HHMMSS'))), ...
                %     'Force', 'task_emg', 'SamplingFrequency', 'emg_channels', ...
                %     'mvc_value', 'task_leg', 'task_shape', 'Target');
                % disp('Task saved.');


            end
        end
    end

    % offset!
    if strcmp(guidata(force_fig).pressed, 'o')
        guidata(force_fig, setfield(guidata(force_fig),'pressed',''));
        disp('Recalibrating — release force...');
        title(ax, 'Recalibrating — release force...');
        drawnow;

        baseline_buf = zeros(3, baseline_samples);
        col = 1;
        while col <= baseline_samples
            while tcpSocket.BytesAvailable < bytesPerBlock, pause(0.001); end
            D   = readBlock(tcpSocket, TotNumByte, blockSamples);
            len = min(blockSamples, baseline_samples - col + 1);
            baseline_buf(1, col:col+len-1) = D(force_left,  1:len);
            baseline_buf(2, col:col+len-1) = D(force_right, 1:len);
            %baseline_buf(3, col:col+len-1) = D(force_sum,   1:len);
            baseline_buf(3, col:col+len-1) = D(force_left, 1:len) + D(force_right, 1:len);
            col = col + len;
        end
        offset_L = mean(baseline_buf(1,:));
        offset_R = mean(baseline_buf(2,:));
        offset_S = mean(baseline_buf(3,:));
        fprintf('New offsets — L:%.0f  R:%.0f  Sum:%.0f\n', offset_L, offset_R, offset_S);
        title(ax, 'Force — M=MVC  T=task  O=offset  Q=quit');
    end


    drawnow limitrate;

end

%%
% =========================================================================
% STOP
% =========================================================================
write(tcpSocket, [0; CRC8(0,1)], 'uint8');
clear tcpSocket;
close all;
disp('Done.');


%%
%%% ---------------------------------------------
%%% FUNCTIONS
%%% ---------------------------------------------

%% readBlock
function data = readBlock(t, TotNumByte, blockSamples)
Temp = fread(t, [TotNumByte, blockSamples], 'uint8');
Temp = reshape(Temp, TotNumByte, blockSamples);
D    = Temp(1:2:end,:)*256 + Temp(2:2:end,:);
idx  = D >= 32768;
D(idx) = D(idx) - 65536;
data = D;
end

%% run_MVC
function [mvc_value, mvc_value_L, mvc_value_R, mvc_emg, mvc_force_raw,  mvc_force_L, mvc_force_R, buf_L, buf_R, buf_S, emg_buf] = run_MVC(tcpSocket, ax, hl, hr, hs, ...
    buf_L, buf_R, buf_S, TotNumByte, blockSamples, bytesPerBlock, ...
    force_left, force_right, force_sum, offset_L, offset_R, offset_S, ...
    force_dir, sampFreq, mvc_duration, emg_channels, n_emg, ConvFact,...
    emg_lines, emg_buf, emg_offset, show_emg_traces, force_scale)

for ct = 3:-1:1
    title(ax, sprintf('GET READY... %d', ct));
    drawnow; pause(1);
end
title(ax, '*** PUSH NOW ***'); drawnow;
set(ax, 'Color', [1 0.85 0.85]);
drawnow;

flush(tcpSocket);

mvc_n         = mvc_duration * sampFreq;
mvc_emg       = zeros(n_emg, mvc_n);
mvc_force_raw = zeros(1, mvc_n);

% for bilat, save out contributions
mvc_force_L = zeros(1, mvc_n);
mvc_force_R = zeros(1, mvc_n);

col           = 1;
t_start       = tic;

while col <= mvc_n
    elapsed   = toc(t_start);
    remaining = max(0, mvc_duration - elapsed);
    title(ax, sprintf('PUSH! %d', ceil(remaining)));

    while tcpSocket.BytesAvailable < bytesPerBlock, pause(0.001); end
    D = readBlock(tcpSocket, TotNumByte, blockSamples);

    if strcmp(force_dir,'push')
        fL = -(mean(double(D(force_left, :))) - offset_L) * force_scale;
        fR = -(mean(double(D(force_right,:))) - offset_R) * force_scale;
    else
        fL =  (mean(double(D(force_left, :))) - offset_L) * force_scale;
        fR =  (mean(double(D(force_right,:))) - offset_R) * force_scale;
    end
    %fS = -(mean(double(D(force_sum,:))) - offset_S);
    fS = -(mean(double(D(force_left,:) + D(force_right,:))) - offset_S) * force_scale;

    buf_L = [buf_L(2:end), fL];
    buf_R = [buf_R(2:end), fR];
    buf_S = [buf_S(2:end), fS];
    set(hl,'YData',buf_L);
    set(hr,'YData',buf_R);
    set(hs,'YData',buf_S);

    idx_end = min(col+blockSamples-1, mvc_n);
    len     = idx_end - col + 1;

    % if strcmp(force_dir,'push')
    %     mvc_force_raw(col:idx_end) = -(double(D(force_sum, 1:len)) - offset_S);
    % else
    %     mvc_force_raw(col:idx_end) =  (double(D(force_sum, 1:len)) - offset_S);
    % end

    if strcmp(force_dir,'push')
        mvc_force_raw(col:idx_end) = -(double(D(force_left,1:len) + D(force_right,1:len)) - offset_S) * force_scale;
        mvc_force_L(col:idx_end) = -(double(D(force_left, 1:len))  - offset_L) * force_scale;   % 
        mvc_force_R(col:idx_end) = -(double(D(force_right,1:len))  - offset_R * force_scale);
    else
        mvc_force_raw(col:idx_end) =  (double(D(force_left,1:len) + D(force_right,1:len)) - offset_S) * force_scale;
        mvc_force_L(col:idx_end) = (double(D(force_left, 1:len))  - offset_L) * force_scale;   %  positive for pull
        mvc_force_R(col:idx_end) = (double(D(force_right,1:len))  - offset_R) * force_scale;
    end

    mvc_emg(:, col:idx_end) = double(D(emg_channels, 1:len)) * ConvFact;

    col = col + len;

    if show_emg_traces
        for k = 1:n_emg
            emg_buf(k,:) = [emg_buf(k, blockSamples+1:end), double(D(emg_channels(k),:))];
            set(emg_lines(k), 'YData', emg_buf(k,:) + (k-1)*emg_offset);
        end
    end

    drawnow limitrate;
end

title(ax, 'MVC complete');
set(ax, 'Color', 'w');
mvc_value_L = max(mvc_force_L);
mvc_value_R = max(mvc_force_R);
mvc_value = max(mvc_force_raw);
fprintf('MVC — L: %.0f  R: %.0f  Sum: %.0f\n', mvc_value_L, mvc_value_R, mvc_value);

mf = figure;
plot(mvc_force_raw, 'k', 'LineWidth', 1.5); hold on;
plot(mvc_force_L, 'r', 'LineWidth', 1.5);
plot(mvc_force_R, 'b', 'LineWidth', 1.5);

yline(mvc_value, 'k--', sprintf('Peak: %.0f', mvc_value));
yline(mvc_value_L, 'r--', sprintf('Peak: %.0f', mvc_value_L));
yline(mvc_value_R, 'b--', sprintf('Peak: %.0f', mvc_value_R));

title(sprintf('Peaks: %.0f %.0f %.0f', mvc_value, mvc_value_L, mvc_value_R));
xlabel('Samples'); ylabel('Force (ADC)');

%%% REMOVE DIALOGUE BOX FOR NOW
% choice = questdlg(sprintf('MVC = %.0f — accept?', mvc_value), ...
%     'MVC', 'Accept', 'Discard', 'Accept');
% close(mf);
% 
% if isempty(choice) || strcmp(choice, 'Discard')
%     mvc_value = 0;
%     disp('MVC discarded.');
%     title(ax,'Force — M=MVC  T=task  O=offset  Q=quit');
% else
%     fprintf('MVC accepted: %.0f\n', mvc_value);
%     title(ax,'Force — M=MVC  T=task  O=offset  Q=quit');
% end

end

%% setup_emg_figure
function [emg_fig2, emg_lines, emg_buf] = setup_emg_figure(n_emg, N, offset)
emg_buf   = zeros(n_emg, N);
emg_fig2 = figure('Color','w','Name','EMG traces','WindowStyle','normal');
ax2       = axes(emg_fig2);
hold(ax2,'on');
emg_lines = gobjects(n_emg,1);
for k = 1:n_emg
    emg_lines(k) = plot(ax2, 1:N, emg_buf(k,:) + (k-1)*offset, 'Color',[0.3 0.3 0.3],'LineWidth',0.5);
end
xlim(ax2,[1 N]);
title(ax2,'Live EMG traces');
xlabel(ax2,'Samples'); ylabel(ax2,'Channel');
ax2.YLimMode = 'auto';
end

%% run_task
function [task_force, task_emg, emg_buf] = run_task(tcpSocket, ax, hl, hr, hs, ...
    buf_L, buf_R, buf_S, TotNumByte, blockSamples, bytesPerBlock, ...
    force_left, force_right, force_sum, offset_L, offset_R, offset_S, ...
    force_dir, sampFreq, mvc_value, task_leg, task_shape, task_level, mcon_cycles,...
    trap_ramp_s, trap_hold_s, lead_in_s, emg_channels, n_emg, ConvFact,...
    emg_lines, emg_buf, emg_offset, show_emg_traces, force_scale)

updates_per_sec = sampFreq / blockSamples;
ramp_steps  = round(trap_ramp_s * updates_per_sec);
hold_steps  = round(trap_hold_s * updates_per_sec);
lead_steps  = round(lead_in_s   * updates_per_sec);

switch task_shape
    case 'trap'
        target_trace = [zeros(1,lead_steps), ...
            linspace(0,task_level,ramp_steps), ...
            task_level*ones(1,hold_steps), ...
            linspace(task_level,0,ramp_steps), ...
            zeros(1,lead_steps)];
    case 'sombrero'
        brim_level = task_level * 0.4;
        brim_steps = round(1.5 * updates_per_sec);
        dip_steps  = round(1.0 * updates_per_sec);
        target_trace = [zeros(1,lead_steps), ...
            linspace(0,brim_level,ramp_steps), ...
            brim_level*ones(1,brim_steps), ...
            linspace(brim_level,task_level,dip_steps), ...
            task_level*ones(1,hold_steps), ...
            linspace(task_level,brim_level,dip_steps), ...
            brim_level*ones(1,brim_steps), ...
            linspace(brim_level,0,ramp_steps), ...
            zeros(1,lead_steps)];
    case 'mcon'
        % sinusoid fitted into hold period
        % shift by pi/2 so sine starts at peak (zero slope) and ends at trough going up...
        % use -(2n-1) half cycles from pi/2:
        t_hold    = linspace(pi/2, pi/2 + (2*mcon_cycles)*pi, hold_steps);
        sine_wave = task_level + (task_level * 0.3 * sin(t_hold));
        % sine_wave(1) = task_level + 0.3*task_level*sin(pi/2) = task_level*1.3
        % so ramp goes to task_level*1.3
        target_trace = [zeros(1,lead_steps), ...
            linspace(0, sine_wave(1), ramp_steps), ...
            sine_wave, ...
            linspace(sine_wave(end), 0, ramp_steps), ...
            zeros(1,lead_steps)];


        


        

end

n_target   = numel(target_trace);
n_samples  = n_target * blockSamples;

task_force = zeros(7, n_samples);
% rows 1-3: raw ADC offset corrected
% rows 4-6: normalised (MVC fraction)
% row 7:    target


task_emg   = zeros(n_emg, n_samples);

% Countdown
for ct = 3:-1:1
    title(ax, sprintf('Starting in %d...', ct));
    drawnow; pause(1);
end

title(ax, '*** FOLLOW THE TRACE ***'); drawnow;

flush(tcpSocket);

% fix axis
ylim(ax, [-task_level*0.5, task_level*2]);
ax.YLimMode = 'manual';

% Tracker ball
N = numel(buf_L);
lookahead  = round(0.3 * N);
cursor_pos = N - lookahead;

% show target
target_line = plot(ax, 1:N, NaN(1,N), 'g', 'LineWidth', 5);

tracker = plot(ax, cursor_pos, 0, 'ko', 'MarkerSize',16, 'MarkerFaceColor','k');
switch task_leg
    case 'left',      set(tracker,'MarkerFaceColor','r','MarkerEdgeColor','r');
    case 'right',     set(tracker,'MarkerFaceColor','b','MarkerEdgeColor','b');
    case 'bilateral', set(tracker,'MarkerFaceColor','k','MarkerEdgeColor','k');
end

% switch off force lines
set(hl, 'Visible', 'off');
set(hr, 'Visible', 'off');
set(hs, 'Visible', 'off');

% combat lag
emg_update_counter = 0;
emg_update_every   = 5; % round(sampFreq / blockSamples * 3);


% i want a line history
trail_len = round(N * 1);  % show last 30% of window as trail = 0.3
trail_buf = NaN(1, trail_len);
trail_line = plot(ax, (cursor_pos - trail_len + 1):cursor_pos, trail_buf, ...
    'LineWidth', 2);
% match colour to leg
switch task_leg
    case 'left',      set(trail_line, 'Color', 'r');
    case 'right',     set(trail_line, 'Color', 'b');
    case 'bilateral', set(trail_line, 'Color', 'k');
end

col = 1;
for k = 1:n_target

    %t_loop = tic;
    if tcpSocket.BytesAvailable > bytesPerBlock * 3
        flush(tcpSocket);
    end



    while tcpSocket.BytesAvailable < bytesPerBlock, pause(0.001); end
    D = readBlock(tcpSocket, TotNumByte, blockSamples);

    if strcmp(force_dir,'push')
        fL = -(mean(double(D(force_left, :))) - offset_L) * force_scale;
        fR = -(mean(double(D(force_right,:))) - offset_R) * force_scale;
    else
        fL =  (mean(double(D(force_left, :))) - offset_L) * force_scale;
        fR =  (mean(double(D(force_right,:))) - offset_R) * force_scale;
    end
    %fS = -(mean(double(D(force_sum,:))) - offset_S);
    fS = -(mean(double(D(force_left,:) + D(force_right,:))) - offset_S) * force_scale;

    % normalise
    dL = fL/mvc_value;
    dR = fR/mvc_value;
    dS = fS/mvc_value;
    switch task_leg
        case 'left',      disp_track = dL;
        case 'right',     disp_track = dR;
        case 'bilateral', disp_track = dS;
    end

    % % scrolling force plot
    % % comment these lines out for speed
    % buf_L = [buf_L(2:end), dL];
    % buf_R = [buf_R(2:end), dR];
    % buf_S = [buf_S(2:end), dS];
    % set(hl,'YData',buf_L);
    % set(hr,'YData',buf_R);
    % set(hs,'YData',buf_S);

    % scrolling target
    % target_buf = NaN(1,N);
    % for x = 1:N
    %     t_idx = k + (x - cursor_pos);
    %     if t_idx >= 1 && t_idx <= n_target
    %         target_buf(x) = target_trace(t_idx);
    %     end
    % end
    
    % this is vectorised scrolling target:
    x_idx      = 1:N;
    t_idx      = k + (x_idx - cursor_pos);
    valid      = t_idx >= 1 & t_idx <= n_target;
    target_buf = NaN(1,N);
    target_buf(valid) = target_trace(t_idx(valid));


    % history line
    trail_buf = [trail_buf(2:end), disp_track];
    set(trail_line, 'XData', (cursor_pos - trail_len + 1):cursor_pos, ...
        'YData', trail_buf);


    set(target_line,'YData',target_buf);
    set(tracker,'XData',cursor_pos,'YData',disp_track);

    % store
    idx_end = min(col+blockSamples-1, n_samples);
    len     = idx_end - col + 1;
    if strcmp(force_dir,'push')
        task_force(1, col:idx_end) = -(double(D(force_left, 1:len))  - offset_L) * force_scale;
        task_force(2, col:idx_end) = -(double(D(force_right,1:len))  - offset_R) * force_scale;
        %task_force(3, col:idx_end) = -(double(D(force_sum,  1:len))  - offset_S);
        task_force(3, col:idx_end) = -(double(D(force_left,1:len) + D(force_right,1:len)) - offset_S) * force_scale;
    else
        task_force(1, col:idx_end) =  (double(D(force_left, 1:len))  - offset_L) * force_scale;
        task_force(2, col:idx_end) =  (double(D(force_right,1:len))  - offset_R) * force_scale;
        %task_force(3, col:idx_end) = -(double(D(force_sum,  1:len))  - offset_S);
        task_force(3, col:idx_end) = -(double(D(force_left,1:len) + D(force_right,1:len)) - offset_S) * force_scale;
    end
    task_force(4, col:idx_end) = task_force(1, col:idx_end) / mvc_value;
    task_force(5, col:idx_end) = task_force(2, col:idx_end) / mvc_value;
    task_force(6, col:idx_end) = task_force(3, col:idx_end) / mvc_value;


    if k < n_target
        task_force(7, col:idx_end) = linspace(target_trace(k), target_trace(k+1), len);
    else
        task_force(7, col:idx_end) = target_trace(k);
    end


    %task_force(7, col:idx_end) = target_trace(k);

    %task_force(4, col:idx_end) = target_trace(k);   % target value for this update
    task_emg(:, col:idx_end)   = double(D(emg_channels,1:len)) * ConvFact;
    col = col + len;


    % if show_emg_traces
    %     for sh = 1:n_emg
    %         emg_buf(sh,:) = [emg_buf(sh, blockSamples+1:end), double(D(emg_channels(sh),:))];
    %         set(emg_lines(sh), 'YData', emg_buf(sh,:) + (sh-1)*emg_offset);
    %     end
    % end

    emg_update_counter = emg_update_counter + 1;
    if show_emg_traces && mod(emg_update_counter, emg_update_every) == 0
        for sh = 1:n_emg
            emg_buf(sh,:) = [emg_buf(sh, blockSamples+1:end), double(D(emg_channels(sh),:))];
            set(emg_lines(sh), 'YData', emg_buf(sh,:) + (sh-1)*emg_offset);
        end
    end

    drawnow limitrate;
    % fprintf('loop: %.1f ms\n', toc(t_loop)*1000);

    % % drain backlog accumulated during drawnow?
    % while tcpSocket.BytesAvailable >= bytesPerBlock * 2
    %     fread(tcpSocket, [TotNumByte, blockSamples], 'uint8');
    %     col = col + blockSamples;
    %     k   = k + 1;
    %     if k > n_target, break; end
    % end

end

delete(tracker);
delete(target_line);
delete(trail_line);

ax.YLimMode = 'auto'; % reset axis
set(hl, 'Visible', 'on');
set(hr, 'Visible', 'on');
set(hs, 'Visible', 'on');

title(ax, 'Task complete.');
task_force = task_force(:, 1:col-1);
task_emg   = task_emg(:,   1:col-1);
disp('Task complete.');

end

%% save_task
function save_task(datapath, task_emg, task_force, mvc_value, task_leg, task_shape, sampFreq, n_emg, subject)

signal.data          = task_emg;
signal.fsamp         = sampFreq;
signal.nChan         = n_emg;
signal.ngrid         = 2;
signal.gridname      = {'Muovi+', 'Muovi+'};
signal.muscle        = {'Muscle1', 'Muscle2'};

signal.auxiliary     = [task_force(1,:); ...
                         task_force(2,:); ...
                         task_force(3,:); ...
                         task_force(4,:); ...
                         task_force(5,:); ...
                         task_force(6,:); ...
                         task_force(7,:)];
signal.auxiliaryname = {'Force_L_raw', 'Force_R_raw', 'Force_Sum_raw', ...
                         'Force_L_norm', 'Force_R_norm', 'Force_Sum_norm', ...
                         'Target'};
signal.target        = task_force(7,:);

% for forceGUI
Force  = task_force;
Target = task_force(7,:);

save(fullfile(datapath, sprintf('task_%s_%s.mat', subject, datestr(now,'yyyymmdd_HHMMSS'))), ...
    'signal', 'mvc_value', 'task_leg', 'task_shape', 'Force', 'Target', '-v7.3');
disp('Task saved.');
end

%% save_mvc
function save_mvc(datapath, mvc_emg, mvc_force_raw, mvc_force_L, mvc_force_R, mvc_value, sampFreq, n_emg, emg_channels, subject)

signal_mvc.data          = mvc_emg;
signal_mvc.fsamp         = sampFreq;
signal_mvc.nChan         = n_emg;
signal_mvc.ngrid         = 2;
signal_mvc.gridname      = {'Muovi+', 'Muovi+'};
signal_mvc.muscle        = {'Muscle1', 'Muscle2'};
%signal_mvc.auxiliary     = mvc_force_raw;

signal_mvc.auxiliary     = [mvc_force_raw; mvc_force_L; mvc_force_R];
signal_mvc.auxiliaryname = {'Force_Sum_raw', 'Force_L_raw', 'Force_R_raw'};

%signal_mvc.auxiliaryname = {'Force_Sum_raw'};
signal_mvc.target        = [];

% for forceGUI
Force = mvc_force_raw;

save(fullfile(datapath, sprintf('mvc_%s_%s.mat', subject, datestr(now,'yyyymmdd_HHMMSS'))), ...
    'signal_mvc', 'mvc_value', 'emg_channels', 'Force','-v7.3');
    
disp('MVC saved.');
end