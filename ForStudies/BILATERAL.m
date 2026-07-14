%% experiment_muovi6.m
% Live force plot + EMG recording via SyncStation
% uint8 read

close all; clear all; clc;

subject = 'sub01';   % set per participant
force_dir = 'pull'; % set this to push or pull
study    = 'STUDY1';
muscle   = 'TA';        % e.g. VL, TA, GM
condition = 'testing'; % Could be PRE/POST/etc.
% saving order: STUDY subject muscle task_leg task_shape task_level CONDITION

% in volts
%preComputedMVC = 1.2;   % set to a value e.g. 3 to skip MVC, [] to require MVC

mvcLeft      = [];   % V — set per participant
mvcRight     = [];   % V — set per participant


mvc_duration = 3;

task_shape  = 'trap';   % 'trap' | 'sombrero' | 'mcon' | 'multi_trap' | 'multi_target'
task_level  = 0.1;          % target as fraction of MVC (ignored for multi_target)
task_leg    = 'bilateral';  % 'left' | 'right' | 'bilateral' (ignored for multi_target)
trap_ramp_s = 5;
trap_hold_s = 30;
lead_in_s   = 3;
multi_trap_rest_s = 2;   % rest between traps (multi_trap only)

mcon_cycles = 8;

left_multi_target = 0.5;
right_multi_target = 0.25;

% -------------------------------------------------------------------------
% DISPLAY COLOURS (change here — applies everywhere)
% -------------------------------------------------------------------------
colours.bg          = 'k';                      % figure/axes background - default black
colours.grid        = [0.3 0.3 0.3];            % grid lines
%colours.target      = [0.2 0.9 0.2];           % target trace - green 1 (neon)
%colours.target      = [0.33 0.64 0.30];        % target trace - green 2 (better)
colours.target      = [0 0.62 0.451];           % target trace - green 3 (CBsafe)
%colours.target      = [0.5 0.5 0.5];           % target trace - grey
colours.cursor      = 'w';                      % vertical time cursor
colours.text        = 'w';                      % axis labels, titles, ticks
%colours.left        = [1 0.16 0.16];           % left leg force line + ball - red 1 
colours.left        = [0.835 0.369 0];          % left leg force line + ball - orange (CBsafe)
%colours.right       = [0.76 0.56 0.92];        % right leg force line + ball - purple 1
colours.right       = [0.8667 0.1294 0.4902];   % right leg force line + ball - pink (CBsafe)
colours.bilateral   = 'w';                      % bilateral force line + ball
colours.waitingRoom = 'w';                      % background of force trace before task
colours.mvc         = [0.9882 0.5725 0.4471];   % MVC push/pull background

colours.ballSize = 16;

% -------------------------------------------------------------------------
% MULTI-TARGET CONFIG (only used when task_shape = 'multi_target')
% Each entry defines one leg's target. Both must have the same total duration.
% Supported shapes per leg: 'trap' | 'sombrero' | 'mcon'
% -------------------------------------------------------------------------
multi_target_cfg(1) = struct('leg','left',  'shape','trap', 'level',left_multi_target);
multi_target_cfg(2) = struct('leg','right', 'shape','trap', 'level',right_multi_target);

% Auto-select based on task_leg
if ~strcmp(task_shape, 'multi_target')
    switch task_leg
        case 'left',      preComputedMVC = mvcLeft;
        case 'right',     preComputedMVC = mvcRight;
        case 'bilateral', preComputedMVC = mvcLeft + mvcRight;
    end
else
    preComputedMVC = mvcLeft;  % placeholder; run_multi_target uses mvcLeft/mvcRight directly
end

% fudge for saving
if trap_hold_s == 0
    task_shape_save = 'ramp';
else
    task_shape_save = task_shape;
end

% Force channels are 16-bit signed integers (-32768 to +32767 counts)
% Hardware input range is ±2.5V (5V total span)
% Scale factor converts raw counts back to volts: 5V / 2^16 counts
% hardware measures a voltage (say -2.5V to +2.5V), 
% converts it to a 16-bit signed integer (-32768 to +32767), 
% and sends it over TCP as two bytes. 
% readBlock reassembles those two bytes back into the integer, and 
% force_scale converts that integer back to volts.
force_scale = 10.0 / 65536;


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
%datapath = 'D:\OneDrive - The University of Nottingham\Mathew Piasecki (staff)''s files - ePhys Lab\Michael\';

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
fprintf('Offsets — L:%.3f  R:%.3f  Sum:%.3f\n', offset_L, offset_R, offset_S);

% =========================================================================
% FIGURES
% =========================================================================
%% FORCE FIGURE
N = round(10 * sampFreq / blockSamples);   % 10s rolling window
buf_L = zeros(1,N);
buf_R = zeros(1,N);
buf_S = zeros(1,N);

force_fig = figure('Color',colours.waitingRoom,'Name','Force');
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
    fprintf('Using precomputed MVC = %.3f\n', mvc_value);
end



%% MAIN LOOP
% =========================================================================
% MAIN LOOP
% =========================================================================
% Clear any data buffered during setup/figure creation
flush(tcpSocket);
disp('Running. Press M=MVC, Q=quit.');


while ~strcmp(guidata(force_fig).pressed, 'q')

    while tcpSocket.NumBytesAvailable < bytesPerBlock, pause(0.001); end
    D = readBlock(tcpSocket, TotNumByte, blockSamples);

    % Force
    if strcmp(force_dir, 'push')
        fL = -(mean(double(D(force_left, :))) - offset_L) * force_scale;
        fR = -(mean(double(D(force_right,:))) - offset_R) * force_scale;
        fS = -(mean(double(D(force_left,:) + D(force_right,:))) - offset_S) * force_scale;
    else  % pull
        fL =  (mean(double(D(force_left, :))) - offset_L) * force_scale;
        fR =  (mean(double(D(force_right,:))) - offset_R) * force_scale;
        fS =  (mean(double(D(force_left,:) + D(force_right,:))) - offset_S) * force_scale;
    end

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



    %% MVC
    if strcmp(guidata(force_fig).pressed, 'm')
        guidata(force_fig, setfield(guidata(force_fig),'pressed',''));

        if ~isempty(preComputedMVC)
            mvc_value = preComputedMVC;
            buf_L = zeros(1,N); buf_R = zeros(1,N); buf_S = zeros(1,N);
            set(hl,'YData',buf_L); set(hr,'YData',buf_R); set(hs,'YData',buf_S);
            ylabel(ax, 'Force (MVC fraction)');
            fprintf('MVC set to precomputed value: %.3f\n', mvc_value);



        else



            [mvc_value, mvc_value_L, mvc_value_R, mvc_emg, mvc_force_raw, mvc_force_L, mvc_force_R, buf_L, buf_R, buf_S] = run_MVC(tcpSocket, ax, hl, hr, hs, ...
                buf_L, buf_R, buf_S, TotNumByte, blockSamples, bytesPerBlock, ...
                force_left, force_right, force_sum, offset_L, offset_R, offset_S, ...
                force_dir, sampFreq, mvc_duration, emg_channels, n_emg, ConvFact,...
                force_scale, colours, task_leg);


            flush(tcpSocket);


            %fprintf('>>> MVC = %.3f <<<\n', mvc_value);

            switch task_leg
                case 'left',      mvc_value = mvc_value_L;
                case 'right',     mvc_value = mvc_value_R;
                case 'bilateral', mvc_value = mvc_value;   % sum, already correct
            end

            fprintf('>>> MVC = %.3f <<<\n', mvc_value);


            % append to log

            mvc_csv = fullfile(datapath, 'mvc_log.csv');
            if ~isfile(mvc_csv)
                fid = fopen(mvc_csv, 'w');
                fprintf(fid, 'subject,datetime,MVC_Sum,MVC_L,MVC_R\n');
                fclose(fid);
            end
            fid = fopen(mvc_csv, 'a');
            fprintf(fid, '%s,%s,%.5f,%.5f,%.5f\n', subject, datestr(now,'yyyymmdd_HHMMSS'), mvc_value, mvc_value_L, mvc_value_R);
            fclose(fid);

  
            if mvc_value > 0
                buf_L = zeros(1,N);
                buf_R = zeros(1,N);
                buf_S = zeros(1,N);
                set(hl,'YData',buf_L);
                set(hr,'YData',buf_R);
                set(hs,'YData',buf_S);
                ylabel(ax, 'Force (MVC fraction)');
                title(ax, 'Force — M=MVC  T=task  O=offset  Q=quit');
                save_mvc(datapath, mvc_emg, mvc_force_raw, mvc_force_L, mvc_force_R, mvc_value, sampFreq, n_emg, emg_channels,subject, force_dir,task_leg,study, muscle, condition);

    
            end
        end
    end


    %% TASK
    if strcmp(guidata(force_fig).pressed, 't')
        guidata(force_fig, setfield(guidata(force_fig),'pressed',''));
        if mvc_value == 0
            disp('Run MVC first (press M).');
        else
            if strcmp(task_shape, 'multi_target')
                [task_force, task_emg] = run_multi_target(tcpSocket, ax, hl, hr, hs, ...
                    buf_L, buf_R, buf_S, TotNumByte, blockSamples, bytesPerBlock, ...
                    force_left, force_right, offset_L, offset_R, offset_S, ...
                    force_dir, sampFreq, mvcLeft, mvcRight, multi_target_cfg, mcon_cycles, ...
                    trap_ramp_s, trap_hold_s, lead_in_s, emg_channels, n_emg, ConvFact, ...
                    force_scale, colours);
            else
                [task_force, task_emg, task_extra] = run_task(tcpSocket, ax, hl, hr, hs, ...
                    buf_L, buf_R, buf_S, TotNumByte, blockSamples, bytesPerBlock, ...
                    force_left, force_right, force_sum, offset_L, offset_R, offset_S, ...
                    force_dir, sampFreq, mvc_value, task_leg, task_shape, task_level, mcon_cycles, ...
                    trap_ramp_s, trap_hold_s, lead_in_s, emg_channels, n_emg, ConvFact,...
                    force_scale, multi_trap_rest_s, colours);
            end


            flush(tcpSocket);

            if ~isempty(task_force)
                % for multi_target, use 'bilateral' leg label and max level for filename
                if strcmp(task_shape, 'multi_target')
                    save_leg   = 'bilateral';
                    save_level = max([multi_target_cfg.level]);
                else
                    save_leg   = task_leg;
                    save_level = task_level;
                end
                %save_task(datapath, task_emg, task_force, mvc_value, sampFreq, n_emg, subject, force_dir, task_shape, save_level, save_leg, study, muscle, condition);
                save_task(datapath, task_emg, task_force, task_extra, mvc_value, sampFreq, n_emg, subject, force_dir, task_shape_save, save_level, save_leg, study, muscle, condition);



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
        fprintf('New offsets — L:%.3f  R:%.3f  Sum:%.3f\n', offset_L, offset_R, offset_S);
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
function [mvc_value, mvc_value_L, mvc_value_R, mvc_emg, mvc_force_raw,  mvc_force_L, mvc_force_R, buf_L, buf_R, buf_S] = run_MVC(tcpSocket, ax, hl, hr, hs, ...
    buf_L, buf_R, buf_S, TotNumByte, blockSamples, bytesPerBlock, ...
    force_left, force_right, force_sum, offset_L, offset_R, offset_S, ...
    force_dir, sampFreq, mvc_duration, emg_channels, n_emg, ConvFact,...
    force_scale, colours, task_leg)

for ct = 3:-1:1
    title(ax, sprintf('GET READY... %d', ct));
    drawnow; pause(1);
end
title(ax, '*** PUSH NOW ***'); drawnow;
set(ax, 'Color', colours.mvc);
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
        fS = -(mean(double(D(force_left,:) + D(force_right,:))) - offset_S) * force_scale;
    else
        fL =  (mean(double(D(force_left, :))) - offset_L) * force_scale;
        fR =  (mean(double(D(force_right,:))) - offset_R) * force_scale;
        fS =  (mean(double(D(force_left,:) + D(force_right,:))) - offset_S) * force_scale;
    end


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
        mvc_force_R(col:idx_end) = -(double(D(force_right,1:len))  - offset_R) * force_scale;
    else
        mvc_force_raw(col:idx_end) =  (double(D(force_left,1:len) + D(force_right,1:len)) - offset_S) * force_scale;
        mvc_force_L(col:idx_end) = (double(D(force_left, 1:len))  - offset_L) * force_scale;   %  positive for pull
        mvc_force_R(col:idx_end) = (double(D(force_right,1:len))  - offset_R) * force_scale;
    end

    mvc_emg(:, col:idx_end) = double(D(emg_channels, 1:len)) * ConvFact;

    col = col + len;

    drawnow limitrate;
end

title(ax, 'MVC complete');
set(ax, 'Color', colours.waitingRoom);

fprintf('mvc_force_raw range: %.4f to %.4f\n', min(mvc_force_raw), max(mvc_force_raw));
fprintf('mvc_force_L range: %.4f to %.4f\n', min(mvc_force_L), max(mvc_force_L));
fprintf('mvc_force_R range: %.4f to %.4f\n', min(mvc_force_R), max(mvc_force_R));

% % old peak finding for L, R.
% mvc_value_L = max(mvc_force_L);
% mvc_value_R = max(mvc_force_R);
% mvc_value = max(mvc_force_raw);

% actually we want to choose not peak L and peak R for bilat condition,
% but when bilat peaks, what are L and R at that index. Subtle but
% important.
mvc_value = max(mvc_force_raw); % bilat peak
if strcmpi(task_leg, 'bilateral') % switch only if doing bilat conditon
    [~, mvc_idx] = max(mvc_force_raw);
    mvc_value_L  = mvc_force_L(mvc_idx);
    mvc_value_R  = mvc_force_R(mvc_idx);
else
    mvc_value_L = max(mvc_force_L);
    mvc_value_R = max(mvc_force_R);
end

mf = figure;
plot(mvc_force_raw, 'k', 'LineWidth', 1.5); hold on;
plot(mvc_force_L, 'r', 'LineWidth', 1.5);
plot(mvc_force_R, 'b', 'LineWidth', 1.5);

fprintf('MVC — L: %.3f  R: %.3f  Sum: %.3f\n', mvc_value_L, mvc_value_R, mvc_value);
% yline(mvc_value,  'k--', sprintf('Peak: %.3f', mvc_value));
% yline(mvc_value_L,'r--', sprintf('Peak: %.3f', mvc_value_L));
% yline(mvc_value_R,'b--', sprintf('Peak: %.3f', mvc_value_R));

yline(mvc_value,  'k--', sprintf('Peak: %.3f', mvc_value), ...
    'LabelVerticalAlignment','top', 'LabelHorizontalAlignment','left');
yline(mvc_value_L,'r--', sprintf('Peak: %.3f', mvc_value_L), ...
    'LabelVerticalAlignment','bottom', 'LabelHorizontalAlignment','center');
yline(mvc_value_R,'b--', sprintf('Peak: %.3f', mvc_value_R), ...
    'LabelVerticalAlignment','top', 'LabelHorizontalAlignment','right');

%title(sprintf('Peaks: %.0f %.0f %.0f', mvc_value, mvc_value_L, mvc_value_R));
title(sprintf('Peaks: %.3f %.3f %.3f', mvc_value, mvc_value_L, mvc_value_R));


xlabel('Samples'); ylabel('Force (ADC)');


end


%% run_task  (static-plot display — experiment_muovi6)
function [task_force, task_emg, task_extra] = run_task(tcpSocket, ax, hl, hr, hs, ...
    buf_L, buf_R, buf_S, TotNumByte, blockSamples, bytesPerBlock, ...
    force_left, force_right, force_sum, offset_L, offset_R, offset_S, ...
    force_dir, sampFreq, mvc_value, task_leg, task_shape, task_level, mcon_cycles,...
    trap_ramp_s, trap_hold_s, lead_in_s, emg_channels, n_emg, ConvFact,...
    force_scale, multi_trap_rest_s, colours)
% Static-plot task display: target drawn once, cursor + force trail update each block.
% Works for all single-leg shapes: trap | sombrero | mcon | multi_trap.

updates_per_sec = sampFreq / blockSamples;
ramp_steps  = round(trap_ramp_s * updates_per_sec);
hold_steps  = round(trap_hold_s * updates_per_sec);
lead_steps  = round(lead_in_s   * updates_per_sec);

% --- build target trace ---
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
        t_hold    = linspace(pi/2, pi/2 + (2*mcon_cycles)*pi, hold_steps);
        sine_wave = task_level + (task_level * 0.15 * sin(t_hold));
        target_trace = [zeros(1,lead_steps), ...
            linspace(0, sine_wave(1), ramp_steps), ...
            sine_wave, ...
            linspace(sine_wave(end), 0, ramp_steps), ...
            zeros(1,lead_steps)];

    case 'multi_trap'
        rest_steps  = round(multi_trap_rest_s * updates_per_sec);
        total_steps = round(30 * 60 * updates_per_sec);
        single_trap = [linspace(0,task_level,ramp_steps), ...
            task_level*ones(1,hold_steps), ...
            linspace(task_level,0,ramp_steps), ...
            zeros(1,rest_steps)];
        n_reps = ceil(total_steps / numel(single_trap));
        target_trace = [zeros(1,lead_steps), repmat(single_trap,1,n_reps)];
        target_trace = target_trace(1 : lead_steps + total_steps);

    otherwise
        error('Unknown task_shape: %s', task_shape);
end

do_record  = ~strcmp(task_shape, 'multi_trap');
n_steps    = numel(target_trace);
n_samples  = n_steps * blockSamples;
t_axis_upd = (0:n_steps-1) * (blockSamples / sampFreq);

% preallocate storage (7 rows: raw L/R/sum, norm L/R/sum, target)
task_force = zeros(7, n_samples);
task_emg   = zeros(n_emg, n_samples);
task_extra = zeros(12,n_samples); %  channels 135:146
% --- build static display figure ---
%leg_color = struct('left','r','right','b','bilateral','k');
%col_str   = leg_color.(task_leg);
col_str = colours.(task_leg);

task_fig = figure('Color',colours.bg,'Name',sprintf('Task — %s %s %.0f%%MVC', ...
    task_leg, task_shape, task_level*100), ...
    'Units','normalized','OuterPosition',[0 0 1 1]);
guidata(task_fig, struct('pressed',''));
set(task_fig,'KeyPressFcn',@(src,e) guidata(src,setfield(guidata(src),'pressed',e.Key)));

ax_t = axes(task_fig);
hold(ax_t,'on');
% set(ax_t,'Color','k','XColor','w','YColor','w', ...
%     'XGrid','on','YGrid','on','GridColor',[0.3 0.3 0.3]);

set(ax_t, 'Color',colours.bg, 'XColor',colours.text, 'YColor',colours.text, ...
    'XGrid','on','YGrid','on','GridColor',colours.grid);

% static green target
%plot(ax_t, t_axis_upd, target_trace, 'Color',[0.2 0.9 0.2], 'LineWidth', 3);
plot(ax_t, t_axis_upd, target_trace, 'Color',colours.target, 'LineWidth', 5);

% user force line (fills in during trial)
user_line   = plot(ax_t, t_axis_upd, NaN(1,n_steps), 'Color', col_str, 'LineWidth', 3);

ball = plot(ax_t, t_axis_upd(1), 0, 'o', 'MarkerSize', colours.ballSize, 'LineStyle', 'none');

%set(ball, 'MarkerFaceColor',colours.(task_leg), 'MarkerEdgeColor',colours.(task_leg));
set(ball, 'MarkerFaceColor','none', 'MarkerEdgeColor',colours.(task_leg), 'LineWidth', 2);

% vertical time cursor
cursor_line = xline(ax_t, 0, colours.cursor, 'LineWidth', 1.5);

% y-axis scaled to target
ylim(ax_t, [-task_level*0.2, task_level*1.6]);
xlim(ax_t, [t_axis_upd(1), t_axis_upd(end)]);
xlabel(ax_t, 'Time (s)', 'Color',colours.text);
ylabel(ax_t, 'Force (%MVC)', 'Color',colours.text);
yticks_vals = linspace(0, task_level, 5);
yticks(ax_t, yticks_vals);
yticklabels(ax_t, arrayfun(@(v) sprintf('%d%%',round(v*100)), yticks_vals,'UniformOutput',false));

% --- countdown ---
for ct = 3:-1:1
    title(ax_t, sprintf('%s — %s @ %d%% MVC — Starting in %d...', ...
        upper(task_leg(1:min(4,end))), upper(task_shape), round(task_level*100), ct), ...
        'Color',colours.text,'FontSize',13);
    drawnow; pause(1);
end
title(ax_t, sprintf('%s — %s @ %d%% MVC — FOLLOW THE LINE', ...
    upper(task_leg(1:min(4,end))), upper(task_shape), round(task_level*100)), ...
    'Color',colours.text,'FontSize',13);
drawnow;
flush(tcpSocket);

% preallocate history
user_force_hist = NaN(1, n_steps);


render_interval    = 0.075;
t_render = tic;
col = 1;

for k = 1:n_steps

    while tcpSocket.BytesAvailable < bytesPerBlock, pause(0.001); end
    D = readBlock(tcpSocket, TotNumByte, blockSamples);

    % read & normalise force
    if strcmp(force_dir,'push')
        fL = -(mean(double(D(force_left, :))) - offset_L) * force_scale;
        fR = -(mean(double(D(force_right,:))) - offset_R) * force_scale;
        fS = -(mean(double(D(force_left,:) + D(force_right,:))) - offset_S) * force_scale;
    else
        fL =  (mean(double(D(force_left, :))) - offset_L) * force_scale;
        fR =  (mean(double(D(force_right,:))) - offset_R) * force_scale;
        fS =  (mean(double(D(force_left,:) + D(force_right,:))) - offset_S) * force_scale;
    end
    dL = fL/mvc_value;  dR = fR/mvc_value;  dS = fS/mvc_value;
    switch task_leg
        case 'left',      disp_val = dL;
        case 'right',     disp_val = dR;
        case 'bilateral', disp_val = dS;
    end

    % update display
    user_force_hist(k) = disp_val;
    set(user_line,   'YData', user_force_hist);
    set(ball, 'XData', t_axis_upd(k), 'YData', disp_val);
    set(cursor_line, 'Value', t_axis_upd(k));

    % store
    if do_record
        idx_end = min(col+blockSamples-1, n_samples);
        len     = idx_end - col + 1;
        if strcmp(force_dir,'push')
            task_force(1,col:idx_end) = -(double(D(force_left, 1:len)) - offset_L) * force_scale;
            task_force(2,col:idx_end) = -(double(D(force_right,1:len)) - offset_R) * force_scale;
            task_force(3,col:idx_end) = -(double(D(force_left,1:len)+D(force_right,1:len)) - offset_S) * force_scale;
        else
            task_force(1,col:idx_end) =  (double(D(force_left, 1:len)) - offset_L) * force_scale;
            task_force(2,col:idx_end) =  (double(D(force_right,1:len)) - offset_R) * force_scale;
            task_force(3,col:idx_end) =  (double(D(force_left,1:len)+D(force_right,1:len)) - offset_S) * force_scale;
        end
        task_force(4,col:idx_end) = task_force(1,col:idx_end) / mvc_value;
        task_force(5,col:idx_end) = task_force(2,col:idx_end) / mvc_value;
        task_force(6,col:idx_end) = task_force(3,col:idx_end) / mvc_value;
        if k < n_steps
            task_force(7,col:idx_end) = linspace(target_trace(k), target_trace(k+1), len);
        else
            task_force(7,col:idx_end) = target_trace(k);
        end
        task_emg(:,col:idx_end) = double(D(emg_channels,1:len)) * ConvFact;
        task_extra(:,col:idx_end) = D(135:146,1:len);
        col = col + len;
    end

    % render
    if toc(t_render) >= render_interval
        drawnow limitrate;
        t_render = tic;
    end

    % quit (multi_trap only, or emergency)
    if strcmp(guidata(task_fig).pressed, 'q')
        break
    end

end

title(ax_t, 'Task complete.', 'Color',colours.text,'FontSize',13);
drawnow;
close(task_fig);   % <-- add here
task_force = task_force(:, 1:col-1);
task_emg   = task_emg(:,   1:col-1);
task_extra = task_extra(:,1:col-1);
disp('Task complete.');

end

%% save_task
function save_task(datapath, task_emg, task_force, task_extra, mvc_value, sampFreq, n_emg, subject, force_dir, task_shape,task_level,task_leg, study, muscle, condition)

signal.data          = task_emg;
signal.fsamp         = sampFreq;
signal.nChan         = n_emg;
signal.ngrid         = 2;
signal.gridname      = {'Muovi+', 'Muovi+'};
signal.muscle        = {'Muscle1', 'Muscle2'};

signal.auxiliary      = task_force;
signal.extra          = task_extra;
signal.extra_channels = 135:146;

if strcmp(task_shape, 'multi_target')
    signal.auxiliaryname = {'Force_L_raw','Force_R_raw', ...
                             'Force_L_norm','Force_R_norm', ...
                             'Target_L','Target_R'};
    signal.target        = task_force(5:6,:);
    Force  = task_force;
    Target = task_force(5:6,:);
else
    signal.auxiliaryname = {'Force_L_raw', 'Force_R_raw', 'Force_Sum_raw', ...
                             'Force_L_norm', 'Force_R_norm', 'Force_Sum_norm', ...
                             'Target'};
    signal.target        = task_force(7,:);
    Force  = task_force;
    Target = task_force(7,:);
end

% for filename 
task_level_pct = round(task_level * 100);  % 0.1 → 10

if strcmpi(task_leg,'bilateral'); lenLeg = 5; else; lenLeg = 4; end

fname = sprintf('%s_%s_%s_%s_%s_%s_%d_%s_%s.mat', ...
    study, subject, muscle, task_leg(1:lenLeg), task_shape, ...
    force_dir, task_level_pct, condition, datestr(now,'yyyymmdd_HHMMSS'));

save(fullfile(datapath, fname), 'signal', 'mvc_value', ...
    'task_leg', 'task_shape', 'Force', 'Target', '-v7.3');


% 
% save(fullfile(datapath, sprintf('%s_%s_%s_%s_%s_%s.mat',subject, task_shape, force_dir, task_leg(1:lenLeg),task_level_str, datestr(now,'yyyymmdd_HHMMSS'))),...
%     'signal', 'mvc_value', 'task_leg', 'task_shape', 'Force', 'Target', '-v7.3');


disp('Task saved.');
end

%% run_multi_target
function [task_force, task_emg] = run_multi_target(tcpSocket, ax, hl, hr, hs, ...
    buf_L, buf_R, buf_S, TotNumByte, blockSamples, bytesPerBlock, ...
    force_left, force_right, offset_L, offset_R, offset_S, ...
    force_dir, sampFreq, mvcLeft, mvcRight, cfg, mcon_cycles, ...
    trap_ramp_s, trap_hold_s, lead_in_s, emg_channels, n_emg, ConvFact, ...
    force_scale, colours)
% run_multi_target — bilateral static-plot display with per-leg targets.
% Each leg gets its own subplot. Target is pre-drawn as a static line.
% A vertical cursor advances in real time; user force trail follows it.
%
% cfg: struct array with fields: leg ('left'|'right'), shape, level
% Both legs share the same timing parameters (trap_ramp_s, trap_hold_s, etc.)

updates_per_sec = sampFreq / blockSamples;

% --- build target traces for each leg ---
n_legs = numel(cfg);
target_traces = cell(n_legs, 1);
for i = 1:n_legs
    lv = cfg(i).level;
    ramp_steps = round(trap_ramp_s * updates_per_sec);
    hold_steps = round(trap_hold_s * updates_per_sec);
    lead_steps = round(lead_in_s   * updates_per_sec);
    switch cfg(i).shape
        case 'trap'
            target_traces{i} = [zeros(1,lead_steps), ...
                linspace(0,lv,ramp_steps), ...
                lv*ones(1,hold_steps), ...
                linspace(lv,0,ramp_steps), ...
                zeros(1,lead_steps)];
        case 'sombrero'
            brim_level = lv * 0.4;
            brim_steps = round(1.5 * updates_per_sec);
            dip_steps  = round(1.0 * updates_per_sec);
            target_traces{i} = [zeros(1,lead_steps), ...
                linspace(0,brim_level,ramp_steps), ...
                brim_level*ones(1,brim_steps), ...
                linspace(brim_level,lv,dip_steps), ...
                lv*ones(1,hold_steps), ...
                linspace(lv,brim_level,dip_steps), ...
                brim_level*ones(1,brim_steps), ...
                linspace(brim_level,0,ramp_steps), ...
                zeros(1,lead_steps)];
        case 'mcon'
            ramp_steps_m = round(trap_ramp_s * updates_per_sec);
            hold_steps_m = round(trap_hold_s * updates_per_sec);
            lead_steps_m = round(lead_in_s   * updates_per_sec);
            t_hold    = linspace(pi/2, pi/2 + (2*mcon_cycles)*pi, hold_steps_m);
            sine_wave = lv + (lv * 0.15 * sin(t_hold));
            target_traces{i} = [zeros(1,lead_steps_m), ...
                linspace(0, sine_wave(1), ramp_steps_m), ...
                sine_wave, ...
                linspace(sine_wave(end), 0, ramp_steps_m), ...
                zeros(1,lead_steps_m)];
        otherwise
            error('multi_target: unsupported shape ''%s''', cfg(i).shape);
    end
end

% All legs must have same length — use shortest if they differ
n_steps = min(cellfun(@numel, target_traces));
for i = 1:n_legs
    target_traces{i} = target_traces{i}(1:n_steps);
end

n_samples  = n_steps * blockSamples;
t_axis_upd = (0:n_steps-1) * (blockSamples / sampFreq);  % time axis in seconds

% --- preallocate storage ---
% rows 1-2: raw force L/R; rows 3-4: normalised L/R; rows 5-6: target L/R
task_force = zeros(6, n_samples);
task_emg   = zeros(n_emg, n_samples);

% --- build display figure ---
mt_fig = figure('Color',colours.bg, 'Name','Multi-Target', ...
    'Units','normalized','OuterPosition',[0 0 1 1]);
guidata(mt_fig, struct('pressed',''));
set(mt_fig,'KeyPressFcn',@(src,e) guidata(src,setfield(guidata(src),'pressed',e.Key)));

ax_mt = gobjects(n_legs, 1);
target_plot_lines = gobjects(n_legs, 1);
user_lines        = gobjects(n_legs, 1);
cursor_lines      = gobjects(n_legs, 1);
ball_lines = gobjects(n_legs, 1);

%leg_colours = struct('left','r', 'right','b');

for i = 1:n_legs
    ax_mt(i) = subplot(n_legs, 1, i, 'Parent', mt_fig);
    hold(ax_mt(i), 'on');
    set(ax_mt(i), 'Color',colours.bg,'XColor',colours.text,'YColor',colours.text, ...
        'XGrid','on','YGrid','on','GridColor',colours.grid);

    lv = cfg(i).level;
    leg = cfg(i).leg;
    %col_str = leg_colours.(leg);
    %col_str = colours.(task_leg);
    col_str = colours.(leg);

    % static target line
    target_plot_lines(i) = plot(ax_mt(i), t_axis_upd, target_traces{i}, ...
        'Color',colours.target, 'LineWidth', 3);

    % user force trail — starts as NaN, fills in as trial runs
    user_lines(i) = plot(ax_mt(i), t_axis_upd, NaN(1,n_steps), ...
        col_str, 'LineWidth', 2.5);

    ball_lines(i) = plot(ax_mt(i), t_axis_upd(1), 0, 'o', 'MarkerSize', colours.ballSize, 'LineStyle', 'none');
    switch cfg(i).leg
        case 'left',  set(ball_lines(i), 'MarkerFaceColor',colours.left,  'MarkerEdgeColor',colours.left);
        case 'right', set(ball_lines(i), 'MarkerFaceColor',colours.right, 'MarkerEdgeColor',colours.right);
    end

    % vertical cursor
    cursor_lines(i) = xline(ax_mt(i), 0, 'w', 'LineWidth', 1.5);

    ylim(ax_mt(i), [-lv*0.2, lv*1.6]);
    xlim(ax_mt(i), [t_axis_upd(1), t_axis_upd(end)]);
    ylabel(ax_mt(i), 'Force (%MVC)', 'Color',colours.text);

    % Y tick as percentage
    yticks_vals = linspace(0, lv, 5);
    yticks(ax_mt(i), yticks_vals);
    yticklabels(ax_mt(i), arrayfun(@(v) sprintf('%d%%', round(v*100)), yticks_vals, 'UniformOutput', false));

    title(ax_mt(i), sprintf('%s leg — %s @ %d%% MVC', ...
        upper(leg(1)), upper(cfg(i).shape), round(lv*100)), ...
        'Color',colours.text,'FontSize',13);
end
xlabel(ax_mt(end), 'Time (s)', 'Color',colours.text);

% --- countdown ---
for ct = 3:-1:1
    for i = 1:n_legs
        title(ax_mt(i), sprintf('%s leg — Starting in %d...', upper(cfg(i).leg(1)), ct), 'Color',colours.text,'FontSize',13);
    end
    drawnow; pause(1);
end
for i = 1:n_legs
    title(ax_mt(i), sprintf('%s leg — %s @ %d%% MVC — FOLLOW THE LINE', ...
        upper(cfg(i).leg(1)), upper(cfg(i).shape), round(cfg(i).level*100)), ...
        'Color',colours.text,'FontSize',13);
end
drawnow;
flush(tcpSocket);

% --- preallocate user force history for display ---
user_force_hist = NaN(n_legs, n_steps);

render_interval = 0.075;
t_render = tic;

col = 1;

for k = 1:n_steps

    while tcpSocket.BytesAvailable < bytesPerBlock, pause(0.001); end
    D = readBlock(tcpSocket, TotNumByte, blockSamples);

    % read force
    if strcmp(force_dir, 'push')
        fL = -(mean(double(D(force_left, :))) - offset_L) * force_scale;
        fR = -(mean(double(D(force_right,:))) - offset_R) * force_scale;
    else
        fL =  (mean(double(D(force_left, :))) - offset_L) * force_scale;
        fR =  (mean(double(D(force_right,:))) - offset_R) * force_scale;
    end

    % normalise per-leg MVC
    dL = fL / mvcLeft;
    dR = fR / mvcRight;

    % map to cfg legs
    leg_vals = struct('left', dL, 'right', dR);

    for i = 1:n_legs
        user_force_hist(i, k) = leg_vals.(cfg(i).leg);
    end

    % update display lines and cursor
    for i = 1:n_legs
        set(user_lines(i),   'YData', user_force_hist(i,:));
        set(cursor_lines(i), 'Value', t_axis_upd(k));
        set(ball_lines(i), 'XData', t_axis_upd(k), 'YData', user_force_hist(i,k));
    end

    % store
    idx_end = min(col + blockSamples - 1, n_samples);
    len     = idx_end - col + 1;
    if strcmp(force_dir, 'push')
        task_force(1, col:idx_end) = -(double(D(force_left, 1:len))  - offset_L) * force_scale;
        task_force(2, col:idx_end) = -(double(D(force_right,1:len))  - offset_R) * force_scale;
    else
        task_force(1, col:idx_end) =  (double(D(force_left, 1:len))  - offset_L) * force_scale;
        task_force(2, col:idx_end) =  (double(D(force_right,1:len))  - offset_R) * force_scale;
    end
    task_force(3, col:idx_end) = task_force(1, col:idx_end) / mvcLeft;
    task_force(4, col:idx_end) = task_force(2, col:idx_end) / mvcRight;
    % target interpolated
    if k < n_steps
        task_force(5, col:idx_end) = linspace(target_traces{1}(k), target_traces{1}(k+1), len);
        task_force(6, col:idx_end) = linspace(target_traces{2}(k), target_traces{2}(k+1), len);
    else
        task_force(5, col:idx_end) = target_traces{1}(k);
        task_force(6, col:idx_end) = target_traces{2}(k);
    end

    task_emg(:, col:idx_end) = double(D(emg_channels, 1:len)) * ConvFact;
    col = col + len;

    % render
    if toc(t_render) >= render_interval
        drawnow limitrate;
        t_render = tic;
    end

    % quit key
    if strcmp(guidata(mt_fig).pressed, 'q')
        break
    end
end

task_force = task_force(:, 1:col-1);
task_emg   = task_emg(:,   1:col-1);

% add titles to show completion
for i = 1:n_legs
    title(ax_mt(i), sprintf('%s leg — Task complete.', upper(cfg(i).leg(1))), 'Color',colours.text,'FontSize',13);
end
drawnow;
disp('Multi-target task complete.');
close(mt_fig);   %

end

%% save_mvc
function save_mvc(datapath, mvc_emg, mvc_force_raw, mvc_force_L, mvc_force_R, mvc_value, sampFreq, n_emg, emg_channels, subject, force_dir,task_leg,study, muscle, condition)

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

% for filename
if strcmpi(task_leg,'bilateral'); lenLeg = 5; else; lenLeg = 4; end
fname = sprintf('mvc_%s_%s_%s_%s_%s_%s_%s.mat', ...
    study, subject, muscle, task_leg(1:lenLeg), ...
    force_dir, condition, datestr(now,'yyyymmdd_HHMMSS'));

save(fullfile(datapath, fname), 'signal_mvc', 'mvc_value', ...
    'task_leg', 'emg_channels', 'Force', '-v7.3');

% save(fullfile(datapath, sprintf('mvc_%s_%s_%s_%s.mat',subject, force_dir, task_leg(1:lenLeg), datestr(now,'yyyymmdd_HHMMSS'))),...
%     'signal_mvc', 'mvc_value', 'emg_channels', 'Force','-v7.3');
    
disp('MVC saved.');
end