%% experiment_sessanta1_singleforce.m
% Live force plot + EMG recording via direct Sessantaquattro+ connection
% Single force/load-cell channel (Forza-j on AUX IN1) — no left/right/bilateral split

close all; clear all; clc;

subject   = 'sub01';   % set per participant
force_dir = 'pull';    % 'push' or 'pull'
study     = 'STUDY1';
muscle    = 'TA';        % e.g. VL, TA, GM
condition = 'testing';   % PRE/POST/etc.
% saving order: STUDY subject muscle task_shape task_level CONDITION

% mvc
mvc        = 1;   % V — set per participant (leave empty to measure live)
mvc_duration = 3;

% all in seconds
% general task
task_shape  = 'trap';   % 'trap' | 'sombrero' | 'mcon' | 'multi_trap' | 'half-sombrero'
task_level  = 0.1;      % target as fraction of MVC
trap_ramp_s = 5;
trap_hold_s = 10;
lead_in_s   = 5;

%multi-trap
multi_trap_rest_s = 2;   % rest between traps (multi_trap only)

%sombrero
use_constant_slope = true;   % true = derive ramps from slope; false = fixed durations
brim_height = 0.4; % fraction of task level (mid plateau height)
brim_length = 5;   % length of the mid plateau
brim_ramp_to_peak = 10; % ramp length from mid to peak (only if use_constant_slope=false)

%mcon
mcon_cycles = 8;

% -------------------------------------------------------------------------
% DISPLAY COLOURS
% -------------------------------------------------------------------------
colours.bg          = 'k';
colours.grid        = [0.3 0.3 0.3];
colours.target      = [0 0.62 0.451];           % CBsafe green
colours.cursor      = 'w';
colours.text        = 'w';
colours.force       = [0.8667 0.1294 0.4902];   % single force trace/ball colour
colours.waitingRoom = 'w';
colours.mvc         = [0.9882 0.5725 0.4471];   % MVC push/pull background
colours.ballSize    = 16;

preComputedMVC = mvc;

% fudge for saving
if trap_hold_s == 0 && ~contains(task_shape,'sombrero')
    task_shape_save = 'ramp';
else
    task_shape_save = task_shape;
end

% AUX input on Sessantaquattro+ (16-bit, HRES=0): LSB=146.48uV, +-6.6V full range
force_scale = 6.6 / 65536;

% =========================================================================
% USER OPTIONS
% =========================================================================
TCPPort      = 45454;
sampFreq     = 2000;
blockSamples = 50; % set to 50 for more sampling

TotNumChan = 72;            % 64 bio + 2 AUX + 4 IMU quaternion + 2 accessory
TotNumByte = 72*2;          % 144 (16-bit, HRES=0)
n_emg        = 64;
emg_channels = 1:64;
force_ch     = 66;          % AUX IN1 (Forza-j)
extra_channels = [71, 72];  % accessory ch1 (trig/buffer bits), sample counter. Unnecessary?

bytesPerBlock = TotNumByte * blockSamples;

datapath = 'C:\Users\masgh\The University of Nottingham\Mathew Piasecki (staff) - ePhys Lab\Michael\';
ConvFact = 0.000286;   % converts raw ADC to mV for EMG

mvc_value = 0;

% =========================================================================
% CONNECT
% =========================================================================
tcpSocket = tcpserver(TCPPort, "ByteOrder","big-endian");
tcpSocket.InputBufferSize = 500000;
while tcpSocket.Connected < 1, pause(0.1); end
disp('Connected. Collecting baseline — keep force at rest...');

FSAMP = 2;   % 2000 Hz (mode ~=3)
NCH   = 3;   % 64 bioelectric ch
MODE  = 0;   % monopolar
HRES  = 0;   % 16-bit
HPF   = 1;
EXTEN = 0; TRIG = 0; REC = 0; GO = 1;

Command = GO + REC*2 + TRIG*4 + EXTEN*16 + HPF*64 + HRES*128 + MODE*256 + NCH*2048 + FSAMP*8192;
write(tcpSocket, Command, 'int16');

% =========================================================================
% BASELINE OFFSET (2 seconds)
% =========================================================================
baseline_samples = sampFreq * 2;
baseline_buf = zeros(1, baseline_samples);
col = 1;
while col <= baseline_samples
    while tcpSocket.BytesAvailable < bytesPerBlock, pause(0.001); end
    D = readBlock(tcpSocket, TotNumByte, blockSamples);
    len = min(blockSamples, baseline_samples - col + 1);
    baseline_buf(col:col+len-1) = D(force_ch, 1:len);
    col = col + len;
end
offset_F = mean(baseline_buf);
fprintf('Offset — Force:%.3f\n', offset_F);

% =========================================================================
% FIGURES
% =========================================================================
N = round(10 * sampFreq / blockSamples);   % 10s rolling window
buf_F = zeros(1,N);

force_fig = figure('Color',colours.waitingRoom,'Name','Force');
ax = axes(force_fig); hold(ax,'on');
set(ax, 'XGrid','on','YGrid','on','XMinorGrid','on','YMinorGrid','on');

hf = plot(ax, 1:N, buf_F, 'Color', colours.force, 'LineWidth',1.5);
title(ax,'Force — M=MVC  T=task  O=offset  Q=quit');
xlabel(ax,'Updates'); ylabel(ax,'Force (offset-corrected)');
ax.YLimMode = 'auto';
xlim(ax,[1 N]);

guidata(force_fig, struct('pressed',''));
set(force_fig,'KeyPressFcn',@(src,e) guidata(src,setfield(guidata(src),'pressed',e.Key)));

if ~isempty(preComputedMVC)
    mvc_value = preComputedMVC;
    ylabel(ax, 'Force (MVC fraction)');
    fprintf('Using precomputed MVC = %.3f\n', mvc_value);
end

% =========================================================================
% MAIN LOOP
% =========================================================================
flush(tcpSocket);
disp('Running. Press M=MVC, Q=quit.');

while ~strcmp(guidata(force_fig).pressed, 'q')

    while tcpSocket.NumBytesAvailable < bytesPerBlock, pause(0.001); end
    D = readBlock(tcpSocket, TotNumByte, blockSamples);

    if strcmp(force_dir, 'push')
        f = -(mean(double(D(force_ch, :))) - offset_F) * force_scale;
    else
        f =  (mean(double(D(force_ch, :))) - offset_F) * force_scale;
    end

    if mvc_value > 0
        buf_F = [buf_F(2:end), f/mvc_value];
    else
        buf_F = [buf_F(2:end), f];
    end
    set(hf,'YData',buf_F);

    %% MVC
    if strcmp(guidata(force_fig).pressed, 'm')
        guidata(force_fig, setfield(guidata(force_fig),'pressed',''));

        if ~isempty(preComputedMVC)
            mvc_value = preComputedMVC;
            buf_F = zeros(1,N);
            set(hf,'YData',buf_F);
            ylabel(ax, 'Force (MVC fraction)');
            fprintf('MVC set to precomputed value: %.3f\n', mvc_value);
        else
            [mvc_value, mvc_emg, mvc_force_raw, buf_F] = run_MVC(tcpSocket, ax, hf, ...
                buf_F, TotNumByte, blockSamples, bytesPerBlock, ...
                force_ch, offset_F, force_dir, sampFreq, mvc_duration, ...
                emg_channels, n_emg, ConvFact, force_scale, colours);

            flush(tcpSocket);
            fprintf('>>> MVC = %.3f <<<\n', mvc_value);

            % append to log
            mvc_csv = fullfile(datapath, 'mvc_log.csv');
            if ~isfile(mvc_csv)
                fid = fopen(mvc_csv, 'w');
                fprintf(fid, 'subject,datetime,MVC\n');
                fclose(fid);
            end
            fid = fopen(mvc_csv, 'a');
            fprintf(fid, '%s,%s,%.5f\n', subject, datestr(now,'yyyymmdd_HHMMSS'), mvc_value);
            fclose(fid);

            if mvc_value > 0
                buf_F = zeros(1,N);
                set(hf,'YData',buf_F);
                ylabel(ax, 'Force (MVC fraction)');
                title(ax, 'Force — M=MVC  T=task  O=offset  Q=quit');
                save_mvc(datapath, mvc_emg, mvc_force_raw, mvc_value, sampFreq, n_emg, emg_channels, subject, force_dir, study, muscle, condition);
            end
        end
    end

    %% TASK
    if strcmp(guidata(force_fig).pressed, 't')
        guidata(force_fig, setfield(guidata(force_fig),'pressed',''));
        if mvc_value == 0
            disp('Run MVC first (press M).');
        else
            [task_force, task_emg, task_extra] = run_task(tcpSocket, ax, hf, ...
                buf_F, TotNumByte, blockSamples, bytesPerBlock, ...
                force_ch, offset_F, force_dir, sampFreq, mvc_value, task_shape, task_level, mcon_cycles, ...
                trap_ramp_s, trap_hold_s, lead_in_s, emg_channels, n_emg, ConvFact, ...
                force_scale, multi_trap_rest_s, colours, brim_height, brim_length, brim_ramp_to_peak, use_constant_slope, extra_channels);

            flush(tcpSocket);

            if ~isempty(task_force)
                save_task(datapath, task_emg, task_force, task_extra, mvc_value, sampFreq, n_emg, subject, force_dir, task_shape_save, task_level, study, muscle, condition, extra_channels);
            end
        end
    end

    % offset!
    if strcmp(guidata(force_fig).pressed, 'o')
        guidata(force_fig, setfield(guidata(force_fig),'pressed',''));
        disp('Recalibrating — release force...');
        title(ax, 'Recalibrating — release force...');
        drawnow;

        baseline_buf = zeros(1, baseline_samples);
        col = 1;
        while col <= baseline_samples
            while tcpSocket.BytesAvailable < bytesPerBlock, pause(0.001); end
            D   = readBlock(tcpSocket, TotNumByte, blockSamples);
            len = min(blockSamples, baseline_samples - col + 1);
            baseline_buf(col:col+len-1) = D(force_ch, 1:len);
            col = col + len;
        end
        offset_F = mean(baseline_buf);
        fprintf('New offset — Force:%.3f\n', offset_F);
        title(ax, 'Force — M=MVC  T=task  O=offset  Q=quit');
    end

    drawnow limitrate;
end

% =========================================================================
% STOP
% =========================================================================
write(tcpSocket, Command-1, 'int16');   % clears GO bit
clear tcpSocket;
% drawnow;
% pause(0.1);
close all;
disp('Done.');


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
function [mvc_value, mvc_emg, mvc_force_raw, buf_F] = run_MVC(tcpSocket, ax, hf, ...
    buf_F, TotNumByte, blockSamples, bytesPerBlock, ...
    force_ch, offset_F, force_dir, sampFreq, mvc_duration, emg_channels, n_emg, ConvFact,...
    force_scale, colours)

for ct = 3:-1:1
    title(ax, sprintf('GET READY... %d', ct));
    drawnow; pause(1);
    if ct == 2
        flush(tcpSocket);
    end
end
title(ax, '*** PUSH NOW ***'); drawnow;
set(ax, 'Color', colours.mvc);
drawnow;

mvc_n         = mvc_duration * sampFreq;
mvc_emg       = zeros(n_emg, mvc_n);
mvc_force_raw = zeros(1, mvc_n);

col     = 1;
t_start = tic;

while col <= mvc_n
    elapsed   = toc(t_start);
    remaining = max(0, mvc_duration - elapsed);
    title(ax, sprintf('PUSH! %d', ceil(remaining)));

    while tcpSocket.BytesAvailable < bytesPerBlock, pause(0.001); end
    D = readBlock(tcpSocket, TotNumByte, blockSamples);

    if strcmp(force_dir,'push')
        f = -(mean(double(D(force_ch, :))) - offset_F) * force_scale;
    else
        f =  (mean(double(D(force_ch, :))) - offset_F) * force_scale;
    end
    buf_F = [buf_F(2:end), f];
    set(hf,'YData',buf_F);

    idx_end = min(col+blockSamples-1, mvc_n);
    len     = idx_end - col + 1;

    if strcmp(force_dir,'push')
        mvc_force_raw(col:idx_end) = -(double(D(force_ch,1:len)) - offset_F) * force_scale;
    else
        mvc_force_raw(col:idx_end) =  (double(D(force_ch,1:len)) - offset_F) * force_scale;
    end

    mvc_emg(:, col:idx_end) = double(D(emg_channels, 1:len)) * ConvFact;

    col = col + len;
    drawnow limitrate;
end

title(ax, 'MVC complete');
set(ax, 'Color', colours.waitingRoom);

fprintf('mvc_force_raw range: %.4f to %.4f\n', min(mvc_force_raw), max(mvc_force_raw));

mvc_value = max(mvc_force_raw);

old_mf = findobj('Type','figure','Name','MVC Peak');
if ~isempty(old_mf), close(old_mf); end

mf = figure;
plot(mvc_force_raw, 'Color', colours.force, 'LineWidth', 1.5); hold on;
yline(mvc_value, 'k--', sprintf('Peak: %.3f', mvc_value), ...
    'LabelVerticalAlignment','top', 'LabelHorizontalAlignment','left');
title(sprintf('Peak MVC: %.3f', mvc_value));
xlabel('Samples'); ylabel('Force (ADC)');

drawnow;
pause(0.2);

end


%% run_task (static-plot display, single force channel)
function [task_force, task_emg, task_extra] = run_task(tcpSocket, ax, hf, ...
    buf_F, TotNumByte, blockSamples, bytesPerBlock, ...
    force_ch, offset_F, force_dir, sampFreq, mvc_value, task_shape, task_level, mcon_cycles,...
    trap_ramp_s, trap_hold_s, lead_in_s, emg_channels, n_emg, ConvFact,...
    force_scale, multi_trap_rest_s, colours, brim_height, brim_length, brim_ramp_to_peak, use_constant_slope, extra_channels)

updates_per_sec = sampFreq / blockSamples;
ramp_steps  = round(trap_ramp_s * updates_per_sec);
hold_steps  = round(trap_hold_s * updates_per_sec);
lead_steps  = round(lead_in_s   * updates_per_sec);

if use_constant_slope
    slope_rate  = task_level / trap_ramp_s;
    brim_level  = task_level * brim_height;
    ramp1_steps = round((brim_level / slope_rate) * updates_per_sec);
    ramp2_steps = round(((task_level - brim_level) / slope_rate) * updates_per_sec);
    ramp_down_steps = round((task_level / slope_rate) * updates_per_sec);
else
    brim_level  = task_level * brim_height;
    ramp1_steps = ramp_steps;
    ramp2_steps = round(brim_ramp_to_peak * updates_per_sec);
    ramp_down_steps = ramp_steps;
end

switch task_shape
    case 'trap'
        target_trace = [zeros(1,lead_steps), ...
            linspace(0,task_level,ramp_steps), ...
            task_level*ones(1,hold_steps), ...
            linspace(task_level,0,ramp_steps), ...
            zeros(1,lead_steps)];

    case 'sombrero'
        brim_steps = round(brim_length * updates_per_sec);
        target_trace = [zeros(1,lead_steps), ...
            linspace(0,brim_level,ramp1_steps), ...
            brim_level*ones(1,brim_steps), ...
            linspace(brim_level,task_level,ramp2_steps), ...
            task_level*ones(1,hold_steps), ...
            linspace(task_level,brim_level,ramp2_steps), ...
            brim_level*ones(1,brim_steps), ...
            linspace(brim_level,0,ramp1_steps), ...
            zeros(1,lead_steps)];

    case 'half-sombrero'
        brim_steps = round(brim_length * updates_per_sec);
        target_trace = [zeros(1,lead_steps), ...
            linspace(0,brim_level,ramp1_steps), ...
            brim_level*ones(1,brim_steps), ...
            linspace(brim_level,task_level,ramp2_steps), ...
            task_level*ones(1,hold_steps), ...
            linspace(task_level,0,ramp_down_steps), ...
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

% preallocate storage (3 rows: raw force, norm force, target)
task_force = zeros(3, n_samples);
task_emg   = zeros(n_emg, n_samples);
n_extra = numel(extra_channels);
task_extra = zeros(n_extra, n_samples);

task_fig = figure('Color',colours.bg,'Name',sprintf('Task — %s %.0f%%MVC', ...
    task_shape, task_level*100), ...
    'Units','normalized','OuterPosition',[0 0 1 1]);
guidata(task_fig, struct('pressed',''));
set(task_fig,'KeyPressFcn',@(src,e) guidata(src,setfield(guidata(src),'pressed',e.Key)));

ax_t = axes(task_fig);
hold(ax_t,'on');
set(ax_t, 'Color',colours.bg, 'XColor',colours.text, 'YColor',colours.text, ...
    'XGrid','on','YGrid','on','GridColor',colours.grid);

plot(ax_t, t_axis_upd, target_trace, 'Color',colours.target, 'LineWidth', 5);
user_line = plot(ax_t, t_axis_upd, NaN(1,n_steps), 'Color', colours.force, 'LineWidth', 3);
ball = plot(ax_t, t_axis_upd(1), 0, 'o', 'MarkerSize', colours.ballSize, 'LineStyle', 'none');
set(ball, 'MarkerFaceColor','none', 'MarkerEdgeColor',colours.force, 'LineWidth', 2);
cursor_line = xline(ax_t, 0, colours.cursor, 'LineWidth', 1.5);

ylim(ax_t, [-task_level*0.2, task_level*1.6]);
xlim(ax_t, [t_axis_upd(1), t_axis_upd(end)]);
xlabel(ax_t, 'Time (s)', 'Color',colours.text);
ylabel(ax_t, 'Force (%MVC)', 'Color',colours.text);
yticks_vals = linspace(0, task_level, 5);
yticks(ax_t, yticks_vals);
yticklabels(ax_t, arrayfun(@(v) sprintf('%d%%',round(v*100)), yticks_vals,'UniformOutput',false));

for ct = 3:-1:1
    title(ax_t, sprintf('%s @ %d%% MVC — Starting in %d...', ...
        upper(task_shape), round(task_level*100), ct), 'Color',colours.text,'FontSize',13);
    drawnow; pause(1);
end
title(ax_t, sprintf('%s @ %d%% MVC — FOLLOW THE LINE', upper(task_shape), round(task_level*100)), ...
    'Color',colours.text,'FontSize',13);
drawnow;
flush(tcpSocket);

user_force_hist = NaN(1, n_steps);

render_interval = 0.075;
t_render = tic;
col = 1;

for k = 1:n_steps

    while tcpSocket.BytesAvailable < bytesPerBlock, pause(0.001); end
    D = readBlock(tcpSocket, TotNumByte, blockSamples);

    if strcmp(force_dir,'push')
        f = -(mean(double(D(force_ch, :))) - offset_F) * force_scale;
    else
        f =  (mean(double(D(force_ch, :))) - offset_F) * force_scale;
    end
    disp_val = f/mvc_value;

    user_force_hist(k) = disp_val;
    set(user_line,   'YData', user_force_hist);
    set(ball, 'XData', t_axis_upd(k), 'YData', disp_val);
    set(cursor_line, 'Value', t_axis_upd(k));

    if do_record
        idx_end = min(col+blockSamples-1, n_samples);
        len     = idx_end - col + 1;
        if strcmp(force_dir,'push')
            task_force(1,col:idx_end) = -(double(D(force_ch, 1:len)) - offset_F) * force_scale;
        else
            task_force(1,col:idx_end) =  (double(D(force_ch, 1:len)) - offset_F) * force_scale;
        end
        task_force(2,col:idx_end) = task_force(1,col:idx_end) / mvc_value;
        if k < n_steps
            task_force(3,col:idx_end) = linspace(target_trace(k), target_trace(k+1), len);
        else
            task_force(3,col:idx_end) = target_trace(k);
        end
        task_emg(:,col:idx_end) = double(D(emg_channels,1:len)) * ConvFact;
        task_extra(:,col:idx_end) = D(extra_channels,1:len);
        col = col + len;
    end

    if toc(t_render) >= render_interval
        drawnow limitrate;
        t_render = tic;
    end

    if strcmp(guidata(task_fig).pressed, 'q')
        break
    end
end

title(ax_t, 'Task complete.', 'Color',colours.text,'FontSize',13);
drawnow;
pause(0.2); 
close(task_fig);
task_force = task_force(:, 1:col-1);
task_emg   = task_emg(:,   1:col-1);
task_extra = task_extra(:,1:col-1);
disp('Task complete.');

end

%% save_task
function save_task(datapath, task_emg, task_force, task_extra, mvc_value, sampFreq, n_emg, subject, force_dir, task_shape, task_level, study, muscle, condition, extra_channels)

signal.data          = task_emg;
signal.fsamp         = sampFreq;
signal.nChan         = n_emg;
signal.ngrid         = 1;
signal.gridname      = {'Sessantaquattro+'};
signal.muscle        = {muscle};

signal.auxiliary       = task_force;
signal.auxiliaryname   = {'Force_raw', 'Force_norm', 'Target'};
signal.extra            = task_extra;
signal.extra_channels   = extra_channels;
signal.target           = task_force(3,:);

Force  = task_force;
Target = task_force(3,:);

task_level_pct = round(task_level * 100);  % 0.1 -> 10

fname = sprintf('%s_%s_%s_%s_%s_%d_%s_%s.mat', ...
    study, subject, muscle, task_shape, force_dir, task_level_pct, condition, datestr(now,'yyyymmdd_HHMMSS'));

save(fullfile(datapath, fname), 'signal', 'mvc_value', ...
    'task_shape', 'Force', 'Target', '-v7.3');

disp('Task saved.');
end

%% save_mvc
function save_mvc(datapath, mvc_emg, mvc_force_raw, mvc_value, sampFreq, n_emg, emg_channels, subject, force_dir, study, muscle, condition)

signal_mvc.data          = mvc_emg;
signal_mvc.fsamp         = sampFreq;
signal_mvc.nChan         = n_emg;
signal_mvc.ngrid         = 1;
signal_mvc.gridname      = {'Sessantaquattro+'};
signal_mvc.muscle        = {muscle};
signal_mvc.auxiliary     = mvc_force_raw;
signal_mvc.auxiliaryname = {'Force_raw'};
signal_mvc.target        = [];

Force = mvc_force_raw;

fname = sprintf('mvc_%s_%s_%s_%s_%s_%s.mat', ...
    study, subject, muscle, force_dir, condition, datestr(now,'yyyymmdd_HHMMSS'));

save(fullfile(datapath, fname), 'signal_mvc', 'mvc_value', ...
    'emg_channels', 'Force', '-v7.3');

disp('MVC saved.');
end