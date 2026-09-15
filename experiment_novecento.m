%% experiment_novecento1.m
% Live force plot + EMG recording via Novecento+ (direct TCP, not SyncStation)
% Copies experiment_muovi6.m functionality but uses the Novecento+
% communication protocol (15-byte ConfString, per-input probe config,
% 16-ch rear AUX block incl. LOAD CELL 1/2) instead of Muovi/SyncStation.
%
% MA Sept 2026

close all; clear all; clc;

subject   = 'sub01';
force_dir = 'push';
study     = 'NOVE';
muscle    = 'TA';
condition = 'testing';

mvcLeft      = 200;
mvcRight     = 200;
mvc_duration = 3; 

% multi_trap is Avi's fatigue protocol
% multi_target is prototype - dont use
task_shape  = 'trap'; % 'trap' | 'sombrero' | 'mcon' | 'multi_trap' | 'multi_target' | 'half-sombrero'
task_level  = 0.1;
task_leg    = 'right';
trap_ramp_s = 5;
trap_hold_s = 10;
lead_in_s   = 5; % normal traps use this for lead in and lead out
multi_trap_rest_s = 2; % lead out for multi_trap (fatigue)

use_constant_slope = true;
brim_height = 0.4;
brim_length = 5;
brim_ramp_to_peak = 10;
mcon_cycles = 8;
left_multi_target  = 0.5;
right_multi_target = 0.25;

colours.bg          = 'k';
colours.grid        = [0.3 0.3 0.3];
colours.target      = [0 0.62 0.451];
colours.cursor      = 'w';
colours.text        = 'w';
colours.left        = [0.835 0.369 0];
colours.right       = [0.8667 0.1294 0.4902];
colours.bilateral   = 'w';
colours.waitingRoom = 'w';
colours.mvc         = [0.9882 0.5725 0.4471];
colours.ballSize    = 16;

multi_target_cfg(1) = struct('leg','left',  'shape','trap', 'level',left_multi_target);
multi_target_cfg(2) = struct('leg','right', 'shape','trap', 'level',right_multi_target);

if ~strcmp(task_shape, 'multi_target')
    switch task_leg
        case 'left',      preComputedMVC = mvcLeft;
        case 'right',     preComputedMVC = mvcRight;
        case 'bilateral', preComputedMVC = mvcLeft + mvcRight;
    end
else
    preComputedMVC = mvcLeft;
end

if trap_hold_s == 0 && ~contains(task_shape,'sombrero')
    task_shape_save = 'ramp';
else
    task_shape_save = task_shape;
end

% =========================================================================
% NOVECENTO+ CONNECTION / PROTOCOL CONFIG
% =========================================================================
WiFi     = 0;
IP_USB   = '169.254.1.10';
IP_WiFi  = '192.168.1.1';
TCPPort  = 23456;

nGrids   = 6;         % number of BIO64HD (64ch) grids in use — increase up to 6 (IN1..IN6)
FSelAux  = 2;          % rear AUX/LOAD CELL block sampling rate: 1=500,2=2000,3=4000,4=8000 Hz
AuxFsampCode = [0 16 32 48];  % ACQ_SETT_A codes for the 4 rates above
FsampVal     = [500 2000 4000 8000];

GridFsampSel = 1;      % INx_CONF FSAMP<1:0> code for the grids: 00=500 01=2000 10=4000 11=8000
sampFreq     = FsampVal(GridFsampSel+1);   % must equal the grid rate used below (2000 Hz default)
if AuxFsampCode(FSelAux) == 0 && FSelAux ~= 1
    error('FSelAux/AuxFsampCode mismatch');
end
if FsampVal(FSelAux) ~= sampFreq
    warning('AUX (load cell) rate (%d Hz) differs from grid EMG rate (%d Hz) — this script assumes they match.', FsampVal(FSelAux), sampFreq);
end

blockPeriods500 = 13;                 % number of 500Hz base periods read per GUI update
mult            = sampFreq/500;       % grid/AUX samples per base period (4 @ 2000Hz)
accMult         = 8000/500;           % accessory samples per base period (16 @ 8000Hz, fixed)
blockSamples    = mult * blockPeriods500;   % samples-per-channel per read, at sampFreq
% DISPLAY RATE: sampFreq/blockSamples updates/sec (~38 Hz @ defaults, similar to muovi6)

n_bio_per_grid = 64;
n_ext_per_grid = 6;              % 4 IMU + 2 ACC
NumChanGrid    = n_bio_per_grid + n_ext_per_grid;  % 70, BIO64HD probe type
n_emg          = nGrids * n_bio_per_grid;

ConvFact = 0.0002861;   % mV per count for HRES=0, Gain code 00 (286.1 nV resolution)

% % ---- LOAD CELL calibration — 
% calibration ephys lab
% 514.98x + 63.529
% 1/514.98
% newtons = (1/514.98) *9.81

% force_scale_L = 514.98;   % Slope (m)
% force_scale_R = 514.98;   % Update if right cell differs
% cal_intercept_L = 63.529; % Intercept (c)
% cal_intercept_R = 63.529;

% ---- LOAD CELL ----
% Currently using raw Novecento load-cell counts.
% Set to 1 for now because the load cells have not been calibrated yet.

% S-type
% (1 / 514.98) * 9.81 = 0.01905 N per count
force_scale_L = 0.01905;   
force_scale_R = 0.01905;   

% B1411991 load cell
% 254.54x + 10.173
% (1 / 254.54) * 9.81 = 0.0385 N per count

% Arm rig
% (1/ 531.79 )*9.81 = 0.0185 

%force_scale_L = 1;   
%force_scale_R = 1;   

datapath = 'C:\Users\masgh\The University of Nottingham\Mathew Piasecki (staff) - ePhys Lab\Michael\';

emg_ylim_std = [0 500];
mvc_value = 0;

% =========================================================================
% BUILD ConfString (15 bytes, Novecento+ protocol v2.3)
% =========================================================================
IN_Active = zeros(10,1);
Mode  = zeros(10,1);
Gain  = zeros(10,1);
HPF   = zeros(10,1);
HRES  = zeros(10,1);
Fsamp = zeros(10,1);

for g = 1:nGrids
    IN_Active(g) = 1;
    Mode(g)  = 0;               % monopolar
    Gain(g)  = 0;                % preamp gain 8 (HRES=0) -> range +/-9.375mV
    HPF(g)   = 1;                 % 10.5 Hz HPF @ 2000Hz
    HRES(g)  = 0;                 % 16-bit
    Fsamp(g) = GridFsampSel;      % 2000 Hz
end

ConfString = zeros(1,15);
ConfString(1) = bin2dec('10000000') + AuxFsampCode(FSelAux) + IN_Active(10)*2 + IN_Active(9);
ConfString(2) = 0;
for i = 1:8
    ConfString(2) = ConfString(2) + IN_Active(i)*(2^(i-1));
end
ConfString(3) = 0;   % AN_OUT_A — analog out unused
ConfString(4) = 1;   % AN_OUT_B — analog out channel unused
for i = 1:10
    ConfString(4+i) = Mode(i)*64 + Gain(i)*16 + HPF(i)*8 + HRES(i)*4 + Fsamp(i);
end
ConfString(15) = CRC8(ConfString, 14);

% =========================================================================
% CONNECT
% =========================================================================
if WiFi == 1
    tcpSocket = tcpclient(IP_WiFi, TCPPort);
else
    tcpSocket = tcpclient(IP_USB, TCPPort);
end
tcpSocket.ByteOrder = "little-endian";
tcpSocket.Timeout = 100;
disp('Connected to Novecento+');

% 1. Flush any leftover bytes from previous crashes/runs
pause(1);
flush(tcpSocket);

% --- Hardware config request: confirm probe types on the active inputs ---
GetSetCmd = zeros(1,2);
GetSetCmd(1) = 1;
GetSetCmd(2) = CRC8(GetSetCmd,1);

% add this -------------
maxRetries = 5;
for attempt = 1:maxRetries
    flush(tcpSocket);
    write(tcpSocket, GetSetCmd, 'uint8');
    while tcpSocket.NumBytesAvailable < 20
        pause(0.01);
    end
    Settings = read(tcpSocket, 20, 'uint8')';
    if Settings(2) == 5   % < BIO64HD's  probe code
        break;
    end
    if attempt == maxRetries
        warning('Probe code still unexpected after %d attempts, proceeding anyway.', maxRetries);
    end
    pause(0.5);
end
% ---------------------------

% write(tcpSocket, GetSetCmd, 'uint8');
% while tcpSocket.NumBytesAvailable < 20
%     pause(0.01);
% end
% Settings = read(tcpSocket, 20, 'uint8')';

ChVsType = [0 14 22 38 46 70 102 0 0 0 0 0 0 0 0 0];
NumChan  = zeros(10,1);
Ptr_IN   = zeros(11,1);
Size_IN  = zeros(10,1);
Ptr_IN(1) = 1;


for i = 1:10

    NumChan(i) = ChVsType(Settings(i+1)+1);

    fprintf('Input %d: probe code = %d, channels = %d\n', ...
        i, Settings(i+1), NumChan(i));

    if IN_Active(i) == 1 && NumChan(i) ~= NumChanGrid
        error(['Input %d is configured as active, but Novecento reports ' ...
               'probe code %d (%d channels). Expected BIO64HD (70 channels).'], ...
               i, Settings(i+1), NumChan(i));
    end

    if IN_Active(i) == 1
        Size_IN(i) = mult * NumChan(i);
    else
        Size_IN(i) = 0;
    end

    Ptr_IN(i+1) = Ptr_IN(i) + Size_IN(i);
end

PacketSize1Block = (Ptr_IN(11)-1) + 16*mult + 128;   % grids + AUX(16ch) + accessory(4ch*32bit@8kHz)
bytesPerBlock     = PacketSize1Block * blockPeriods500 * 2;   % int16 = 2 bytes
tcpSocket.InputBufferSize = bytesPerBlock * 20;

% channel row layout of the combined D matrix returned by readBlock():
%   rows 1 : nGrids*70        -> grids, each [64 BIO, 4 IMU, 2 ACC]
%   rows nGrids*70+1 : +16    -> rear AUX block: AUX1-4, LOADCELL_L, LOADCELL_R, EXT1-10
%   rows end-3 : end          -> accessory ch 1-4 (32-bit: counter, status/trigger, blk ctr, DAC ctr)
force_left  = nGrids*70 + 5;
force_right = nGrids*70 + 6;
accStart    = nGrids*70 + 16 + 1;
extra_channels  = accStart : (accStart+3);  % ACC1-ACC4
% later do this :
% ACC2 = uint32(signal.extra(2,:));
% trigger = bitget(ACC2,1);

emg_channels = [];
for g = 1:nGrids
    base = (g-1)*70;
    emg_channels = [emg_channels, base+1 : base+64]; %#ok<AGROW>
end

% --- send the acquisition-start configuration ---
write(tcpSocket, ConfString, 'uint8');
pause(0.2);
%flush(tcpSocket);
clear_tcp_backlog(tcpSocket, bytesPerBlock);
disp('Novecento+ streaming started.');

% =========================================================================
% BASELINE OFFSET (2 seconds)
% =========================================================================
baseline_samples = sampFreq * 2;
baseline_buf = zeros(2, baseline_samples);
col = 1;
while col <= baseline_samples
    while tcpSocket.BytesAvailable < bytesPerBlock, pause(0.001); end
    D = readBlockNovecento(tcpSocket, PacketSize1Block, blockPeriods500, Ptr_IN, nGrids, mult, accMult, blockSamples);
    len = min(blockSamples, baseline_samples - col + 1);
    baseline_buf(1, col:col+len-1) = D(force_left,  1:len);
    baseline_buf(2, col:col+len-1) = D(force_right, 1:len);
    col = col + len;
end
offset_L = mean(baseline_buf(1,:));
offset_R = mean(baseline_buf(2,:));
fprintf('Offsets (counts) — L:%.3f  R:%.3f\n', offset_L, offset_R);

% =========================================================================
% FIGURES
% =========================================================================
N = round(10 * sampFreq / blockSamples);
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
xlabel(ax,'Updates'); ylabel(ax,'Force (N, offset-corrected)');
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
%flush(tcpSocket);
disp('Running. Press M=MVC, Q=quit.');

while ~strcmp(guidata(force_fig).pressed, 'q')

    while tcpSocket.NumBytesAvailable < bytesPerBlock, pause(0.001); end
    D = readBlockNovecento(tcpSocket, PacketSize1Block, blockPeriods500, Ptr_IN, nGrids, mult, accMult, blockSamples);

    if strcmp(force_dir, 'push')
        fL = (mean(double(D(force_left, :))) - offset_L) * force_scale_L;
        fR = (mean(double(D(force_right,:))) - offset_R) * force_scale_R;
    else
        fL =  -(mean(double(D(force_left, :))) - offset_L) * force_scale_L;
        fR =  -(mean(double(D(force_right,:))) - offset_R) * force_scale_R;
    end
    fS = fL + fR;

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
 
            clear_tcp_backlog(tcpSocket, bytesPerBlock);

            %flush(tcpSocket);
            

            [mvc_value, mvc_value_L, mvc_value_R, mvc_emg, mvc_force_raw, mvc_force_L, mvc_force_R, buf_L, buf_R, buf_S] = run_MVC(tcpSocket, ax, hl, hr, hs, ...
                buf_L, buf_R, buf_S, PacketSize1Block, blockPeriods500, Ptr_IN, nGrids, mult, accMult, blockSamples, bytesPerBlock, ...
                force_left, force_right, offset_L, offset_R, ...
                force_dir, sampFreq, mvc_duration, emg_channels, n_emg, ConvFact,...
                force_scale_L, force_scale_R, colours, task_leg);

            %flush(tcpSocket);
            clear_tcp_backlog(tcpSocket, bytesPerBlock);


            switch task_leg
                case 'left',      mvc_value = mvc_value_L;
                case 'right',     mvc_value = mvc_value_R;
                case 'bilateral', mvc_value = mvc_value;
            end
            %fprintf('>>> MVC = %.3f N <<<\n', mvc_value);
            fprintf('>>> MVC = %.3f raw counts <<<\n', mvc_value);

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
                buf_L = zeros(1,N); buf_R = zeros(1,N); buf_S = zeros(1,N);
                set(hl,'YData',buf_L); set(hr,'YData',buf_R); set(hs,'YData',buf_S);
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
                    buf_L, buf_R, buf_S, PacketSize1Block, blockPeriods500, Ptr_IN, nGrids, mult, accMult, blockSamples, bytesPerBlock, ...
                    force_left, force_right, offset_L, offset_R, ...
                    force_dir, sampFreq, mvcLeft, mvcRight, multi_target_cfg, mcon_cycles, ...
                    trap_ramp_s, trap_hold_s, lead_in_s, emg_channels, n_emg, ConvFact, ...
                    force_scale_L, force_scale_R, colours, brim_height, brim_length, brim_ramp_to_peak, use_constant_slope);
            elseif strcmp(task_shape, 'multi_trap')
                [task_force, task_emg, task_extra] = run_multi_trap_fatigue(tcpSocket, ...
                    PacketSize1Block, blockPeriods500, Ptr_IN, nGrids, mult, accMult, blockSamples, bytesPerBlock, ...
                    force_left, force_right, offset_L, offset_R, ...
                    force_dir, sampFreq, mvc_value, task_leg, task_level, ...
                    trap_ramp_s, trap_hold_s, lead_in_s, multi_trap_rest_s, ...
                    emg_channels, n_emg, ConvFact, force_scale_L, force_scale_R, colours, extra_channels);
            else
                [task_force, task_emg, task_extra] = run_task(tcpSocket, ax, hl, hr, hs, ...
                    buf_L, buf_R, buf_S, PacketSize1Block, blockPeriods500, Ptr_IN, nGrids, mult, accMult, blockSamples, bytesPerBlock, ...
                    force_left, force_right, offset_L, offset_R, ...
                    force_dir, sampFreq, mvc_value, task_leg, task_shape, task_level, mcon_cycles, ...
                    trap_ramp_s, trap_hold_s, lead_in_s, emg_channels, n_emg, ConvFact,...
                    force_scale_L, force_scale_R, multi_trap_rest_s, colours, brim_height, brim_length, brim_ramp_to_peak, use_constant_slope, extra_channels);
            end

            %flush(tcpSocket);
            clear_tcp_backlog(tcpSocket, bytesPerBlock);

            if ~isempty(task_force)
                if strcmp(task_shape, 'multi_target')
                    save_leg   = 'bilateral';
                    save_level = max([multi_target_cfg.level]);
                else
                    save_leg   = task_leg;
                    save_level = task_level;
                end
                save_task(datapath, task_emg, task_force, task_extra, mvc_value, sampFreq, n_emg, subject, force_dir, task_shape_save, save_level, save_leg, study, muscle, condition, extra_channels);
            end
        end
    end

    % offset!
    if strcmp(guidata(force_fig).pressed, 'o')
        guidata(force_fig, setfield(guidata(force_fig),'pressed',''));
        disp('Recalibrating — release force...');
        title(ax, 'Recalibrating — release force...');
        drawnow;

        baseline_buf = zeros(2, baseline_samples);
        col = 1;
        while col <= baseline_samples
            while tcpSocket.BytesAvailable < bytesPerBlock, pause(0.001); end
            D   = readBlockNovecento(tcpSocket, PacketSize1Block, blockPeriods500, Ptr_IN, nGrids, mult, accMult, blockSamples);
            len = min(blockSamples, baseline_samples - col + 1);
            baseline_buf(1, col:col+len-1) = D(force_left,  1:len);
            baseline_buf(2, col:col+len-1) = D(force_right, 1:len);
            col = col + len;
        end
        offset_L = mean(baseline_buf(1,:));
        offset_R = mean(baseline_buf(2,:));
        fprintf('New offsets (counts) — L:%.3f  R:%.3f\n', offset_L, offset_R);
        title(ax, 'Force — M=MVC  T=task  O=offset  Q=quit');
    end

    drawnow limitrate;

end

% =========================================================================
% STOP
% =========================================================================
ConfString(1) = bin2dec('00000000');
ConfString(15) = CRC8(ConfString, 14);
write(tcpSocket, ConfString, 'uint8');
pause(1);
clear tcpSocket;
close all;
disp('Done.');


%%
%%% ---------------------------------------------
%%% FUNCTIONS
%%% ---------------------------------------------

%% readBlockNovecento
% Reads blockPeriods500 base periods (500 Hz granularity) from Novecento+
% and re-assembles into a combined channel x sample matrix D at sampFreq:
%   rows 1:nGrids*70            grids [64 BIO, 4 IMU, 2 ACC each]
%   rows nGrids*70+1 : +16      rear AUX block (AUX1-4, LOADCELL_L, LOADCELL_R, EXT1-10)
%   rows end-3:end              accessory ch1-4 (32-bit, decimated from 8kHz to sampFreq)
function D = readBlockNovecento_old(tcpSocket, PacketSize1Block, nPeriods500, Ptr_IN, nGrids, mult, accMult, blockSamples)

Temp = read(tcpSocket, PacketSize1Block*nPeriods500, 'int16')';
Temp = reshape(Temp, PacketSize1Block, nPeriods500);

D_grids = zeros(nGrids*70, blockSamples);
for g = 1:nGrids
    rows  = Ptr_IN(g):Ptr_IN(g+1)-1;
    slice = Temp(rows, :);
    flat  = reshape(slice, 1, []);
    chunk = reshape(flat, 70, blockSamples);
    D_grids((g-1)*70+1 : g*70, :) = chunk;
end

auxRows  = Ptr_IN(11) : Ptr_IN(11)+16*mult-1;
auxSlice = Temp(auxRows, :);
auxFlat  = reshape(auxSlice, 1, []);
D_aux    = reshape(auxFlat, 16, blockSamples);

accRowsStart = auxRows(end) + 1;
accRows      = accRowsStart : accRowsStart + 128 - 1;
accSlice     = int16(reshape(Temp(accRows, :), 1, []));
accInt32     = typecast(accSlice, 'int32');
accChunk     = reshape(accInt32, 4, accMult*nPeriods500);
decim        = accMult / mult;
D_acc        = accChunk(:, decim:decim:end);   % decimate 8kHz -> sampFreq

D = [D_grids; D_aux; D_acc];
end

%% run_MVC
function [mvc_value, mvc_value_L, mvc_value_R, mvc_emg, mvc_force_raw,  mvc_force_L, mvc_force_R, buf_L, buf_R, buf_S] = run_MVC(tcpSocket, ax, hl, hr, hs, ...
    buf_L, buf_R, buf_S, PacketSize1Block, nPeriods500, Ptr_IN, nGrids, mult, accMult, blockSamples, bytesPerBlock, ...
    force_left, force_right, offset_L, offset_R, ...
    force_dir, sampFreq, mvc_duration, emg_channels, n_emg, ConvFact,...
    force_scale_L, force_scale_R, colours, task_leg)

mvc_n_baseline = 1 * sampFreq;                    % 1s hidden baseline 
mvc_n_push     = mvc_duration * sampFreq;
mvc_n          = mvc_n_baseline + mvc_n_push;      % total recorded samples %
%mvc_n         = mvc_duration * sampFreq;

mvc_emg       = zeros(n_emg, mvc_n);
mvc_force_raw = zeros(1, mvc_n);
mvc_force_L   = zeros(1, mvc_n);
mvc_force_R   = zeros(1, mvc_n);
col           = 1;

% this is complicated, because we want to secretly collect 1s data before
% PUSH NOW
% cant flush tcp as that breaks the force code. 

for ct = 3:-1:1
    title(ax, sprintf('GET READY... %d', ct));

    if ct == 2
        drawnow; pause(1);
        clear_tcp_backlog(tcpSocket, bytesPerBlock);

    elseif ct == 1
        % --- silently record this second as baseline instead of pausing ---
        drawnow;
        while col <= mvc_n_baseline
            while tcpSocket.BytesAvailable < bytesPerBlock, pause(0.001); end
            D = readBlockNovecento(tcpSocket, PacketSize1Block, nPeriods500, Ptr_IN, nGrids, mult, accMult, blockSamples);

            idx_end = min(col+blockSamples-1, mvc_n_baseline);
            len     = idx_end - col + 1;

            if strcmp(force_dir,'push')
                mvc_force_L(col:idx_end) = (double(D(force_left, 1:len))  - offset_L) * force_scale_L;
                mvc_force_R(col:idx_end) = (double(D(force_right,1:len))  - offset_R) * force_scale_R;
            else
                mvc_force_L(col:idx_end) = -(double(D(force_left, 1:len))  - offset_L) * force_scale_L;
                mvc_force_R(col:idx_end) = -(double(D(force_right,1:len))  - offset_R) * force_scale_R;
            end
            mvc_emg(:, col:idx_end) = double(D(emg_channels, 1:len)) * ConvFact;

            col = col + len;
        end

    else % ct == 3
        drawnow; pause(1);
    end
end



% for ct = 3:-1:1
%     title(ax, sprintf('GET READY... %d', ct));
%     drawnow; pause(1);
%     if ct == 2
%         clear_tcp_backlog(tcpSocket, bytesPerBlock);
%     end
% end


title(ax, '*** PUSH NOW ***'); drawnow;
set(ax, 'Color', colours.mvc);
drawnow;

% mvc_n         = mvc_duration * sampFreq;
% mvc_emg       = zeros(n_emg, mvc_n);
% mvc_force_raw = zeros(1, mvc_n);
% mvc_force_L   = zeros(1, mvc_n);
% mvc_force_R   = zeros(1, mvc_n);

% col     = 1;
t_start = tic;
mvc_start_time = tic;



while col <= mvc_n
    elapsed   = toc(t_start);
    remaining = max(0, mvc_duration - elapsed);
    title(ax, sprintf('PUSH! %d', ceil(remaining)));

    while tcpSocket.BytesAvailable < bytesPerBlock, pause(0.001); end
    D = readBlockNovecento(tcpSocket, PacketSize1Block, nPeriods500, Ptr_IN, nGrids, mult, accMult, blockSamples);

    if strcmp(force_dir,'push')
        fL = (mean(double(D(force_left, :))) - offset_L) * force_scale_L;
        fR = (mean(double(D(force_right,:))) - offset_R) * force_scale_R;
    else
        fL = -(mean(double(D(force_left, :))) - offset_L) * force_scale_L;
        fR = -(mean(double(D(force_right,:))) - offset_R) * force_scale_R;
    end
    fS = fL + fR;

    buf_L = [buf_L(2:end), fL];
    buf_R = [buf_R(2:end), fR];
    buf_S = [buf_S(2:end), fS];
    set(hl,'YData',buf_L);
    set(hr,'YData',buf_R);
    set(hs,'YData',buf_S);

    idx_end = min(col+blockSamples-1, mvc_n);
    len     = idx_end - col + 1;

    if strcmp(force_dir,'push')
        mvc_force_L(col:idx_end) = (double(D(force_left, 1:len))  - offset_L) * force_scale_L;
        mvc_force_R(col:idx_end) = (double(D(force_right,1:len))  - offset_R) * force_scale_R;
    else
        mvc_force_L(col:idx_end) = -(double(D(force_left, 1:len))  - offset_L) * force_scale_L;
        mvc_force_R(col:idx_end) = -(double(D(force_right,1:len))  - offset_R) * force_scale_R;
    end
    mvc_force_raw(col:idx_end) = mvc_force_L(col:idx_end) + mvc_force_R(col:idx_end);

    mvc_emg(:, col:idx_end) = double(D(emg_channels, 1:len)) * ConvFact;

    col = col + len;
    drawnow limitrate;
end

actual_duration = toc(mvc_start_time);
fprintf('--> RECORDING FINISHED: Collected %d samples in %.3f seconds (Target: %.3f s)\n', ...
    mvc_n, actual_duration, mvc_duration);

title(ax, 'MVC complete');
set(ax, 'Color', colours.waitingRoom);

fprintf('mvc_force_raw range: %.4f to %.4f N\n', min(mvc_force_raw), max(mvc_force_raw));
fprintf('mvc_force_L range: %.4f to %.4f N\n', min(mvc_force_L), max(mvc_force_L));
fprintf('mvc_force_R range: %.4f to %.4f N\n', min(mvc_force_R), max(mvc_force_R));

mvc_value = max(mvc_force_raw);
if strcmpi(task_leg, 'bilateral')
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

fprintf('MVC — L: %.3f  R: %.3f  Sum: %.3f (N)\n', mvc_value_L, mvc_value_R, mvc_value);

yline(mvc_value,  'k--', sprintf('Peak: %.3f', mvc_value), ...
    'LabelVerticalAlignment','top', 'LabelHorizontalAlignment','left');
yline(mvc_value_L,'r--', sprintf('Peak: %.3f', mvc_value_L), ...
    'LabelVerticalAlignment','bottom', 'LabelHorizontalAlignment','center');
yline(mvc_value_R,'b--', sprintf('Peak: %.3f', mvc_value_R), ...
    'LabelVerticalAlignment','top', 'LabelHorizontalAlignment','right');

title(sprintf('Peaks: %.3f %.3f %.3f N', mvc_value, mvc_value_L, mvc_value_R));
xlabel('Samples'); ylabel('Force (N)');
legend({'Sum','Left','Right'},'Location','bestoutside');

end

%% run_task
function [task_force, task_emg, task_extra] = run_task(tcpSocket, ax, hl, hr, hs, ...
    buf_L, buf_R, buf_S, PacketSize1Block, nPeriods500, Ptr_IN, nGrids, mult, accMult, blockSamples, bytesPerBlock, ...
    force_left, force_right, offset_L, offset_R, ...
    force_dir, sampFreq, mvc_value, task_leg, task_shape, task_level, mcon_cycles,...
    trap_ramp_s, trap_hold_s, lead_in_s, emg_channels, n_emg, ConvFact,...
    force_scale_L, force_scale_R, multi_trap_rest_s, colours, brim_height, brim_length, brim_ramp_to_peak, use_constant_slope, extra_channels)

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

    % case 'multi_trap'
    %     rest_steps  = round(multi_trap_rest_s * updates_per_sec);
    %     total_steps = round(30 * 60 * updates_per_sec);
    %     single_trap = [linspace(0,task_level,ramp_steps), ...
    %         task_level*ones(1,hold_steps), ...
    %         linspace(task_level,0,ramp_steps), ...
    %         zeros(1,rest_steps)];
    %     n_reps = ceil(total_steps / numel(single_trap));
    %     target_trace = [zeros(1,lead_steps), repmat(single_trap,1,n_reps)];
    %     target_trace = target_trace(1 : lead_steps + total_steps);

    otherwise
        error('Unknown task_shape: %s', task_shape);
end

%do_record  = ~strcmp(task_shape, 'multi_trap');
n_steps    = numel(target_trace);
n_samples  = n_steps * blockSamples;
t_axis_upd = (0:n_steps-1) * (blockSamples / sampFreq);

task_force = zeros(7, n_samples);
task_emg   = zeros(n_emg, n_samples);
n_extra = numel(extra_channels);
task_extra = zeros(n_extra, n_samples);

col_str = colours.(task_leg);

task_fig = figure('Color',colours.bg,'Name',sprintf('Task — %s %s %.0f%%MVC', ...
    task_leg, task_shape, task_level*100), ...
    'Units','normalized','OuterPosition',[0 0 1 1]);
guidata(task_fig, struct('pressed',''));
set(task_fig,'KeyPressFcn',@(src,e) guidata(src,setfield(guidata(src),'pressed',e.Key)));

ax_t = axes(task_fig);
hold(ax_t,'on');
set(ax_t, 'Color',colours.bg, 'XColor',colours.text, 'YColor',colours.text, ...
    'XGrid','on','YGrid','on','GridColor',colours.grid);

plot(ax_t, t_axis_upd, target_trace, 'Color',colours.target, 'LineWidth', 5);
user_line = plot(ax_t, t_axis_upd, NaN(1,n_steps), 'Color', col_str, 'LineWidth', 3);
ball = plot(ax_t, t_axis_upd(1), 0, 'o', 'MarkerSize', colours.ballSize, 'LineStyle', 'none');
set(ball, 'MarkerFaceColor','none', 'MarkerEdgeColor',colours.(task_leg), 'LineWidth', 2);
cursor_line = xline(ax_t, 0, colours.cursor, 'LineWidth', 1.5);

ylim(ax_t, [-task_level*0.2, task_level*1.6]);
xlim(ax_t, [t_axis_upd(1), t_axis_upd(end)]);
xlabel(ax_t, 'Time (s)', 'Color',colours.text);
ylabel(ax_t, 'Force (%MVC)', 'Color',colours.text);
yticks_vals = linspace(0, task_level, 5);
yticks(ax_t, yticks_vals);
yticklabels(ax_t, arrayfun(@(v) sprintf('%d%%',round(v*100)), yticks_vals,'UniformOutput',false));

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
%flush(tcpSocket);
clear_tcp_backlog(tcpSocket, bytesPerBlock);

user_force_hist = NaN(1, n_steps);

render_interval = 0.075;
t_render = tic;
col = 1;

for k = 1:n_steps

    while tcpSocket.BytesAvailable < bytesPerBlock, pause(0.001); end
    D = readBlockNovecento(tcpSocket, PacketSize1Block, nPeriods500, Ptr_IN, nGrids, mult, accMult, blockSamples);

    if strcmp(force_dir,'push')
        fL = (mean(double(D(force_left, :))) - offset_L) * force_scale_L;
        fR = (mean(double(D(force_right,:))) - offset_R) * force_scale_R;
    else
        fL = -(mean(double(D(force_left, :))) - offset_L) * force_scale_L;
        fR = -(mean(double(D(force_right,:))) - offset_R) * force_scale_R;
    end
    fS = fL + fR;
    dL = fL/mvc_value;  dR = fR/mvc_value;  dS = fS/mvc_value;
    switch task_leg
        case 'left',      disp_val = dL;
        case 'right',     disp_val = dR;
        case 'bilateral', disp_val = dS;
    end

    user_force_hist(k) = disp_val;
    set(user_line,   'YData', user_force_hist);
    set(ball, 'XData', t_axis_upd(k), 'YData', disp_val);
    set(cursor_line, 'Value', t_axis_upd(k));

    %if do_record
    idx_end = min(col+blockSamples-1, n_samples);
    len     = idx_end - col + 1;
    if strcmp(force_dir,'push')
        task_force(1,col:idx_end) = (double(D(force_left, 1:len)) - offset_L) * force_scale_L;
        task_force(2,col:idx_end) = (double(D(force_right,1:len)) - offset_R) * force_scale_R;
    else
        task_force(1,col:idx_end) = -(double(D(force_left, 1:len)) - offset_L) * force_scale_L;
        task_force(2,col:idx_end) = -(double(D(force_right,1:len)) - offset_R) * force_scale_R;
    end
    task_force(3,col:idx_end) = task_force(1,col:idx_end) + task_force(2,col:idx_end);
    task_force(4,col:idx_end) = task_force(1,col:idx_end) / mvc_value;
    task_force(5,col:idx_end) = task_force(2,col:idx_end) / mvc_value;
    task_force(6,col:idx_end) = task_force(3,col:idx_end) / mvc_value;
    if k < n_steps
        task_force(7,col:idx_end) = linspace(target_trace(k), target_trace(k+1), len);
    else
        task_force(7,col:idx_end) = target_trace(k);
    end
    task_emg(:,col:idx_end) = double(D(emg_channels,1:len)) * ConvFact;
    task_extra(:,col:idx_end) = D(extra_channels,1:len);
    col = col + len;
    %end

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
close(task_fig);
task_force = task_force(:, 1:col-1);
task_emg   = task_emg(:,   1:col-1);
task_extra = task_extra(:,1:col-1);
disp('Task complete.');

end

%% save_task
function save_task(datapath, task_emg, task_force, task_extra, mvc_value, sampFreq, n_emg, subject, force_dir, task_shape,task_level,task_leg, study, muscle, condition, extra_channels)

signal.data          = task_emg;
signal.fsamp         = sampFreq;
signal.nChan         = n_emg;
signal.ngrid         = n_emg/64;
signal.gridname      = repmat({'BIO64HD'}, 1, n_emg/64);
signal.muscle        = arrayfun(@(i) sprintf('Muscle%d',i), 1:n_emg/64, 'UniformOutput', false);

signal.auxiliary      = task_force;
signal.extra          = task_extra;
signal.extra_channels = extra_channels;

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

task_level_pct = round(task_level * 100);

if strcmpi(task_leg,'bilateral'); lenLeg = 5; else; lenLeg = 4; end

fname = sprintf('%s_%s_%s_%s_%s_%s_%d_%s_%s.mat', ...
    study, subject, muscle, task_leg(1:lenLeg), task_shape, ...
    force_dir, task_level_pct, condition, datestr(now,'yyyymmdd_HHMMSS'));

save(fullfile(datapath, fname), 'signal', 'mvc_value', ...
    'task_leg', 'task_shape', 'Force', 'Target', '-v7.3');

disp('Task saved.');
end

%% run_multi_target
function [task_force, task_emg] = run_multi_target(tcpSocket, ax, hl, hr, hs, ...
    buf_L, buf_R, buf_S, PacketSize1Block, nPeriods500, Ptr_IN, nGrids, mult, accMult, blockSamples, bytesPerBlock, ...
    force_left, force_right, offset_L, offset_R, ...
    force_dir, sampFreq, mvcLeft, mvcRight, cfg, mcon_cycles, ...
    trap_ramp_s, trap_hold_s, lead_in_s, emg_channels, n_emg, ConvFact, ...
    force_scale_L, force_scale_R, colours , brim_height, brim_length, brim_ramp_to_peak, use_constant_slope)

updates_per_sec = sampFreq / blockSamples;

n_legs = numel(cfg);
target_traces = cell(n_legs, 1);
for i = 1:n_legs
    lv = cfg(i).level;
    ramp_steps = round(trap_ramp_s * updates_per_sec);
    hold_steps = round(trap_hold_s * updates_per_sec);
    lead_steps = round(lead_in_s   * updates_per_sec);
    brim_level = lv * brim_height;

    if use_constant_slope
        slope_rate      = lv / trap_ramp_s;
        ramp1_steps     = round((brim_level / slope_rate) * updates_per_sec);
        ramp2_steps     = round(((lv - brim_level) / slope_rate) * updates_per_sec);
        ramp_down_steps = round((lv / slope_rate) * updates_per_sec);
    else
        ramp1_steps     = ramp_steps;
        ramp2_steps     = round(brim_ramp_to_peak * updates_per_sec);
        ramp_down_steps = ramp_steps;
    end

    switch cfg(i).shape
        case 'trap'
            target_traces{i} = [zeros(1,lead_steps), ...
                linspace(0,lv,ramp_steps), ...
                lv*ones(1,hold_steps), ...
                linspace(lv,0,ramp_steps), ...
                zeros(1,lead_steps)];
        case 'sombrero'
            brim_steps = round(brim_length * updates_per_sec);
            target_traces{i} = [zeros(1,lead_steps), ...
                linspace(0,brim_level,ramp1_steps), ...
                brim_level*ones(1,brim_steps), ...
                linspace(brim_level,lv,ramp2_steps), ...
                lv*ones(1,hold_steps), ...
                linspace(lv,brim_level,ramp2_steps), ...
                brim_level*ones(1,brim_steps), ...
                linspace(brim_level,0,ramp1_steps), ...
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

n_steps = min(cellfun(@numel, target_traces));
for i = 1:n_legs
    target_traces{i} = target_traces{i}(1:n_steps);
end

n_samples  = n_steps * blockSamples;
t_axis_upd = (0:n_steps-1) * (blockSamples / sampFreq);

task_force = zeros(6, n_samples);
task_emg   = zeros(n_emg, n_samples);

mt_fig = figure('Color',colours.bg, 'Name','Multi-Target', ...
    'Units','normalized','OuterPosition',[0 0 1 1]);
guidata(mt_fig, struct('pressed',''));
set(mt_fig,'KeyPressFcn',@(src,e) guidata(src,setfield(guidata(src),'pressed',e.Key)));

ax_mt = gobjects(n_legs, 1);
user_lines   = gobjects(n_legs, 1);
cursor_lines = gobjects(n_legs, 1);
ball_lines   = gobjects(n_legs, 1);

for i = 1:n_legs
    ax_mt(i) = subplot(n_legs, 1, i, 'Parent', mt_fig);
    hold(ax_mt(i), 'on');
    set(ax_mt(i), 'Color',colours.bg,'XColor',colours.text,'YColor',colours.text, ...
        'XGrid','on','YGrid','on','GridColor',colours.grid);

    lv = cfg(i).level;
    leg = cfg(i).leg;
    col_str = colours.(leg);

    plot(ax_mt(i), t_axis_upd, target_traces{i}, 'Color',colours.target, 'LineWidth', 3);
    user_lines(i) = plot(ax_mt(i), t_axis_upd, NaN(1,n_steps), col_str, 'LineWidth', 2.5);
    ball_lines(i) = plot(ax_mt(i), t_axis_upd(1), 0, 'o', 'MarkerSize', colours.ballSize, 'LineStyle', 'none');
    switch cfg(i).leg
        case 'left',  set(ball_lines(i), 'MarkerFaceColor',colours.left,  'MarkerEdgeColor',colours.left);
        case 'right', set(ball_lines(i), 'MarkerFaceColor',colours.right, 'MarkerEdgeColor',colours.right);
    end
    cursor_lines(i) = xline(ax_mt(i), 0, 'w', 'LineWidth', 1.5);

    ylim(ax_mt(i), [-lv*0.2, lv*1.6]);
    xlim(ax_mt(i), [t_axis_upd(1), t_axis_upd(end)]);
    ylabel(ax_mt(i), 'Force (%MVC)', 'Color',colours.text);

    yticks_vals = linspace(0, lv, 5);
    yticks(ax_mt(i), yticks_vals);
    yticklabels(ax_mt(i), arrayfun(@(v) sprintf('%d%%', round(v*100)), yticks_vals, 'UniformOutput', false));

    title(ax_mt(i), sprintf('%s leg — %s @ %d%% MVC', ...
        upper(leg(1)), upper(cfg(i).shape), round(lv*100)), ...
        'Color',colours.text,'FontSize',13);
end
xlabel(ax_mt(end), 'Time (s)', 'Color',colours.text);

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
%flush(tcpSocket);
clear_tcp_backlog(tcpSocket, bytesPerBlock);

user_force_hist = NaN(n_legs, n_steps);

render_interval = 0.075;
t_render = tic;
col = 1;

for k = 1:n_steps

    while tcpSocket.BytesAvailable < bytesPerBlock, pause(0.001); end
    D = readBlockNovecento(tcpSocket, PacketSize1Block, nPeriods500, Ptr_IN, nGrids, mult, accMult, blockSamples);

    if strcmp(force_dir, 'push')
        fL = (mean(double(D(force_left, :))) - offset_L) * force_scale_L;
        fR = (mean(double(D(force_right,:))) - offset_R) * force_scale_R;
    else
        fL = -(mean(double(D(force_left, :))) - offset_L) * force_scale_L;
        fR = -(mean(double(D(force_right,:))) - offset_R) * force_scale_R;
    end

    dL = fL / mvcLeft;
    dR = fR / mvcRight;
    leg_vals = struct('left', dL, 'right', dR);

    for i = 1:n_legs
        user_force_hist(i, k) = leg_vals.(cfg(i).leg);
    end

    for i = 1:n_legs
        set(user_lines(i),   'YData', user_force_hist(i,:));
        set(cursor_lines(i), 'Value', t_axis_upd(k));
        set(ball_lines(i), 'XData', t_axis_upd(k), 'YData', user_force_hist(i,k));
    end

    idx_end = min(col + blockSamples - 1, n_samples);
    len     = idx_end - col + 1;
    if strcmp(force_dir, 'push')
        task_force(1, col:idx_end) = (double(D(force_left, 1:len))  - offset_L) * force_scale_L;
        task_force(2, col:idx_end) = (double(D(force_right,1:len))  - offset_R) * force_scale_R;
    else
        task_force(1, col:idx_end) = -(double(D(force_left, 1:len))  - offset_L) * force_scale_L;
        task_force(2, col:idx_end) = -(double(D(force_right,1:len))  - offset_R) * force_scale_R;
    end
    task_force(3, col:idx_end) = task_force(1, col:idx_end) / mvcLeft;
    task_force(4, col:idx_end) = task_force(2, col:idx_end) / mvcRight;
    if k < n_steps
        task_force(5, col:idx_end) = linspace(target_traces{1}(k), target_traces{1}(k+1), len);
        task_force(6, col:idx_end) = linspace(target_traces{2}(k), target_traces{2}(k+1), len);
    else
        task_force(5, col:idx_end) = target_traces{1}(k);
        task_force(6, col:idx_end) = target_traces{2}(k);
    end

    task_emg(:, col:idx_end) = double(D(emg_channels, 1:len)) * ConvFact;
    col = col + len;

    if toc(t_render) >= render_interval
        drawnow limitrate;
        t_render = tic;
    end

    if strcmp(guidata(mt_fig).pressed, 'q')
        break
    end
end

task_force = task_force(:, 1:col-1);
task_emg   = task_emg(:,   1:col-1);

for i = 1:n_legs
    title(ax_mt(i), sprintf('%s leg — Task complete.', upper(cfg(i).leg(1))), 'Color',colours.text,'FontSize',13);
end
drawnow;
disp('Multi-target task complete.');
close(mt_fig);

end

%% save_mvc
function save_mvc(datapath, mvc_emg, mvc_force_raw, mvc_force_L, mvc_force_R, mvc_value, sampFreq, n_emg, emg_channels, subject, force_dir,task_leg,study, muscle, condition)

signal_mvc.data          = mvc_emg;
signal_mvc.fsamp         = sampFreq;
signal_mvc.nChan         = n_emg;
signal_mvc.ngrid         = n_emg/64;
signal_mvc.gridname      = repmat({'BIO64HD'}, 1, n_emg/64);
signal_mvc.muscle        = arrayfun(@(i) sprintf('Muscle%d',i), 1:n_emg/64, 'UniformOutput', false);

signal_mvc.auxiliary     = [mvc_force_raw; mvc_force_L; mvc_force_R];
%signal_mvc.auxiliaryname = {'Force_Sum_N', 'Force_L_N', 'Force_R_N'};
signal_mvc.auxiliaryname = {'Force_Sum_raw', 'Force_L_raw', 'Force_R_raw'};
signal_mvc.target        = [];

if strcmpi(task_leg,'bilateral')
    Force = mvc_force_raw;
elseif strcmpi(task_leg,'left')
    Force = mvc_force_L;
elseif strcmpi(task_leg,'right')
    Force = mvc_force_R;
end

if strcmpi(task_leg,'bilateral'); lenLeg = 5; else; lenLeg = 4; end
fname = sprintf('mvc_%s_%s_%s_%s_%s_%s_%s.mat', ...
    study, subject, muscle, task_leg(1:lenLeg), ...
    force_dir, condition, datestr(now,'yyyymmdd_HHMMSS'));

save(fullfile(datapath, fname), 'signal_mvc', 'mvc_value', ...
    'task_leg', 'emg_channels', 'Force', '-v7.3');

disp('MVC saved.');
end

%% readBlockNovecento
function D = readBlockNovecento(tcpSocket, PacketSize1Block, nPeriods500, Ptr_IN, nGrids, mult, accMult, blockSamples)

% Calculate exact block byte size locally (int16 = 2 bytes)
bytesPerBlock = PacketSize1Block * nPeriods500 * 2;

% 1. DRAIN SURPLUS BACKLOG: If buffer is > 2 blocks behind, step to the latest complete block boundary
avail = tcpSocket.NumBytesAvailable;
if avail > bytesPerBlock * 2
    excess = floor(avail / bytesPerBlock) - 1;
    read(tcpSocket, excess * bytesPerBlock, 'uint8');
end

% 2. READ EXACT BLOCK
Temp = read(tcpSocket, PacketSize1Block*nPeriods500, 'int16')';
Temp = reshape(Temp, PacketSize1Block, nPeriods500);

% 3. EXTRACT PANELS (existing logic)
D_grids = zeros(nGrids*70, blockSamples);
for g = 1:nGrids
    rows  = Ptr_IN(g):Ptr_IN(g+1)-1;
    slice = Temp(rows, :);
    flat  = reshape(slice, 1, []);
    D_grids((g-1)*70+1 : g*70, :) = reshape(flat, 70, blockSamples);
end

auxRows  = Ptr_IN(11) : Ptr_IN(11)+16*mult-1;
D_aux    = reshape(reshape(Temp(auxRows, :), 1, []), 16, blockSamples);

accRowsStart = auxRows(end) + 1;
accRows      = accRowsStart : accRowsStart + 128 - 1;
accSlice     = int16(reshape(Temp(accRows, :), 1, []));
accInt32     = typecast(accSlice, 'int32');
accChunk     = reshape(accInt32, 4, accMult*nPeriods500);
decim        = accMult / mult;
D_acc        = accChunk(:, decim:decim:end);

D = [D_grids; D_aux; D_acc];
end

%% clear tcp backlog
function clear_tcp_backlog(tcpSocket, bytesPerBlock)
    % Reads out full integer blocks, leaving only clean block alignments
    avail = tcpSocket.NumBytesAvailable;
    nBlocks = floor(avail / bytesPerBlock);
    if nBlocks > 0
        read(tcpSocket, nBlocks * bytesPerBlock, 'uint8');
    end
end

%% run_multi_trap_fatigue — one trap at a time, loops until 'q', always records
function [task_force, task_emg, task_extra] = run_multi_trap_fatigue(tcpSocket, ...
    PacketSize1Block, nPeriods500, Ptr_IN, nGrids, mult, accMult, blockSamples, bytesPerBlock, ...
    force_left, force_right, offset_L, offset_R, ...
    force_dir, sampFreq, mvc_value, task_leg, task_level, ...
    trap_ramp_s, trap_hold_s, lead_in_s, multi_trap_rest_s, ...
    emg_channels, n_emg, ConvFact, force_scale_L, force_scale_R, colours, extra_channels)

updates_per_sec = sampFreq / blockSamples;
ramp_steps = round(trap_ramp_s * updates_per_sec);
hold_steps = round(trap_hold_s * updates_per_sec);
lead_steps = round(lead_in_s   * updates_per_sec);
rest_steps = round(multi_trap_rest_s * updates_per_sec);

single_trap = [zeros(1,lead_steps), linspace(0,task_level,ramp_steps), task_level*ones(1,hold_steps), ...
               linspace(task_level,0,ramp_steps), zeros(1,rest_steps)];
n_steps_rep = numel(single_trap);
t_axis_rep  = (0:n_steps_rep-1) * (blockSamples/sampFreq);

col_str = colours.(task_leg);

task_fig = figure('Color',colours.bg,'Name','Multi-trap fatigue', ...
    'Units','normalized','OuterPosition',[0 0 1 1]);
guidata(task_fig, struct('pressed',''));
set(task_fig,'KeyPressFcn',@(src,e) guidata(src,setfield(guidata(src),'pressed',e.Key)));

ax_t = axes(task_fig); hold(ax_t,'on');
set(ax_t,'Color',colours.bg,'XColor',colours.text,'YColor',colours.text, ...
    'XGrid','on','YGrid','on','GridColor',colours.grid);
plot(ax_t, t_axis_rep, single_trap, 'Color',colours.target, 'LineWidth',5);
user_line = plot(ax_t, t_axis_rep, NaN(1,n_steps_rep), 'Color',col_str,'LineWidth',3);
ball = plot(ax_t, t_axis_rep(1), 0, 'o','MarkerSize',colours.ballSize,'LineStyle','none');
set(ball,'MarkerFaceColor','none','MarkerEdgeColor',colours.(task_leg),'LineWidth',2);
cursor_line = xline(ax_t, 0, colours.cursor, 'LineWidth',1.5);
ylim(ax_t, [-task_level*0.2, task_level*1.6]);
xlim(ax_t, [t_axis_rep(1), t_axis_rep(end)]);
xlabel(ax_t,'Time (s)','Color',colours.text); ylabel(ax_t,'Force (%MVC)','Color',colours.text);

for ct = 3:-1:1
    title(ax_t, sprintf('Fatigue protocol — starting in %d... (Q to stop anytime)', ct), ...
        'Color',colours.text,'FontSize',13);
    drawnow; pause(1);
end
clear_tcp_backlog(tcpSocket, bytesPerBlock);

rep_force = {}; rep_emg = {}; rep_extra = {};
rep_num = 0;
render_interval = 0.075; t_render = tic;

quit_flag = false;
while ~quit_flag
    rep_num = rep_num + 1;
    set(user_line,'YData',NaN(1,n_steps_rep));
    title(ax_t, sprintf('Rep %d — %s @ %d%%MVC — FOLLOW THE LINE (Q to stop)', ...
        rep_num, upper(task_leg), round(task_level*100)), 'Color',colours.text,'FontSize',13);

    n_samples_rep = n_steps_rep * blockSamples;
    rf = zeros(7, n_samples_rep);
    re = zeros(n_emg, n_samples_rep);
    rx = zeros(numel(extra_channels), n_samples_rep);
    user_hist = NaN(1, n_steps_rep);
    col = 1;

    for k = 1:n_steps_rep
        while tcpSocket.BytesAvailable < bytesPerBlock, pause(0.001); end
        D = readBlockNovecento(tcpSocket, PacketSize1Block, nPeriods500, Ptr_IN, nGrids, mult, accMult, blockSamples);

        if strcmp(force_dir,'push')
            fL = -(mean(double(D(force_left,:))) - offset_L) * force_scale_L;
            fR =  (mean(double(D(force_right,:))) - offset_R) * force_scale_R;
        else
            fL =  (mean(double(D(force_left,:))) - offset_L) * force_scale_L;
            fR = -(mean(double(D(force_right,:))) - offset_R) * force_scale_R;
        end
        fS = fL + fR;
        dL = fL/mvc_value; dR = fR/mvc_value; dS = fS/mvc_value;
        switch task_leg
            case 'left',      disp_val = dL;
            case 'right',     disp_val = dR;
            case 'bilateral', disp_val = dS;
        end

        user_hist(k) = disp_val;
        set(user_line,'YData',user_hist);
        set(ball,'XData',t_axis_rep(k),'YData',disp_val);
        set(cursor_line,'Value',t_axis_rep(k));

        idx_end = min(col+blockSamples-1, n_samples_rep);
        len = idx_end - col + 1;
        if strcmp(force_dir,'push')
            rf(1,col:idx_end) = (double(D(force_left,1:len)) - offset_L) * force_scale_L;
            rf(2,col:idx_end) = (double(D(force_right,1:len)) - offset_R) * force_scale_R;
        else
            rf(1,col:idx_end) = -(double(D(force_left,1:len)) - offset_L) * force_scale_L;
            rf(2,col:idx_end) = -(double(D(force_right,1:len)) - offset_R) * force_scale_R;
        end
        rf(3,col:idx_end) = rf(1,col:idx_end) + rf(2,col:idx_end);
        rf(4,col:idx_end) = rf(1,col:idx_end) / mvc_value;
        rf(5,col:idx_end) = rf(2,col:idx_end) / mvc_value;
        rf(6,col:idx_end) = rf(3,col:idx_end) / mvc_value;
        if k < n_steps_rep
            rf(7,col:idx_end) = linspace(single_trap(k), single_trap(k+1), len);
        else
            rf(7,col:idx_end) = single_trap(k);
        end
        re(:,col:idx_end) = double(D(emg_channels,1:len)) * ConvFact;
        rx(:,col:idx_end) = D(extra_channels,1:len);
        col = col + len;

        if toc(t_render) >= render_interval
            drawnow limitrate; t_render = tic;
        end

        if strcmp(guidata(task_fig).pressed, 'q')
            quit_flag = true; break
        end
    end

    rep_force{end+1} = rf(:,1:col-1);
    rep_emg{end+1}   = re(:,1:col-1);
    rep_extra{end+1} = rx(:,1:col-1);
end

task_force = cat(2, rep_force{:});
task_emg   = cat(2, rep_emg{:});
task_extra = cat(2, rep_extra{:});

title(ax_t, sprintf('Fatigue protocol complete — %d reps.', rep_num), 'Color',colours.text,'FontSize',13);
drawnow;
close(task_fig);
fprintf('Multi-trap fatigue: %d reps recorded.\n', rep_num);
end