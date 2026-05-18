%% experimentMuoviDraft.m
% Live force plot + EMG recording via SyncStation
% Based on minimalReadTest — uint8 read, no extras

close all; clear all; clc;

force_dir = 'push';   % 'push' = pushing activates force, 'pull' = pulling activates

% =========================================================================
% USER OPTIONS
% =========================================================================
TCPPort      = 54320;
sampFreq     = 2000;
blockSamples = 200;

TotNumChan    = 146;
TotNumByte    = 292;
bytesPerBlock = TotNumByte * blockSamples;

force_left  = 141;
force_right = 142;
force_sum   = 143;

n_emg         = 128;          % 64 per Muovi+
emg_channels  = [1:64, 71:134];
datapath      = 'C:\Users\masgh\data\emgReaderData\';


emg_ylim_std = [0 500];   % adjust to taste
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
    baseline_buf(3, col:col+len-1) = D(force_sum,   1:len);
    col = col + len;
end
offset_L = mean(baseline_buf(1,:));
offset_R = mean(baseline_buf(2,:));
offset_S = mean(baseline_buf(3,:));
fprintf('Offsets — L:%.0f  R:%.0f  Sum:%.0f\n', offset_L, offset_R, offset_S);

% =========================================================================
% FIGURES
% =========================================================================
N = round(10 * sampFreq / blockSamples);   % 10s rolling window
buf_L = zeros(1,N);
buf_R = zeros(1,N);
buf_S = zeros(1,N);

force_fig = figure('Color','w','Name','Force');
ax = axes(force_fig); hold(ax,'on');
hl = plot(ax, 1:N, buf_L, 'r', 'LineWidth',1.5);
hr = plot(ax, 1:N, buf_R, 'b', 'LineWidth',1.5);
hs = plot(ax, 1:N, buf_S, 'k', 'LineWidth',2);
legend(ax,{'Left','Right','Sum'},'Location','northwest');
title(ax,'Force — press Q to stop and save EMG');
xlabel(ax,'Updates'); ylabel(ax,'Force (offset-corrected ADC)');
ax.YLimMode = 'auto';
xlim(ax,[1 N]);

guidata(force_fig, struct('pressed',''));
set(force_fig,'KeyPressFcn',@(src,e) guidata(src,setfield(guidata(src),'pressed',e.Key)));


% FIGURE: EMG ACTIVITY
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
% STORAGE
% =========================================================================
emg_data = zeros(n_emg, 0);   % grows each block
force_data = zeros(3, max_record_s * sampFreq);
% =========================================================================
% MAIN LOOP
% =========================================================================
disp('Running. Press Q to stop.');

while ~strcmp(guidata(force_fig).pressed, 'q')

    while tcpSocket.NumBytesAvailable < bytesPerBlock, pause(0.001); end
    D = readBlock(tcpSocket, TotNumByte, blockSamples);

    % Force
    if strcmp(force_dir, 'push')
        fL = -(mean(double(D(force_left, :))) - offset_L);
        fR = -(mean(double(D(force_right,:))) - offset_R);

    else  % pull
        fL =  mean(double(D(force_left, :))) - offset_L;
        fR =  mean(double(D(force_right, :))) - offset_L;

    end

    fS = -(mean(double(D(force_sum,  :))) - offset_S);

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

    % EMG
    emg_data = [emg_data, double(D(emg_channels,:))];

    emg_block = double(D(emg_channels,:));
    emg_std_buf = std(double(emg_block), 0, 2);
    set(emg_bar, 'YData', emg_std_buf);

    if strcmp(guidata(force_fig).pressed, 'm')
        guidata(force_fig, setfield(guidata(force_fig),'pressed',''));
        mvc_duration = 5;   % seconds

        % choose which channel to use for MVC based on task
        [mvc_value, buf_L, buf_R, buf_S] = run_MVC(tcpSocket, ax, hl, hr, hs, ...
            buf_L, buf_R, buf_S, TotNumByte, blockSamples, bytesPerBlock, ...
            force_left, force_right, force_sum, offset_L, offset_R, offset_S, ...
            force_dir, sampFreq, mvc_duration);
        

        if mvc_value > 0
            ylabel(ax, 'Force (MVC fraction)');
        end
    end















    drawnow limitrate;



end

% =========================================================================
% STOP & SAVE
% =========================================================================
write(tcpSocket, [0; CRC8(0,1)], 'uint8');
clear tcpSocket;

fname = fullfile(datapath, sprintf('emg_%s.mat', datestr(now,'yyyymmdd_HHMMSS')));
%save(fname, 'emg_data', 'sampFreq', 'emg_channels');

save(fname, 'emg_data', 'sampFreq', 'emg_channels', 'mvc_value');

fprintf('EMG saved: %s\n', fname);
close all;



%%% ---------------------------------------------
%%% FUNCTIONS
%%% ---------------------------------------------

%% readblock
function data = readBlock(t, TotNumByte, blockSamples)
Temp = fread(t, [TotNumByte, blockSamples], 'uint8');
Temp = reshape(Temp, TotNumByte, blockSamples);
D    = Temp(1:2:end,:)*256 + Temp(2:2:end,:);
idx  = D >= 32768;
D(idx) = D(idx) - 65536;
data = D;
end

%% runMVC
function [mvc_value, buf_L, buf_R, buf_S] = run_MVC(tcpSocket, ax, hl, hr, hs, ...
    buf_L, buf_R, buf_S, TotNumByte, blockSamples, bytesPerBlock, ...
    force_left, force_right, force_sum, offset_L, offset_R, offset_S, ...
    force_dir, sampFreq, mvc_duration)

for ct = 3:-1:1
    title(ax, sprintf('GET READY... %d', ct));
    drawnow; pause(1);
end
title(ax, '*** PUSH NOW ***'); drawnow;

flush(tcpSocket);
fread(tcpSocket, [TotNumByte, blockSamples], 'uint8');   % flush

mvc_n         = mvc_duration * sampFreq;
mvc_force_raw = zeros(1, mvc_n);
col           = 1;
t_start       = tic;

while col <= mvc_n
    elapsed   = toc(t_start);
    remaining = max(0, mvc_duration - elapsed);
    title(ax, sprintf('PUSH! %d', ceil(remaining)));

    while tcpSocket.BytesAvailable < bytesPerBlock, pause(0.001); end
    D = readBlock(tcpSocket, TotNumByte, blockSamples);

    if strcmp(force_dir,'push')
        fL = -(mean(double(D(force_left, :))) - offset_L);
        fR = -(mean(double(D(force_right,:))) - offset_R);
    else
        fL =  (mean(double(D(force_left, :))) - offset_L);
        fR =  (mean(double(D(force_right,:))) - offset_R);
    end
    fS = -(mean(double(D(force_sum,:))) - offset_S);

    buf_L = [buf_L(2:end), fL];
    buf_R = [buf_R(2:end), fR];
    buf_S = [buf_S(2:end), fS];
    set(hl,'YData',buf_L);
    set(hr,'YData',buf_R);
    set(hs,'YData',buf_S);

    f_mvc = fS;   % or fL/fR depending on mvc_mode — using sum for now
    idx_end = min(col+blockSamples-1, mvc_n);
    len     = idx_end - col + 1;
    mvc_force_raw(col:idx_end) = f_mvc;
    col = col + len;
    drawnow limitrate;
end

title(ax, 'MVC complete — reviewing...');
mvc_value = max(mvc_force_raw);

mf = figure;
plot(mvc_force_raw, 'b', 'LineWidth', 1.5); hold on;
yline(mvc_value, 'r--', sprintf('Peak: %.0f', mvc_value));
title(sprintf('MVC trace — Peak: %.0f', mvc_value));
xlabel('Samples'); ylabel('Force (ADC)');

choice = questdlg(sprintf('MVC = %.0f — accept?', mvc_value), ...
    'MVC', 'Accept', 'Discard', 'Accept');
close(mf);

if isempty(choice) || strcmp(choice, 'Discard')
    mvc_value = 0;
    disp('MVC discarded.');
else
    fprintf('MVC accepted: %.0f\n', mvc_value);
    title(ax,'Force — press Q to stop and save EMG');
end

end

%%