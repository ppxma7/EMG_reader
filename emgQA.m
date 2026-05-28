%% emg_qa_viewer.m
% Fast live EMG trace viewer for pre-experiment QA.
% Run this BEFORE experiment_muovi4.m to check electrode contact quality.
% 128 EMG channels split across two subplots (64 per Muovi+).
% Y-axis in real mV, autoscaled. Press Q to quit.

close all; clear; clc;

% =========================================================================
% CONFIG
% =========================================================================
TCPPort      = 54320;
sampFreq     = 2000;
blockSamples = 50;

TotNumChan    = 146;
TotNumByte    = 292;
bytesPerBlock = TotNumByte * blockSamples;

emg_channels  = [1:64, 71:134];
n_emg         = numel(emg_channels);
ConvFact      = 0.000286;   % raw ADC → mV

N_display = round(sampFreq * 3 / blockSamples) * blockSamples;  % 3s window
t_axis    = (0:N_display-1) * (1/sampFreq);

% =========================================================================
% CONFIG STRING — 2x Muovi+ in slots 5 and 6
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
fwrite(tcpSocket, ConfString(1:4), 'uint8');
fprintf('Connected. Streaming EMG QA...\n');
fprintf('  Q = quit\n\n');

pause(0.5);
flush(tcpSocket);

% =========================================================================
% FIGURE
% =========================================================================
emg_buf = zeros(n_emg, N_display);

fig = figure('Color','k', 'Name','EMG QA Viewer — Q to quit', ...
    'MenuBar','none', 'ToolBar','none', 'Position',[50 50 1200 800]);

ax(1) = subplot(1,2,1); hold(ax(1),'on');
ax(2) = subplot(1,2,2); hold(ax(2),'on');

for a = 1:2
    set(ax(a), 'Color','k', 'XColor','w', 'YColor','w');
    xlabel(ax(a), 'Time (s)', 'Color','w');
    ylabel(ax(a), 'mV', 'Color','w');
    xlim(ax(a), [0, t_axis(end)]);
    ylim(ax(a), [-0.5 0.5]);
end
title(ax(1), 'Muovi 1 (ch 1-64)',   'Color','w');
title(ax(2), 'Muovi 2 (ch 65-128)', 'Color','w');

h_lines = gobjects(n_emg, 1);
for k = 1:64
    h_lines(k)    = plot(ax(1), t_axis, emg_buf(k,:),    'Color',[0.4 0.8 0.4], 'LineWidth', 0.3);
    h_lines(k+64) = plot(ax(2), t_axis, emg_buf(k+64,:), 'Color',[0.4 0.8 0.4], 'LineWidth', 0.3);
end

guidata(fig, struct('pressed',''));
set(fig, 'KeyPressFcn', @(src,e) guidata(src, setfield(guidata(src),'pressed',e.Key)));

% another figure
% After the existing figure setup, add:
grid_fig = figure('Color','k','Name','EMG Grid RMS','Position',[100 650 600 350]);
ax_g(1) = subplot(1,2,1);
ax_g(2) = subplot(1,2,2);

% initialise with zeros

ElChannelMap = [52 39 26 13 1; ...
                53 40 27 14 1; ...
                54 41 28 15 2; ...
                55 42 29 16 3; ...
                56 43 30 17 4; ...
                57 44 31 18 5; ...
                58 45 32 19 6; ...
                59 46 33 20 7; ...
                60 47 34 21 8; ...
                61 48 35 22 9; ...
                62 49 36 23 10; ...
                63 50 37 24 11; ...
                64 51 38 25 12];

grid_data = zeros(13,5);


h_img(1) = imagesc(ax_g(1), grid_data);
h_img(2) = imagesc(ax_g(2), grid_data);

for a = 1:2
    set(ax_g(a),'Color','k','XColor','w','YColor','w');
    colormap(ax_g(a), 'hot');
    colorbar(ax_g(a));
    clim(ax_g(a), [0 0.5]);   % mV RMS range — adjust as needed
    axis(ax_g(a), 'equal', 'tight');
    xlabel(ax_g(a),'Column','Color','w');
    ylabel(ax_g(a),'Row','Color','w');
end
title(ax_g(1),'Muovi 1 RMS (mV)','Color','w');
title(ax_g(2),'Muovi 2 RMS (mV)','Color','w');


% =========================================================================
% LIVE LOOP
% =========================================================================
while ishandle(fig) && ~strcmp(guidata(fig).pressed, 'q')

    gd = guidata(fig);
    gd.pressed = '';
    guidata(fig, gd);

    while ishandle(fig) && tcpSocket.BytesAvailable < bytesPerBlock
        pause(0.001);
    end
    if ~ishandle(fig), break; end

    % drain backlog if behind
    if tcpSocket.BytesAvailable > bytesPerBlock * 5
        flush(tcpSocket);
        continue;
    end

    Temp = fread(tcpSocket, [TotNumByte, blockSamples], 'uint8');
    Temp = reshape(Temp, TotNumByte, blockSamples);
    D    = double(Temp(1:2:end,:))*256 + double(Temp(2:2:end,:));
    idx  = D >= 32768;
    D(idx) = D(idx) - 65536;

    blk = double(D(emg_channels, :)) * ConvFact;
    emg_buf = [emg_buf(:, blockSamples+1:end), blk];

    for k = 1:64
        set(h_lines(k),    'YData', emg_buf(k,:));
        set(h_lines(k+64), 'YData', emg_buf(k+64,:));
    end

    % autoscale both panels to same range
    mx = max(abs(emg_buf(:)));
    mx = max(mx, 0.05);
    ylim(ax(1), [-mx mx]);
    ylim(ax(2), [-mx mx]);

    rms1 = mean(sqrt(mean(emg_buf(1:64,:).^2,   2)));
    rms2 = mean(sqrt(mean(emg_buf(65:128,:).^2, 2)));
    title(ax(1), sprintf('Muovi 1 (ch 1-64)   | mean RMS = %.3f mV', rms1), 'Color','w');
    title(ax(2), sprintf('Muovi 2 (ch 65-128) | mean RMS = %.3f mV', rms2), 'Color','w');

    % ← ADD HERE: grid heatmap
    ch_rms = sqrt(mean(blk.^2, 2));   % 128×1 mV, current block only
    grid1 = zeros(13,5);
    grid2 = zeros(13,5);
    for row = 1:13
        for col = 1:5
            ch = ElChannelMap(row,col);
            if ch >= 1 && ch <= 64
                grid1(row,col) = ch_rms(ch);
                grid2(row,col) = ch_rms(ch+64);
            end
        end
    end
    set(h_img(1), 'CData', grid1);
    set(h_img(2), 'CData', grid2);

    drawnow limitrate;
end

% =========================================================================
% STOP
% =========================================================================
fwrite(tcpSocket, [0; CRC8(0,1)], 'uint8');
clear tcpSocket;
if ishandle(fig), close(fig); end
disp('EMG QA viewer stopped.');