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

    drawnow limitrate;
end

% =========================================================================
% STOP
% =========================================================================
fwrite(tcpSocket, [0; CRC8(0,1)], 'uint8');
clear tcpSocket;
if ishandle(fig), close(fig); end
disp('EMG QA viewer stopped.');