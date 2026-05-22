%% emg_qa_viewer.m
% Fast live EMG trace viewer for pre-experiment QA.
% Run this BEFORE experiment_muovi4.m to check electrode contact quality.
% All 128 EMG channels displayed as stacked raw traces.
% Press Q to quit cleanly.
%
% Controls:
%   Q        — quit
%   +/-      — increase/decrease vertical spacing between traces

close all; clear; clc;

% =========================================================================
% CONFIG — match experiment_muovi4.m settings
% =========================================================================
TCPPort      = 54320;
sampFreq     = 2000;
blockSamples = 50;

TotNumChan    = 146;
TotNumByte    = 292;
bytesPerBlock = TotNumByte * blockSamples;

emg_channels  = [1:64, 71:134];   % 128 EMG channels (skip AUX in each Muovi+)
n_emg         = numel(emg_channels);
ConvFact      = 0.000286;          % raw ADC → mV

% Display options
emg_offset = 0.5;    % mV spacing between traces (adjust with +/-)
N_display  = round(sampFreq * 3 / blockSamples) * blockSamples;  % 3s rolling window

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
fprintf('  Q = quit | +/- = spacing\n\n');

% flush ~0.5s startup
pause(0.5);
flush(tcpSocket);

% =========================================================================
% FIGURE
% =========================================================================
emg_buf = zeros(n_emg, N_display);

fig = figure('Color','k', 'Name','EMG QA Viewer — Q to quit', ...
    'MenuBar','none', 'ToolBar','none', 'Position',[50 50 900 900]);
ax  = axes(fig, 'Color','k', 'XColor','w', 'YColor','w');
hold(ax, 'on');

colours = lines(n_emg);
h_lines = gobjects(n_emg, 1);
t_axis  = (0:N_display-1) * (1/sampFreq);
for k = 1:n_emg
    h_lines(k) = plot(ax, t_axis, emg_buf(k,:) + (k-1)*emg_offset, ...
        'Color', colours(k,:), 'LineWidth', 0.4);
end

xlim(ax, [0, t_axis(end)]);
ylim(ax, [-emg_offset, n_emg * emg_offset]);
xlabel(ax, 'Time (s)', 'Color','w');
ylabel(ax, 'Channel', 'Color','w');
ax.YTick = (0:15:n_emg-1) * emg_offset;
ax.YTickLabel = arrayfun(@(x) num2str(x+1), 0:15:n_emg-1, 'UniformOutput', false);
ax.GridColor = [0.3 0.3 0.3]; ax.XGrid = 'on';
title(ax, sprintf('Live EMG — %d channels | spacing=%.2f mV', n_emg, emg_offset), ...
    'Color','w', 'FontSize', 10);

guidata(fig, struct('pressed','', 'offset', emg_offset));
set(fig, 'KeyPressFcn', @(src,e) key_handler(src, e));

% =========================================================================
% LIVE LOOP
% =========================================================================
while ishandle(fig) && ~strcmp(guidata(fig).pressed, 'q')

    gd = guidata(fig);
    emg_offset = gd.offset;
    gd.pressed = '';
    guidata(fig, gd);

    while ishandle(fig) && tcpSocket.BytesAvailable < bytesPerBlock
        pause(0.001);
    end
    if ~ishandle(fig), break; end

    Temp = fread(tcpSocket, [TotNumByte, blockSamples], 'uint8');
    Temp = reshape(Temp, TotNumByte, blockSamples);
    D    = double(Temp(1:2:end,:))*256 + double(Temp(2:2:end,:));
    idx  = D >= 32768;
    D(idx) = D(idx) - 65536;

    blk = double(D(emg_channels, :)) * ConvFact;   % n_emg × blockSamples, mV

    emg_buf = [emg_buf(:, blockSamples+1:end), blk];

    for k = 1:n_emg
        set(h_lines(k), 'YData', emg_buf(k,:) + (k-1)*emg_offset);
    end

    ylim(ax, [-emg_offset, n_emg * emg_offset]);
    ax.YTick = (0:15:n_emg-1) * emg_offset;
    title(ax, sprintf('Live EMG — %d channels | spacing=%.2f mV', n_emg, emg_offset), ...
        'Color','w', 'FontSize', 10);

    drawnow limitrate;
end

% =========================================================================
% STOP
% =========================================================================
fwrite(tcpSocket, [0; CRC8(0,1)], 'uint8');
clear tcpSocket;
if ishandle(fig), close(fig); end
disp('EMG QA viewer stopped.');

% =========================================================================
% KEY HANDLER
% =========================================================================
function key_handler(src, e)
gd = guidata(src);
switch e.Key
    case 'q'
        gd.pressed = 'q';
    case 'equal'
        gd.offset = gd.offset * 1.3;
    case 'hyphen'
        gd.offset = max(0.01, gd.offset * 0.77);
end
guidata(src, gd);
end