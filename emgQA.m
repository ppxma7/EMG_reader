%% emg_qa_viewer.m
% Fast live EMG trace viewer for pre-experiment QA.
% Run this BEFORE experiment_muovi4.m to check electrode contact quality.
% All 128 EMG channels displayed as stacked traces, updating at ~hardware rate.
% Press Q to quit cleanly.
%
% Controls:
%   Q        — quit
%   +/-      — increase/decrease vertical spacing between traces
%   B        — toggle bandpass filter on/off (20-500 Hz)
%   R        — toggle RMS envelope overlay on/off

close all; clear; clc;

% =========================================================================
% CONFIG — match experiment_muovi4.m settings
% =========================================================================
TCPPort      = 54320;
sampFreq     = 2000;
blockSamples = 50;       % small blocks = fast update

TotNumChan    = 146;
TotNumByte    = 292;
bytesPerBlock = TotNumByte * blockSamples;

emg_channels  = [1:64, 71:134];   % 128 EMG channels (skip AUX in each Muovi+)
n_emg         = numel(emg_channels);
ConvFact      = 0.000286;          % raw ADC → mV

% Display options
emg_offset    = 0.15;    % mV spacing between traces (adjust with +/-)
N_display     = round(sampFreq * 3 / blockSamples) * blockSamples;  % 3s rolling window
do_filter     = true;    % bandpass on by default
do_rms        = false;   % RMS envelope overlay

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
write(tcpSocket, ConfString(1:4), 'uint8');
fprintf('Connected. Streaming EMG QA...\n');
fprintf('  Q = quit | +/- = spacing | B = bandpass | R = RMS overlay\n\n');

% flush ~0.5s startup
pause(0.5);
flush(tcpSocket);

% =========================================================================
% BANDPASS FILTER DESIGN (20-500 Hz, 4th order Butterworth)
% =========================================================================
[bp_b, bp_a] = butter(4, [20 500] / (sampFreq/2), 'bandpass');
% per-channel filter state (for continuous filtering across blocks)
zi = zeros(n_emg, max(length(bp_b), length(bp_a)) - 1);

% =========================================================================
% FIGURE
% =========================================================================
emg_buf = zeros(n_emg, N_display);   % rolling buffer (mV)

fig = figure('Color','k', 'Name','EMG QA Viewer — Q to quit', ...
    'MenuBar','none','ToolBar','none', 'Position',[50 50 900 900]);
ax  = axes(fig, 'Color','k', 'XColor','w', 'YColor','w');
hold(ax, 'on');

% pre-draw all lines
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

% RMS overlay lines (one per channel, drawn on top)
h_rms = gobjects(n_emg, 1);
for k = 1:n_emg
    h_rms(k) = plot(ax, t_axis, NaN(1,N_display) + (k-1)*emg_offset, ...
        'w', 'LineWidth', 1.2, 'Visible', 'off');
end

filter_txt = text(ax, 0.01, 0.99, 'BP: ON', 'Units','normalized', ...
    'Color','y', 'FontSize',9, 'VerticalAlignment','top');

title(ax, sprintf('Live EMG — %d channels | spacing=%.2f mV', n_emg, emg_offset), ...
    'Color','w', 'FontSize', 10);

% key handler — store in guidata
guidata(fig, struct('pressed','', 'offset', emg_offset, 'filter', true, 'rms', false));
set(fig, 'KeyPressFcn', @(src,e) key_handler(src, e));

% =========================================================================
% LIVE LOOP
% =========================================================================
while ishandle(fig) && ~strcmp(guidata(fig).pressed, 'q')

    % read key-driven option changes
    gd = guidata(fig);
    emg_offset = gd.offset;
    do_filter  = gd.filter;
    do_rms     = gd.rms;
    gd.pressed = '';
    guidata(fig, gd);

    % wait for data
    while ishandle(fig) && tcpSocket.BytesAvailable < bytesPerBlock
        pause(0.001);
    end
    if ~ishandle(fig), break; end

    % read block
    Temp = read(tcpSocket, TotNumByte * blockSamples, 'uint8');
    %Temp = fread(tcpSocket, [TotNumByte, blockSamples], 'uint8');

    Temp = reshape(Temp, TotNumByte, blockSamples);
    D    = double(Temp(1:2:end,:))*256 + double(Temp(2:2:end,:));
    idx  = D >= 32768;
    D(idx) = D(idx) - 65536;

    % extract EMG block (mV)
    blk = double(D(emg_channels, :)) * ConvFact;   % n_emg × blockSamples

    % optional bandpass filter (continuous state)
    if do_filter
        [blk_f, zi] = filter(bp_b, bp_a, blk.', [], 1);
        blk_f = blk_f.';   % back to n_emg × blockSamples
    else
        zi = zeros(n_emg, max(length(bp_b), length(bp_a)) - 1);
        blk_f = blk;
    end

    % roll buffer
    emg_buf = [emg_buf(:, blockSamples+1:end), blk_f];

    % update traces
    for k = 1:n_emg
        set(h_lines(k), 'YData', emg_buf(k,:) + (k-1)*emg_offset);
    end

    % RMS envelope
    if do_rms
        win = min(round(0.05 * sampFreq), N_display);  % 50 ms RMS window
        rms_buf = sqrt(movmean(emg_buf.^2, win, 2));
        for k = 1:n_emg
            set(h_rms(k), 'YData', rms_buf(k,:) + (k-1)*emg_offset, 'Visible','on');
        end
    else
        for k = 1:n_emg
            set(h_rms(k), 'Visible','off');
        end
    end

    % update labels
    filter_str = 'BP: ON';
    if ~do_filter, filter_str = 'BP: OFF'; end
    if do_rms, filter_str = [filter_str ' | RMS: ON']; end
    set(filter_txt, 'String', filter_str);

    % update y-axis spacing if changed
    ylim(ax, [-emg_offset, n_emg * emg_offset]);
    ax.YTick = (0:15:n_emg-1) * emg_offset;
    title(ax, sprintf('Live EMG — %d channels | spacing=%.2f mV | %.0f Hz', ...
        n_emg, emg_offset, sampFreq/blockSamples * ...
        (tcpSocket.BytesAvailable == 0 || 1)), 'Color','w', 'FontSize',10);

    drawnow limitrate;
end

% =========================================================================
% STOP
% =========================================================================
write(tcpSocket, [0; CRC8(0,1)], 'uint8');
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
    case 'equal'    % + key (unshifted = on most keyboards)
        gd.offset = gd.offset * 1.3;
    case 'hyphen'   % - key
        gd.offset = max(0.01, gd.offset * 0.77);
    case 'b'
        gd.filter = ~gd.filter;
    case 'r'
        gd.rms = ~gd.rms;
end
guidata(src, gd);
end