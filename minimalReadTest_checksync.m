%% minimalReadTest.m
close all;
clear all;
clc;

TCPPort      = 54320;
sampFreq     = 2000;
blockSamples = 200;

TotNumChan    = 146;
TotNumByte    = 292;
bytesPerBlock = TotNumByte * blockSamples;

% Config string for 2x Muovi+ in slots 5 and 6
ConfString    = zeros(18,1);
ConfString(1) = 2*2 + 1;
ConfString(2) = (5-1)*16 + 1*8 + 0*2 + 1;
ConfString(3) = (6-1)*16 + 1*8 + 0*2 + 1;
ConfString(4) = CRC8(ConfString, 3);

% Connect
tcpSocket = tcpclient('192.168.76.1', TCPPort);
tcpSocket.InputBufferSize = TotNumChan * sampFreq * 3;

fwrite(tcpSocket, ConfString(1:4), 'uint8');

disp('Connected. Press Q to stop.');

% Flush startup packet and check actual channel count
pause(1);

bytes_available = tcpSocket.BytesAvailable;
samples_received = bytes_available / TotNumByte;

fprintf('Bytes available: %d\n', bytes_available);
fprintf('Implied samples at 292 bytes/sample: %.1f\n', samples_received);
fprintf('Implied samples at 152 bytes/sample: %.1f\n', bytes_available/152);

%% Channels to display
plotCh = [69 139 141 142 143 144 145 146];

labels = { ...
    'Muovi #1 Ch69 (Sync/Trigger)', ...
    'Muovi #2 Ch139 (Sync/Trigger)', ...
    'AUX1 Ch141', ...
    'AUX2 Ch142', ...
    'AUX3 Ch143', ...
    'AUX4 Ch144', ...
    'AUX5 Ch145', ...
    'AUX6 Ch146'};

%% Figure
N   = 2000;
nCh = length(plotCh);

buf = zeros(nCh, N);

fig = figure('Color','w', ...
             'Name','Muovi Trigger/Sync Monitor');

ax = gobjects(nCh,1);
h  = gobjects(nCh,1);

for k = 1:nCh

    ax(k) = subplot(nCh,1,k);

    h(k) = plot(1:N, buf(k,:), 'b', 'LineWidth', 1);

    title(labels{k});
    xlim([1 N]);
    grid on;

end

xlabel('Samples');

guidata(fig, struct('pressed', ''));

set(fig, 'KeyPressFcn', ...
    @(src,e) guidata(src, ...
    setfield(guidata(src),'pressed',e.Key)));

%% Live loop
while isvalid(fig) && ~strcmp(guidata(fig).pressed, 'q')

    while tcpSocket.BytesAvailable < bytesPerBlock
        pause(0.001);
        if ~isvalid(fig)
            break;
        end
    end

    if ~isvalid(fig)
        break;
    end

    Temp = fread(tcpSocket, [TotNumByte, blockSamples], 'uint8');

    % Convert byte stream to signed 16-bit samples
    D = Temp(1:2:end,:) * 256 + Temp(2:2:end,:);

    idx = D >= 32768;
    D(idx) = D(idx) - 65536;

    % Update display
    for k = 1:nCh

        ch = plotCh(k);

        buf(k,:) = [buf(k, blockSamples+1:end), D(ch,:)];

        set(h(k), 'YData', buf(k,:));

    end

    drawnow limitrate;

end

%% Stop acquisition
try
    fwrite(tcpSocket, [0; CRC8(0,1)], 'uint8');
catch
end

clear tcpSocket;

disp('Done.');