%% minimalReadTest.m
close all; clear all; clc;

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

% flush startup packet and check actual channel count
pause(1);
bytes_available = tcpSocket.BytesAvailable;
samples_received = bytes_available / TotNumByte;
fprintf('Bytes available: %d\n', bytes_available);
fprintf('Implied samples at 292 bytes/sample: %.1f\n', samples_received);
fprintf('Implied samples at 152 bytes/sample: %.1f\n', bytes_available/152);

% Figure — 6 subplots, one per SyncStation AUX channel
N      = 2000;
labels = {'AUX1 (141)','AUX2 (142)','AUX3 (143)','AUX4 (144)','AUX5 (145)','AUX6 (146)'};
buf    = zeros(6, N);

fig = figure('Color','w');
ax  = gobjects(6,1);
h   = gobjects(6,1);
for k = 1:6
    ax(k) = subplot(6,1,k);
    h(k)  = plot(1:N, buf(k,:), 'b', 'LineWidth', 1);
    title(ax(k), labels{k});
    xlim(ax(k), [1 N]);
    ax(k).YLimMode = 'auto';
end
xlabel(ax(6), 'Samples');

guidata(fig, struct('pressed',''));
set(fig, 'KeyPressFcn', @(src,e) guidata(src, setfield(guidata(src),'pressed',e.Key)));

% Live loop
while ~strcmp(guidata(fig).pressed, 'q')
    while tcpSocket.BytesAvailable < bytesPerBlock
        pause(0.001);
    end

    Temp = fread(tcpSocket, [TotNumByte, blockSamples], 'uint8');
    
    D    = Temp(1:2:end,:)*256 + Temp(2:2:end,:);
    idx  = D >= 32768;
    D(idx) = D(idx) - 65536;

    for k = 1:6
        buf(k,:) = [buf(k, blockSamples+1:end), D(140+k, :)];
        set(h(k), 'YData', buf(k,:));
    end
    drawnow limitrate;
end

% Stop
fwrite(tcpSocket, [0; CRC8(0,1)], 'uint8');
clear tcpSocket;
disp('Done.');