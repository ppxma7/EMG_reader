%% minimalReadTest.m
close all; clear all; clc;

aux_ch_to_plot = 143;  % 141=left, 142=right, 143=sum_ext, 144=sum_internal
TCPPort  = 54320;
sampFreq = 2000;
blockSamples = 200;

% 2x Muovi+ (slots 5,6): 70ch each + 6 SyncStation = 146 total
% bytes per sample = 146*2 = 292
TotNumChan   = 146;
TotNumByte   = 292;
bytesPerBlock = TotNumByte * blockSamples;

% Config string for 2x Muovi+ in slots 5 and 6
ConfString    = zeros(18,1);
ConfString(1) = 2*2 + 1;          % 2 devices
ConfString(2) = (5-1)*16 + 1*8 + 0*2 + 1;   % slot 5, EMG=1, Mode=0
ConfString(3) = (6-1)*16 + 1*8 + 0*2 + 1;   % slot 6, EMG=1, Mode=0
ConfString(4) = CRC8(ConfString, 3);

% Connect
tcpSocket = tcpclient('192.168.76.1', TCPPort);
tcpSocket.InputBufferSize = TotNumChan * sampFreq * 3;
fwrite(tcpSocket, ConfString(1:4), 'uint8');
disp('Connected. Press Q to stop.');

% Figure
N   = 2000;
buf = zeros(1, N);
fig = figure;
h   = plot(1:N, buf, 'b', 'LineWidth', 1.5);
title(sprintf('Live raw channel %d', aux_ch_to_plot));
xlabel('Samples'); ylabel('Raw ADC counts');
xlim([1 N]); gca.YLimMode = 'auto';
guidata(fig, struct('pressed',''));
set(fig, 'KeyPressFcn', @(src,e) guidata(src, setfield(guidata(src),'pressed',e.Key)));

% Live loop
while ~strcmp(guidata(fig).pressed, 'q')
    while tcpSocket.BytesAvailable < bytesPerBlock
        pause(0.001);
    end

    Temp = fread(tcpSocket, [TotNumByte, blockSamples], 'uint8');

    % Reconstruct int16 from uint8 pairs
    ChInd = (1:2:TotNumByte*2-1);
    D = Temp(1:2:end,:)*256 + Temp(2:2:end,:);
    idx = D >= 32768;
    D(idx) = D(idx) - 65536;

    buf = [buf(blockSamples+1:end), D(aux_ch_to_plot,:)];
    set(h, 'YData', buf);
    drawnow limitrate;
end

% Stop
fwrite(tcpSocket, [0; CRC8(0,1)], 'uint8');
clear tcpSocket;
disp('Done.');