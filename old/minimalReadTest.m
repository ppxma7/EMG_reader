%% minimalReadTest.m
% Live plot of one raw AUX channel — no processing, no smoothing

close all; clear all; clc;

% =========================================================================
% USER OPTIONS
% =========================================================================
aux_ch_to_plot = 141;   % SyncStatChan(1) = first SyncStation AUX
TCPPort        = 54320;
PlotTime       = 1;

% =========================================================================
% DEVICE CONFIG — 2x Muovi+ in slots 5 and 6
% =========================================================================
DeviceEN = zeros(1,16);
EMG      = ones(1,16);
Mode     = zeros(1,16);
DeviceEN(5) = 1;
DeviceEN(6) = 1;

NumChan  = [38 38 38 38 70 70 8 8 8 8 8 8 8 8 10 10];
sampFreq = 2000;
blockSamples = 200;   % read this many samples per update

% =========================================================================
% BUILD CONFIG STRING
% =========================================================================
SizeComm = sum(DeviceEN);
ConfString = zeros(18,1);
ConfString(1) = SizeComm*2 + 1;
ConfStrLen = 2;
TotNumChan = 0;
TotNumByte = 0;

for i = 1:16
    if DeviceEN(i) == 1
        ConfString(ConfStrLen) = (i-1)*16 + EMG(i)*8 + Mode(i)*2 + 1;
        TotNumChan = TotNumChan + NumChan(i);
        TotNumByte = TotNumByte + NumChan(i) * 2;
        ConfStrLen = ConfStrLen + 1;
    end
end

SyncStatChan = TotNumChan+1 : TotNumChan+6;
TotNumChan   = TotNumChan + 6;
TotNumByte   = TotNumByte + 12;
blockBytes   = (TotNumByte / (sampFreq)) * blockSamples * sampFreq;
% bytes per block
bytesPerBlock = TotNumByte / sampFreq * blockSamples * sampFreq;
bytesPerBlock = (TotNumByte / sampFreq) * blockSamples;
% simpler:
bytesPerSample = TotNumByte;   % bytes for one sample across all channels
bytesPerBlock  = bytesPerSample * blockSamples;

fprintf('TotNumChan = %d,  SyncStatChan = %d:%d\n', TotNumChan, SyncStatChan(1), SyncStatChan(end));
fprintf('Plotting channel %d\n', aux_ch_to_plot);

ConfString(ConfStrLen) = CRC8(ConfString, ConfStrLen-1);

% =========================================================================
% CONNECT
% =========================================================================
tcpSocket = tcpclient('192.168.76.1', TCPPort);
tcpSocket.InputBufferSize = TotNumChan * sampFreq * 3;
fwrite(tcpSocket, ConfString(1:ConfStrLen), 'uint8');
disp('Connected. Press Q on figure to stop.');

% =========================================================================
% FIGURE
% =========================================================================
N = 2000;   % rolling buffer length (samples)
buf = zeros(1, N);

fig = figure;
h = plot(1:N, buf, 'b', 'LineWidth', 1.5);
title(sprintf('Live raw channel %d', aux_ch_to_plot));
xlabel('Samples'); ylabel('Raw ADC counts');
xlim([1 N]);
ax = gca;
ax.YLimMode = 'auto';

guidata(fig, struct('pressed',''));
set(fig, 'KeyPressFcn', @(src,e) guidata(src, setfield(guidata(src),'pressed',e.Key)));

% =========================================================================
% LIVE LOOP
% =========================================================================
while ~strcmp(guidata(fig).pressed, 'q')

    % wait for block
    while tcpSocket.BytesAvailable < bytesPerBlock
        pause(0.001);
    end

    Temp = fread(tcpSocket, [TotNumByte, blockSamples], 'uint8');

    % reconstruct all channels exactly as Read_SyncStation.m
    data = zeros(TotNumChan, blockSamples);
    ChanReady = 1;
    TempWork = Temp;

    for DevId = 1:16
        if DeviceEN(DevId) == 1
            ChInd = (1:2:NumChan(DevId)*2);
            D = TempWork(ChInd,:)*256 + TempWork(ChInd+1,:);
            idx = D >= 32768;
            D(idx) = D(idx) - 65536;
            data(ChanReady:ChanReady+NumChan(DevId)-1,:) = D;
            TempWork(1:NumChan(DevId)*2,:) = [];
            ChanReady = ChanReady + NumChan(DevId);
        end
    end

    % SyncStation channels
    ChInd = (1:2:12);
    D = TempWork(ChInd,:)*256 + TempWork(ChInd+1,:);
    idx = D >= 32768;
    D(idx) = D(idx) - 65536;
    data(ChanReady:ChanReady+5,:) = D;

    % update rolling buffer
    buf = [buf(blockSamples+1:end), data(aux_ch_to_plot,:)];
    set(h, 'YData', buf);
    drawnow limitrate;
end

% =========================================================================
% STOP
% =========================================================================
ConfStop = [0; CRC8(0, 1)];
fwrite(tcpSocket, ConfStop, 'uint8');
clear tcpSocket;
disp('Done.');