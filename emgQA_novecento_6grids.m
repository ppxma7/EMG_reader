%% emg_qa_viewer_novecento.m
% Optimized live EMG QA viewer for Novecento+ (OT Bioelettronica).
% Dynamically scales plots for 1 to 6 BIO64HD grids.
%
% Press Q on either figure to quit.
% =========================================================================
close all; clear all; clc;

%% =========================================================================
% CONFIGURATION
% =========================================================================
WiFi = 0;
IP_USB  = '169.254.1.10';
IP_WiFi = '192.168.1.1';
TCPPort = 23456;

% -------------------------------------------------------------------------
% Acquisition Settings
% -------------------------------------------------------------------------
GridFsampSel = 1;     % 0 = 500 Hz, 1 = 2000 Hz, 2 = 4000 Hz, 3 = 8000 Hz
FsampVal     = [500 2000 4000 8000];
sampFreq     = FsampVal(GridFsampSel+1); % 2000 Hz
FSelAux      = 2;     % 1 = 500 Hz, 2 = 2000 Hz, 3 = 4000 Hz, 4 = 8000 Hz
AuxFsampCode = [0 16 32 48];
blockPeriods500 = 13;                      % Base periods read per update
mult            = sampFreq / 500;          % Grid/AUX samples per base period
accMult         = 8000 / 500;              % Accessory samples per period
blockSamples    = mult * blockPeriods500;  % 52 samples at 2000 Hz

nGrids      = 6;                           % Set to 6 active BIO64HD grids
NumChanGrid = 70;                          % 64 EMG + 4 IMU + 2 ACC
n_emg       = nGrids * 64;
ConvFact    = 0.0002861;                   % mV / raw count (HRES=0, Gain=0)

% Performance Tuning
wf_decim = 4;  % Downsample factor for visual display only

%% =========================================================================
% NOVECENTO INPUT CONFIGURATION
% =========================================================================
IN_Active = zeros(10,1);
Mode      = zeros(10,1);
Gain      = zeros(10,1);
HRES      = zeros(10,1);
HPF       = zeros(10,1);
Fsamp     = zeros(10,1);
for g = 1:nGrids
    IN_Active(g) = 1;
    Mode(g)      = 0;  % Monopolar
    Gain(g)      = 0;  
    HPF(g)       = 1;  
    HRES(g)      = 0;  
    Fsamp(g)     = GridFsampSel;
end

%% =========================================================================
% BUILD CONFIGURATION STRING (15 Bytes)
% =========================================================================
ConfString = zeros(1,15);
ConfString(1) = bin2dec('10000000') + AuxFsampCode(FSelAux) + IN_Active(10)*2 + IN_Active(9);
ConfString(2) = 0;
for i = 1:8
    ConfString(2) = ConfString(2) + IN_Active(i) * 2^(i-1);
end
ConfString(3) = 0; 
ConfString(4) = 1; 
for i = 1:10
    ConfString(4+i) = Mode(i)*64 + Gain(i)*16 + HPF(i)*8 + HRES(i)*4 + Fsamp(i);
end
ConfString(15) = CRC8(ConfString,14);

%% =========================================================================
% CONNECT & QUERY HARDWARE PROBES
% =========================================================================
if WiFi == 1
    tcpSocket = tcpclient(IP_WiFi, TCPPort, 'Timeout', 10);
else
    tcpSocket = tcpclient(IP_USB, TCPPort, 'Timeout', 10);
end
tcpSocket.ByteOrder = "little-endian";
fprintf('\nConnected to Novecento+.\n');

GetSetCmd    = zeros(1,2);
GetSetCmd(1) = 1;
GetSetCmd(2) = CRC8(GetSetCmd,1);

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
% 
% 
% write(tcpSocket, GetSetCmd, 'uint8');
% Settings = read(tcpSocket, 20, 'uint8')';

ChVsType = [0 14 22 38 46 70 102 0 0 0 0 0 0 0 0 0];
NumChan = zeros(10,1);
Ptr_IN  = zeros(11,1);
Size_IN = zeros(10,1);
Ptr_IN(1) = 1;

for i = 1:10
    NumChan(i) = ChVsType(Settings(i+1)+1);
    
    if IN_Active(i) == 1 && NumChan(i) ~= NumChanGrid
        error(['Input %d is active, but Novecento reports probe type code %d (%d channels). ' ...
               'Expected BIO64HD (70 channels).'], i, Settings(i+1), NumChan(i));
    end
    
    if IN_Active(i) == 1
        Size_IN(i) = mult * NumChan(i);
    else
        Size_IN(i) = 0;
    end
    
    Ptr_IN(i+1) = Ptr_IN(i) + Size_IN(i);
end

PacketSize1Block = (Ptr_IN(11)-1) + 16*mult + 128;
bytesPerBlock    = PacketSize1Block * blockPeriods500 * 2; % 2 bytes per int16

%% =========================================================================
% EMG CHANNEL INDICES & SPATIAL MAP PRE-CALCULATION
% =========================================================================
emg_channels = [];
for g = 1:nGrids
    base = (g-1)*70;
    emg_channels = [emg_channels, base+1 : base+64]; %#ok<AGROW>
end

ElChannelMap = [ ...
    52 39 26 13  1; ...
    53 40 27 14  1; ...
    54 41 28 15  2; ...
    55 42 29 16  3; ...
    56 43 30 17  4; ...
    57 44 31 18  5; ...
    58 45 32 19  6; ...
    59 46 33 20  7; ...
    60 47 34 21  8; ...
    61 48 35 22  9; ...
    62 49 36 23 10; ...
    63 50 37 24 11; ...
    64 51 38 25 12];

map_mask = (ElChannelMap >= 1 & ElChannelMap <= 64);
map_idx  = ElChannelMap;
map_idx(~map_mask) = 1; 

%% =========================================================================
% START STREAMING
% =========================================================================
write(tcpSocket, ConfString, 'uint8');
pause(0.2);
flush(tcpSocket);
fprintf('Novecento+ streaming started.\n');
fprintf('  Sampling frequency: %d Hz\n', sampFreq);
fprintf('  Active grids:       %d\n', nGrids);
fprintf('  EMG channels:       %d\n', n_emg);
fprintf('  Press Q to quit.\n\n');

%% =========================================================================
% DISPLAY PARAMETERS & BUFFERS
% =========================================================================
N_display  = round(sampFreq * 3 / blockSamples) * blockSamples;
t_axis     = (0:N_display-1) / sampFreq;
t_axis_sub = t_axis(1:wf_decim:end);
emg_buf    = zeros(n_emg, N_display);

% Subplot grid layout geometry
nRows = ceil(nGrids / 3);
nCols = min(nGrids, 3);

%% =========================================================================
% FIGURE 1 — DYNAMIC SPATIAL MAP (GRID RMS HEATMAPS)
% =========================================================================
grid_fig = figure('Color','k', 'Name','Novecento EMG Grid RMS — Q to quit', ...
                  'MenuBar','none', 'ToolBar','none');
ax_g = gobjects(nGrids,1);
h_img = gobjects(nGrids,1);
grid_data = zeros(13,5);

for g = 1:nGrids
    ax_g(g) = subplot(nRows, nCols, g);
    h_img(g) = imagesc(ax_g(g), grid_data);
    set(ax_g(g), 'Color','k', 'XColor','w', 'YColor','w');
    colormap(ax_g(g),'plasma');
    colorbar(ax_g(g));
    clim(ax_g(g),[0 0.2]);
    axis(ax_g(g),'equal','tight');
    xlabel(ax_g(g),'Column','Color','w');
    ylabel(ax_g(g),'Row','Color','w');
    title(ax_g(g), sprintf('Grid %d RMS (mV)', g), 'Color','w');
end

%% =========================================================================
% FIGURE 2 — DYNAMIC WATERFALL DISPLAY
% =========================================================================
offset_mv = 0.3;
wf_fig    = figure('Color','k', 'Name','Novecento EMG Waterfall — Q to quit', ...
                   'MenuBar','none', 'ToolBar','none');
ax_w = gobjects(nGrids,1);
h_wf = gobjects(n_emg,1);

for g = 1:nGrids
    ax_w(g) = subplot(nRows, nCols, g); hold(ax_w(g),'on');
    set(ax_w(g), 'Color','k', 'XColor','w', 'YColor','w');
    xlabel(ax_w(g),'Time (s)','Color','w');
    ylabel(ax_w(g),'Channel','Color','w');
    xlim(ax_w(g),[0 t_axis(end)]);
    ylim(ax_w(g), [-offset_mv, 64*offset_mv]);
    title(ax_w(g), sprintf('Grid %d — Traces', g), 'Color','w');
    
    ch_offset = (g-1)*64;
    for k = 1:64
        global_ch = ch_offset + k;
        h_wf(global_ch) = plot(ax_w(g), t_axis_sub, ...
            emg_buf(global_ch, 1:wf_decim:end) + (k-1)*offset_mv, ...
            'Color',[0.4 0.8 0.4], 'LineWidth',0.3);
    end
    yticks(ax_w(g), (0:8:63)*offset_mv);
    yticklabels(ax_w(g), string((ch_offset+1):8:(ch_offset+64)));
end

% Keypress handling
guidata(grid_fig, struct('pressed',''));
set(grid_fig, 'KeyPressFcn', @(src,e) guidata(src, setfield(guidata(src),'pressed',e.Key)));
guidata(wf_fig, struct('pressed',''));
set(wf_fig, 'KeyPressFcn', @(src,e) guidata(src, setfield(guidata(src),'pressed',e.Key)));

%% =========================================================================
% LIVE MAIN LOOP
% =========================================================================
while ishandle(grid_fig) && ishandle(wf_fig) && ...
      ~strcmp(guidata(grid_fig).pressed, 'q') && ...
      ~strcmp(guidata(wf_fig).pressed, 'q')
    
    gd_g = guidata(grid_fig); gd_g.pressed = ''; guidata(grid_fig, gd_g);
    gd_w = guidata(wf_fig);   gd_w.pressed = ''; guidata(wf_fig, gd_w);
    
    while tcpSocket.NumBytesAvailable >= bytesPerBlock * 2
        read(tcpSocket, PacketSize1Block * blockPeriods500, 'int16'); 
    end
    
    while ishandle(grid_fig) && ishandle(wf_fig) && tcpSocket.NumBytesAvailable < bytesPerBlock
        pause(0.0005);
    end
    if ~ishandle(grid_fig) || ~ishandle(wf_fig), break; end
    
    D = readBlockNovecento( ...
        tcpSocket, ...
        PacketSize1Block, ...
        blockPeriods500, ...
        Ptr_IN, ...
        nGrids, ...
        mult, ...
        accMult, ...
        blockSamples);
    
    blk = double(D(emg_channels,:)) * ConvFact;
    emg_buf = [emg_buf(:, blockSamples+1:end), blk];
    
    % Dynamic graphics updates across all grids
    emg_sub = emg_buf(:, 1:wf_decim:end);
    ch_rms  = sqrt(mean(blk.^2, 2));
    
    for g = 1:nGrids
        ch_offset = (g-1)*64;
        
        % 1. Update Waterfall
        for k = 1:64
            global_ch = ch_offset + k;
            set(h_wf(global_ch), 'YData', emg_sub(global_ch,:) + (k-1)*offset_mv);
        end
        
        % 2. Update Heatmaps & Titles
        grid_rms = ch_rms(map_idx + ch_offset) .* map_mask;
        set(h_img(g), 'CData', grid_rms);
        
        mean_rms = mean(ch_rms(ch_offset+1 : ch_offset+64));
        title(ax_g(g), sprintf('Grid %d RMS | mean = %.3f mV', g, mean_rms), 'Color','w');
    end
    
    drawnow limitrate;
end

%% =========================================================================
% CLEANUP
% =========================================================================
try
    StopString = zeros(1,15);
    StopString(15) = CRC8(StopString,14);
    write(tcpSocket, StopString, 'uint8');
    pause(0.2);
catch
end
clear tcpSocket;
if ishandle(grid_fig), close(grid_fig); end
if ishandle(wf_fig),   close(wf_fig);   end
disp('Novecento EMG QA viewer stopped.');

%% =========================================================================
% HELPER FUNCTION
% =========================================================================
function D = readBlockNovecento(tcpSocket, PacketSize1Block, nPeriods500, Ptr_IN, nGrids, mult, accMult, blockSamples)
    Temp = read(tcpSocket, PacketSize1Block * nPeriods500, 'int16')';
    Temp = reshape(Temp, PacketSize1Block, nPeriods500);
    D_grids = zeros(nGrids * 70, blockSamples);
    for g = 1:nGrids
        rows  = Ptr_IN(g):Ptr_IN(g+1)-1;
        slice = Temp(rows, :);
        flat  = reshape(slice, 1, []);
        chunk = reshape(flat, 70, blockSamples);
        D_grids((g-1)*70+1 : g*70, :) = chunk;
    end
    auxRows  = Ptr_IN(11) : Ptr_IN(11) + 16*mult - 1;
    auxSlice = Temp(auxRows, :);
    auxFlat  = reshape(auxSlice, 1, []);
    D_aux    = reshape(auxFlat, 16, blockSamples);
    accRowsStart = auxRows(end) + 1;
    accRows      = accRowsStart : accRowsStart + 128 - 1;
    accSlice     = int16(reshape(Temp(accRows, :), 1, []));
    accInt32     = typecast(accSlice, 'int32');
    accChunk     = reshape(accInt32, 4, accMult * nPeriods500);
    decim        = accMult / mult;
    D_acc        = accChunk(:, decim:decim:end);
    D = [D_grids; D_aux; D_acc];
end