% script to quickly check outputs of the EMG_reader
%
% MA June 2026
close all
clear all
clc

%% load
[file, path] = uigetfile();
fullpath = fullfile(path, file);
load(fullpath);

%% Force + Target
t = (0:length(Force)-1)/signal.fsamp;
figure
plot(t,Force(1,:),'-r'); hold on
plot(t,Force(2,:),'-b')
plot(t,Force(3,:),'-k')
plot(t,Force(4,:),'--','Color','#de2d26')
plot(t,Force(5,:),'--','Color','#3182bd')
plot(t,Force(6,:),'--','Color','#31a354')
plot(t,Force(7,:),'-m')
legend(signal.auxiliaryname, 'Interpreter', 'none')
title('Force channels'); xlabel('Time (s)'); ylabel('Volts')

%% Auxiliary channels
figure
plot(signal.auxiliary')
legend(signal.auxiliaryname)
title('Auxiliary channels')

%% EMG channels (overlaid per grid)
nChan = signal.nChan;
ngrid = signal.ngrid;
chPerGrid = nChan/ngrid;
t = (0:size(signal.data,2)-1)/signal.fsamp;

fprintf('Total channels: %d, Grids: %d, Channels per grid: %d\n', nChan, ngrid, chPerGrid);

figure
tl = tiledlayout(ngrid,1);
for g = 1:ngrid
    idx = (g-1)*chPerGrid + (1:chPerGrid);
    nexttile
    plot(t, signal.data(idx,:))
    title(sprintf('Grid %d (%s) - %d channels', g, signal.gridname{g}, chPerGrid))
    xlabel('Time (s)'); ylabel('Amplitude')
    axis tight
end

%% EMG channels (Stacked / Waterfall per Grid)
nChan = signal.nChan;
ngrid = signal.ngrid;
chPerGrid = nChan/ngrid;
t = (0:size(signal.data,2)-1)/signal.fsamp;

fprintf('Total channels: %d, Grids: %d, Channels per grid: %d\n', nChan, ngrid, chPerGrid);

for g = 1:ngrid
    idx = (g-1)*chPerGrid + (1:chPerGrid);
    gridData = signal.data(idx, :);

    % Compute vertical spacing step based on overall peak-to-peak amplitude
    % Compute vertical spacing step using range (max - min) across time (dimension 2)
    offsetStep = max(range(gridData, 2));
    if offsetStep == 0; offsetStep = 1; end % Prevents zero spacing if signals are flat

    % Create a matrix of offsets for vectorised plotting
    offsets = (0:chPerGrid-1)' * offsetStep;
    stackedData = gridData + offsets;
    
    
    figure('Name', sprintf('Grid %d - %s', g, signal.muscle{g}));
    plot(t, stackedData, 'Color',[0.4 0.8 0.4]);
    % 
    set(gcf,'color','black')

    ax = gca;
    ax.Color = 'k';                     % Black background inside plot
    ax.XColor = 'w';                     % White X-axis line & labels
    ax.YColor = 'w';                     % White Y-axis line & labels
    
    % Force dark grid lines
    grid on
    ax.GridColorMode = 'manual';         % Disable auto grid color calculation
    ax.GridColor = [0.25 0.25 0.25];     % Dark grey grid lines (adjust closer to 0 for darker)
    ax.GridAlpha = 1;
    
    % Formatting
    title(sprintf('EMG Stacked: %s (%s) - %d channels', signal.muscle{g}, signal.gridname{g}, chPerGrid))
    xlabel('Time (s)')
    ylabel('Channels (Offset)')
    yticks(offsets)
    yticklabels(cellstr(num2str((1:chPerGrid)')))
    axis tight
    grid on
end

