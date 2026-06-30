% script to quickly check outputs of the EMG_reader
%
% MA June 2026

close all
clear all
clc

%% load
file = uigetfile();
load(file);

%% Force + Target
t = (0:length(Force)-1)/signal.fsamp;

figure
plot(t,Force(1,:),'-r'); hold on
plot(t,Force(2,:),'-b')
plot(t,Force(3,:),'-k')
plot(t,Force(4,:),'--r')
plot(t,Force(5,:),'--b')
plot(t,Force(6,:),'--k')
plot(t,Force(7,:),'-m')
legend(signal.auxiliaryname, 'Interpreter', 'none')
title('Force channels'); xlabel('Time (s)'); ylabel('Volts')


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
    title(sprintf('%s (%s) - %d channels', signal.muscle{g}, signal.gridname{g}, chPerGrid))
    xlabel('Time (s)'); ylabel('Amplitude')
    axis tight
end

%% Auxiliary channels
figure
plot(signal.auxiliary')
legend(signal.auxiliaryname)
title('Auxiliary channels')