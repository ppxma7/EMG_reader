function find_force_live(t, aux_idx, total_channels, labels)
% find_force_live(t, aux_idx, total_channels)
% t               - tcpclient object
% aux_idx         - vector of channel indices to display e.g. [65:70, 135:140]
% total_channels  - full channel count for correct reshape
if nargin < 4
    labels = string(aux_idx);
end

close all
figure('Color','w');
ax = axes;
hold(ax,'on');
title(ax,'Live AUX Channels (stacked)');
xlabel(ax,'Samples');
ylabel(ax,'Amplitude (mV, offset per channel)');

num_aux = numel(aux_idx);
offset  = 5000;   % mV offset between channels (10V range = 10000 mV total)

lines = gobjects(num_aux, 1);
for k = 1:num_aux
    lines(k) = plot(ax, nan, nan);
end

buffer_length = 100000;
buffer = zeros(num_aux, buffer_length);
x = 1:buffer_length;

% Low-pass filter
Fs = 2000; 

set(gcf, 'KeyPressFcn', @(~,e) assignin('base','keyPressed', e.Key));
keyPressed = '';

block_size = 500;
fc = 20;             % Hz — adjust to match OTBioLab smoothness (try 10 if still noisy)
[b,a] = butter(5, fc/(Fs/2), 'low');

while ~strcmp(keyPressed, 'q')

    while t.NumBytesAvailable < total_channels * block_size * 2
        pause(0.001);
    end

    Temp = read(t, total_channels * block_size, 'int16');
    data = reshape(Temp, total_channels, block_size);

    aux_data = double(data(aux_idx, :));

    % Scale raw int16 counts to millivolts
    % SyncStation AUX inputs: ±10 V range → int16 (±32768)
    % 10000 mV / 32768 counts = 0.3052 mV/count
    aux_scale_mV = 10000 / 32768;
    aux_data = aux_data * aux_scale_mV;

    for k = 1:num_aux
        aux_data(k,:) = filtfilt(b, a, aux_data(k,:));
    end

    % Scroll buffer
    buffer = circshift(buffer, -block_size, 2);
    buffer(:, end-block_size+1:end) = aux_data;

    % Update plots
    for k = 1:num_aux
        if num_aux == 1
            y = (buffer(k,:) - mean(buffer(k,:))) * 5;
        else
            y = buffer(k,:) + (k-1)*offset;
        end
        set(lines(k), 'XData', x, 'YData', y);
    end

    xlim(ax, [1 buffer_length]);
    ylim(ax, [-5000 num_aux*offset + 5000]);
    legend(ax, labels, 'Location','northwest');
    drawnow limitrate
end

disp('Stopped.');
close all