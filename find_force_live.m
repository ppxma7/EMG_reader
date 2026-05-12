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
ylabel(ax,'Amplitude (offset per channel)');

num_aux = numel(aux_idx);
offset  = 25000;

lines = gobjects(num_aux, 1);
for k = 1:num_aux
    lines(k) = plot(ax, nan, nan);
end

buffer_length = 100000;
buffer = zeros(num_aux, buffer_length);
x = 1:buffer_length;

% Low-pass filter
Fs = 2000; 
% fc = 50;
% [b,a] = butter(4, fc/(Fs/2), 'low');

set(gcf, 'KeyPressFcn', @(~,e) assignin('base','keyPressed', e.Key));
keyPressed = '';

block_size = 500;  % was 100
fc = 20;             % was 50 — force barely changes faster than 2Hz
[b,a] = butter(5, fc/(Fs/2), 'low');  % lower order too, avoid instability at low fc

while ~strcmp(keyPressed, 'q')

    while t.NumBytesAvailable < total_channels * block_size * 2
        pause(0.001);
    end

    Temp = read(t, total_channels * block_size, 'int16');
    data = reshape(Temp, total_channels, block_size);

    aux_data = double(data(aux_idx, :));

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
    ylim(ax, [-1000000 1000000]);
    legend(ax, labels, 'Location','northwest');
    drawnow limitrate
end

disp('Stopped.');
close all