function find_force_live(t, aux_idx)

close all
figure('Color','w');
ax = axes;
hold(ax,'on');

title(ax,'Live AUX Channels (stacked)');
xlabel(ax,'Samples');
ylabel(ax,'Amplitude (offset per channel)');

num_aux = length(aux_idx);
offset = 25000; %5000;   % vertical spacing for stacked mode

% Pre‑allocate line handles
lines = gobjects(num_aux,1);
for k = 1:num_aux
    lines(k) = plot(ax, nan, nan);
end

buffer_length = 10000;
buffer = zeros(num_aux, buffer_length);
x = 1:buffer_length;

disp('Plotting AUX channels only. Press q to quit.');

% Filtering
Fs = 2000;
fc = 50;
[b,a] = butter(4, fc/(Fs/2), 'low');

set(gcf,'KeyPressFcn',@(src,event) assignin('base','keyPressed',event.Key));
keyPressed = '';

while ~strcmp(keyPressed,'q')

    % Read a small block (100 samples)
    block_size = 100;
    total_channels = max(aux_idx);
    expected = total_channels * block_size;

    while t.NumBytesAvailable < expected
    end

    Temp = read(t, expected, "int16");
    data = reshape(Temp, total_channels, block_size);

    % Extract AUX channels only
    aux_data = double(data(aux_idx, :));

    % Apply filtering
    for k = 1:num_aux
        aux_data(k,:) = filtfilt(b, a, aux_data(k,:));
    end

    % Scroll buffer
    buffer(:,1:end-block_size) = buffer(:,block_size+1:end);
    buffer(:,end-block_size+1:end) = aux_data;

    % Update plot
    for k = 1:num_aux
        y = buffer(k,:);

        if num_aux == 1
            % SINGLE CHANNEL MODE
            % Auto‑scale amplitude
            y = y - mean(y);     % center
            y = y * 5;           % amplify so movement is visible
        else
            % STACKED MODE
            y = y + (k-1)*offset;
        end

        set(lines(k), 'XData', x, 'YData', y);
    end

    xlim(ax, [1 buffer_length]);

    % if num_aux == 1
    %     % AUTO‑ZOOM Y‑AXIS FOR SINGLE CHANNEL
    %     sig = buffer(1,:);
    %     ymin = min(sig) - 2000;
    %     ymax = max(sig) + 2000;
    % 
    %     % If signal is tiny, enforce a minimum zoom window
    %     if ymax - ymin < 500
    %         ymin = ymin - 250;
    %         ymax = ymax + 250;
    %     end
    % 
    %     ylim(ax, [ymin ymax]);
    % else
    %     ylim(ax, [0 num_aux*offset*1.5]);
    % end
    ylim(ax, [-120000 120000]);

    legend(string(aux_idx));

    drawnow limitrate

end

disp('Stopped.');
close all
