function find_force_live(t, aux_idx)

close all
figure('Color','w');
ax = axes;
hold(ax,'on');

title(ax,'Live AUX Channels (stacked)');
xlabel(ax,'Samples');
ylabel(ax,'Amplitude (offset per channel)');

num_aux = length(aux_idx);
offset = 2000;   % vertical spacing

% Pre‑allocate line handles
lines = gobjects(num_aux,1);
for k = 1:num_aux
    lines(k) = plot(ax, nan, nan);
end

buffer_length = 10000;
buffer = zeros(num_aux, buffer_length);
x = 1:buffer_length;

disp('Plotting AUX channels only. Press q to quit.');

set(gcf,'KeyPressFcn',@(src,event) assignin('base','keyPressed',event.Key));
keyPressed = '';

while ~strcmp(keyPressed,'q')

    % Read a small block (100 samples)
    block_size = 100;
    total_channels = max(aux_idx);   % safe upper bound
    expected = total_channels * block_size;

    while t.NumBytesAvailable < expected
    end

    Temp = read(t, expected, "int16");
    data = reshape(Temp, total_channels, block_size);

    % Extract AUX channels only
    aux_data = double(data(aux_idx, :));

    % Scroll buffer
    buffer(:,1:end-block_size) = buffer(:,block_size+1:end);
    buffer(:,end-block_size+1:end) = aux_data;

    % Update plot
    for k = 1:num_aux
        y = buffer(k,:) + (k-1)*offset;
        set(lines(k), 'XData', x, 'YData', y);
    end

    xlim(ax, [1 buffer_length]);
    %ylim(ax, [0 num_aux*offset]);
    ylim(ax, [0 num_aux*offset*1.5]);
    drawnow limitrate

end

disp('Stopped.');
close all
