function target_trace = generate_target(task_shape, task_level, trap_ramp_s, trap_hold_s, lead_in_s, sampFreq, blockSamples, mcon_cycles)
% generate_target — preview the target trace before running
% Usage: target_trace = generate_target('mcon', 0.2, 2, 10, 5, 2000, 200, 3)

updates_per_sec = sampFreq / blockSamples;
ramp_steps  = round(trap_ramp_s * updates_per_sec);
hold_steps  = round(trap_hold_s * updates_per_sec);
lead_steps  = round(lead_in_s   * updates_per_sec);

switch task_shape
    case 'trap'
        target_trace = [zeros(1,lead_steps), ...
            linspace(0,task_level,ramp_steps), ...
            task_level*ones(1,hold_steps), ...
            linspace(task_level,0,ramp_steps), ...
            zeros(1,lead_steps)];

    case 'sombrero'
        brim_level = task_level * 0.4;
        brim_steps = round(1.5 * updates_per_sec);
        dip_steps  = round(1.0 * updates_per_sec);
        target_trace = [zeros(1,lead_steps), ...
            linspace(0,brim_level,ramp_steps), ...
            brim_level*ones(1,brim_steps), ...
            linspace(brim_level,task_level,dip_steps), ...
            task_level*ones(1,hold_steps), ...
            linspace(task_level,brim_level,dip_steps), ...
            brim_level*ones(1,brim_steps), ...
            linspace(brim_level,0,ramp_steps), ...
            zeros(1,lead_steps)];

    case 'mcon'
        % shift by pi/2 so sine starts at peak (zero slope) and ends at trough going up...
        % use -(2n-1) half cycles from pi/2:
        fine = 100;   % increase smoothness factor
        hold_steps_f = hold_steps * fine;
        t_hold    = linspace(pi/2, pi/2 + (2*mcon_cycles)*pi, hold_steps_f);
        sine_wave = task_level + (task_level * 0.3 * sin(t_hold));
        sine_wave = sine_wave(1:fine:end);
        % sine_wave(1) = task_level + 0.3*task_level*sin(pi/2) = task_level*1.3
        % so ramp goes to task_level*1.3
        target_trace = [zeros(1,lead_steps), ...
            linspace(0, sine_wave(1), ramp_steps), ...
            sine_wave, ...
            linspace(sine_wave(end), 0, ramp_steps), ...
            zeros(1,lead_steps)];
    otherwise
        error('Unknown task_shape: %s', task_shape);
end

% plot preview
t_axis = (0:numel(target_trace)-1) * (blockSamples/sampFreq);
figure('Color','w','Name',sprintf('Target preview — %s', task_shape));
plot(t_axis, target_trace, 'g', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Force (MVC fraction)');
title(sprintf('%s — level=%.0f%%  hold=%.0fs  cycles=%d', ...
    upper(task_shape), task_level*100, trap_hold_s, mcon_cycles));
ylim([-0.05, task_level*1.5]);
grid on;
end