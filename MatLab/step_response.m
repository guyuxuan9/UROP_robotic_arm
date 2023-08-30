log_path = "C:\\Users\\jesse\\Downloads\\raspberry_pi\\step2";
bag = ros2bagreader(log_path);
baginfo = ros2("bag","info",log_path);

pulse_width = select(bag,"Topic","/motion_topic/multi_id_pos_dur");
msgs = readMessages(pulse_width);

servo6 = zeros(size(msgs, 1), 1, 'uint16'); % Initialize an array to store the values
for i = 1:size(msgs, 1)
    servo6(i) = msgs{i, 1}.id_pos_dur_list(4).position;
end

sample_start = 258;
sample_end = 410;

servo6 = servo6(sample_start:sample_end);
sample_point = uint16((0:sample_end-sample_start)');
time = double(sample_point) / 30;
% find peak value, peak time
[peak_value, peak_index] = max(servo6);

% find second peak value, second peak index
[second_peak_value, second_peak_index] = max(servo6(30:60));
second_peak_index = second_peak_index + 30;

% find "period" of oscillation
T = (second_peak_index - peak_index)/30;

% define steady-state value
servo6_steady = 523;

% calculate max overshoot
max_overshoot = peak_value - servo6_steady;
perc_overshoot = double(max_overshoot)/double(servo6_steady);

% Define the percentage of settling you're interested in (1% in this case)
percentage_of_settling = 0.01;

% Define the range around the final value within which settling is considered achieved
settling_range = percentage_of_settling * servo6_steady;

% Find the time index where settling is achieved
settling_time_index = find(abs(servo6(50:80)) <= (settling_range + servo6_steady), 1);
settling_time_index = settling_time_index + 50;
ts = settling_time_index/30;

figure;
offset = 462;
scatter(time, servo6-offset,'DisplayName','Experiment');
hold on
scatter(time(peak_index), servo6(peak_index)-offset, 'r', 'filled');
scatter(time(second_peak_index), servo6(second_peak_index)-offset, 'r', 'filled');

% highlight the first point in the settling range
scatter(time(settling_time_index), servo6(settling_time_index)-offset, 'r', 'filled');


yline(servo6_steady - settling_range-offset,'Color', 'r', 'LineStyle', '--');
yline(servo6_steady + settling_range-offset,'Color', 'r', 'LineStyle', '--')
line([(peak_index-1)/30, (peak_index-1)/30], [servo6(1)-offset,peak_value-offset], 'Color', 'k', 'LineStyle', '--', 'LineWidth', 0.5);
line([time(settling_time_index),time(settling_time_index)],[servo6(1)-offset,servo6(settling_time_index)-offset],'Color', 'k', 'LineStyle', '--', 'LineWidth', 0.5)

% label t_p
text((peak_index-1)/30, 455, '$t_p$', 'Interpreter', 'latex', 'HorizontalAlignment', 'center', 'FontSize', 12, 'Color', 'k');

line([(second_peak_index-1)/30, (second_peak_index-1)/30], [497-offset,second_peak_value-offset], 'Color', 'k', 'LineStyle', '--', 'LineWidth', 0.5);
line([(peak_index-1)/30, (second_peak_index-1)/30], [497-offset,497-offset], 'Color', 'r', 'LineStyle', '-', 'LineWidth', 0.5);
line([time(1),time(settling_time_index)],[490-offset,490-offset],'Color', 'r', 'LineStyle', '-', 'LineWidth', 0.5)



% add text for period T
text((peak_index-1)/30 + ((second_peak_index-1)/30 - (peak_index-1)/30) / 2, 493-offset, '$T$','Interpreter','latex', 'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', 'k');
% add text for settling time t_s
text(time(settling_time_index) / 2, 485-offset, '$t_s \approx \frac{4.6}{\sigma} = \frac{4.6}{\xi \omega_n}$','Interpreter','latex', 'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', 'k');
xlabel('Time (s)');
ylabel('Pulse Width');
title('Step response of servo motor 6');
%% calculate wn and xi
xi = -log(perc_overshoot) * sqrt(1 / (pi^2 + (log(perc_overshoot))^2));
wn = 4.6/(ts * xi);
xi = 0.18;
wn = 1.05/xi;
%% write the transfer function
G = tf(wn^2,[1,2*xi*wn,wn^2]);

%% step response simulation
% Generate the step response
t = 0:0.01:6;         % Time vector
% Define the step input value
step_input = 61;

[y, t] = step(step_input * G, t);

% Plot the step response
plot(t, y,'DisplayName','Simulation');
legend("Experiment","","","","","","","","","","","Simulation");

%% PID bode plot
G = tf([34.03],[1,2.1,34.03]);
G2 = tf([34.03],[1,2.1,34.03],'InputDelay',0.03);
figure;
margin(G);


figure;
margin(G2);

