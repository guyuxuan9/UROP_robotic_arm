%%
log_path = "PATH_OF_THE_BAG_FILE";
bag = ros2bagreader(log_path);
baginfo = ros2("bag","info",log_path);

pulse_width = select(bag,"Topic","/motion_topic/multi_id_pos_dur");
msgs = readMessages(pulse_width);



servo6 = zeros(size(msgs, 1), 1, 'uint16'); % Initialize an array to store the values
servo5 = zeros(size(msgs, 1), 1, 'uint16'); % Initialize an array to store the values
servo4 = zeros(size(msgs, 1), 1, 'uint16'); % Initialize an array to store the values
servo3 = zeros(size(msgs, 1), 1, 'uint16'); % Initialize an array to store the values

for i = 1:size(msgs, 1)
    servo6(i) = msgs{i, 1}.id_pos_dur_list(4).position;
    servo5(i) = msgs{i, 1}.id_pos_dur_list(3).position;
    servo4(i) = msgs{i, 1}.id_pos_dur_list(2).position;
    servo3(i) = msgs{i, 1}.id_pos_dur_list(1).position;
end

sample_start = 50;
sample_end = 300;

servo6 = servo6(sample_start:sample_end);
servo5 = servo5(sample_start:sample_end);
servo4 = servo4(sample_start:sample_end);
servo3 = servo3(sample_start:sample_end);

figure;

subplot(2,2,1)
scatter(1:numel(servo6), servo6);
xlabel('Sample Number');
ylabel('Pulse Width');
title('Movement of servo motor 6');

subplot(2,2,2);
scatter(1:numel(servo5), servo5);
xlabel('Sample Number');
ylabel('Pulse Width');
title('Movement of servo motor 5');

subplot(2,2,3);
scatter(1:numel(servo4), servo4);
xlabel('Sample Number');
ylabel('Pulse Width');
title('Movement of servo motor 4');

subplot(2,2,4);
scatter(1:numel(servo3), servo3);
xlabel('Sample Number');
ylabel('Pulse Width');
title('Movement of servo motor 3');

%%
bag = ros2bagreader(log_path);
controller_out = select(bag,"Topic","/controller/output");
msgs_controller = readMessages(controller_out);

x_coord = zeros(size(msgs_controller,1),1,'double');
y_coord = zeros(size(msgs_controller,1),1,'double');
z_coord = zeros(size(msgs_controller,1),1,'double');

for i=1:size(msgs_controller,1)
    x_coord(i) = msgs_controller{i,1}.controller_out(1);
    y_coord(i) = msgs_controller{i,1}.controller_out(2);
    z_coord(i) = msgs_controller{i,1}.controller_out(3);
end

for i = 1:size(msgs_controller,1)
    x_coord(i) = y_coord(i)/(tand(double(x_coord(i))/1000*240));
end
x_coord = x_coord(sample_start:sample_end);
y_coord = y_coord(sample_start:sample_end);
z_coord = z_coord(sample_start:sample_end);

% Create a figure
figure;

plot3(x_coord(1), y_coord(1), z_coord(1), 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
axis equal;
daspect([1 1 1]);
% Set axis labels and title
xlabel('X Coordinate');
ylabel('Y Coordinate');
zlabel('Z Coordinate');
title('3D Diagram of Coordinates');
hold on

% Loop through each point and plot
for i = 2:size(x_coord, 1)
    % Plot the current point
    plot3(x_coord(i), y_coord(i), z_coord(i), 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    
    % Pause to show the point for a short duration
    pause(0.1); % Adjust the pause duration as needed
    hold on
    % Refresh the plot
    drawnow;
end



% Turn off hold mode
hold off;