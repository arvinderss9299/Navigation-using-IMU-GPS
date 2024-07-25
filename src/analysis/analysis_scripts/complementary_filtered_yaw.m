clear all
bag = rosbag('circles.bag');
imu_topic_data = select(bag, "Time",[bag.StartTime bag.EndTime], "Topic", "/imutopic");
imustructs = readMessages(imu_topic_data, "DataFormat", "struct");
imu_matrix = cell2mat(imustructs);

df = readtable("circles.csv", 'PreserveVariableNames',true);
t = df.Time - df.Time(510,1);
time = t(510:end,1);

quaternions = [df.("orientation.x") df.("orientation.y") df.("orientation.z") df.("orientation.w")];
raw_data_euler =  quat2eul(quaternions(510:end,:));
raw_data_yaw = raw_data_euler(:,3);
unwrapped_raw_data_yaw  = unwrap(raw_data_yaw);

magnetic_field = [imu_matrix.MagneticField];
magnetic_x = [magnetic_field.X]';
magnetic_y = [magnetic_field.Y]';

magnetic_field_x = magnetic_x(510:end,1);
magnetic_field_y = magnetic_y(510:end,1);

magnetic_field_x_avg = (max(magnetic_field_x) - min(magnetic_field_y))/2; 
magnetic_field_y_avg = (max(magnetic_field_x) - min(magnetic_field_y))/2;

offset_x = (max(magnetic_field_x) + min(magnetic_field_x)) / 2;
offset_y = (max(magnetic_field_y) + min(magnetic_field_y)) / 2;

avg_magnetic_field = (magnetic_field_x_avg + magnetic_field_y_avg ) / 2;

scale_x = avg_magnetic_field / magnetic_field_x_avg;
scale_y = avg_magnetic_field / magnetic_field_y_avg;

corrected_mag_x = (magnetic_field_x - offset_x) * scale_x;
corrected_mag_y = (magnetic_field_y - offset_y) * scale_y;

yaw_from_mag_with_bias = zeros(length(corrected_mag_x),1); % High Bias

for i = 1:length(corrected_mag_x)
    yaw_from_mag_with_bias(i,1) = atan2(corrected_mag_x(i,1), corrected_mag_y(i,1));
end

angular_velocity = [imu_matrix.AngularVelocity];
angular_z = [angular_velocity.Z]';
angular_velocity_z = angular_z(510:end,1);



unwrapped_yaw_from_mag = unwrap(yaw_from_mag_with_bias);
yaw_from_mag = wrapToPi(unwrapped_yaw_from_mag - unwrapped_yaw_from_mag(1,1));
unwrapped_yaw_from_gyro = cumtrapz(time, angular_velocity_z);
yaw_from_gyro = wrapToPi(unwrapped_yaw_from_gyro); % High Drift

unwrapped_comp_yaw = zeros(length(yaw_from_gyro),1);

for i = 1:length(yaw_from_gyro)
    unwrapped_comp_yaw(i,1) = (0.99*(unwrapped_yaw_from_gyro(i,1)) + 0.01*unwrapped_yaw_from_mag(i,1));
end

unwrapped_comp_yaw = unwrapped_comp_yaw + unwrapped_raw_data_yaw(1,1);
complementaryfilter_yaw = wrapToPi(unwrapped_comp_yaw);

hold on
%plot(time, yaw_from_gyro, 'color','r');
%plot(time, yaw_from_mag, 'color','g');
plot(time, complementaryfilter_yaw, 'color','b');
plot(time, raw_data_yaw, 'color','m');
legend('Complementary filtered yaw', 'Yaw from quaternions(Ground Truth)')
grid on
title('Complementary filtered yaw vs Yaw from quaternions(Ground Truth)')
xlabel('Time(s)')
ylabel('Yaw(radians)')


%{
plot(corrected_mag_x, corrected_mag_y, 'color','r')
plot(magnetic_field_x, magnetic_field_y, 'color','b')
grid on
legend('Corrected Magnetometer reading', 'Raw Magnetometer reading')
title('Magnetometer data before and after the correction')
xlabel('Magnetic Field X')
ylabel('Magnetic Field Y')
%}





