clear all
df_imu = readtable("moving_imu.csv", 'PreserveVariableNames', true);
df_gps = readtable("moving_gps.csv", 'PreserveVariableNames', true);
df_stationary_imu = readtable("stationary_imu.csv", 'PreserveVariableNames', true);
t_stationary_imu = df_stationary_imu.Time;
time_stationary_imu = t_stationary_imu - t_stationary_imu(1,1);
stationary_acceleration = df_stationary_imu.("linear_acceleration.x");
mean_acceleration_error = mean(stationary_acceleration);
stdv_acceleration_error = std(stationary_acceleration);

t_gps = df_gps.Time;
time_gps = t_gps - t_gps(1,1);
velocity_x_gps = zeros(length(time_gps), 2);
easting = df_gps.("utm_easting.data");
northing = df_gps.("utm_northing.data");

for i = 1:length(time_gps)-1
    velocity_x_gps(i+1,1) = sqrt((easting(i+1,1) - easting(i,1) )^2 + ...
        (northing(i+1,1) - northing(i,1))^2)/(time_gps(i+1, 1) - time_gps(i,1));
end

t_imu = df_imu.Time;
time_imu = t_imu - t_imu(1,1);
mag_x_moving = df_imu.("magnetic_field.x");
mag_y_moving = df_imu.("magnetic_field.y");

%%%%%%%%%%%%%% yaw
bag = rosbag('circles.bag');
imu_topic_data = select(bag, "Time",[bag.StartTime bag.EndTime], "Topic", "/imutopic");
imustructs = readMessages(imu_topic_data, "DataFormat", "struct");
imu_matrix = cell2mat(imustructs);

df = readtable("circles.csv", 'PreserveVariableNames',true);
t = df.Time - df.Time(510,1);
time = t(510:end,1);

quaternions = [df_imu.("orientation.x") df_imu.("orientation.y") df_imu.("orientation.z") df_imu.("orientation.w")];
raw_data_euler =  quat2eul(quaternions);
raw_data_yaw = raw_data_euler(:,3);
unwrapped_raw_data_yaw = wrapToPi(raw_data_yaw);


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

corrected_mag_x = (mag_x_moving - offset_x) * scale_x;
corrected_mag_y = (mag_y_moving - offset_y) * scale_y;

yaw_from_mag_with_bias = zeros(length(corrected_mag_x),1); % High Bias

for i = 1:length(corrected_mag_x)
    yaw_from_mag_with_bias(i,1) = atan2(corrected_mag_x(i,1), corrected_mag_y(i,1));
end

angular_velocity_z = df_imu.("angular_velocity.z");

unwrapped_yaw_from_mag = unwrap(yaw_from_mag_with_bias);
yaw_from_mag = wrapToPi(unwrapped_yaw_from_mag - unwrapped_yaw_from_mag(1,1));
unwrapped_yaw_from_gyro = cumtrapz(time_imu, angular_velocity_z);
yaw_from_gyro = wrapToPi(unwrapped_yaw_from_gyro); % High Drift

unwrapped_comp_yaw = zeros(length(yaw_from_gyro),1);

for i = 1:length(yaw_from_gyro)
    unwrapped_comp_yaw(i,1) = (0.99*(unwrapped_yaw_from_gyro(i,1)) + 0.01*unwrapped_yaw_from_mag(i,1));
end

unwrapped_comp_yaw = unwrapped_comp_yaw + unwrapped_raw_data_yaw(1,1);
complementaryfilter_yaw = wrapToPi(unwrapped_comp_yaw);

%%%%%%%%%%%%%% yaw

acceleration_x = df_imu.("linear_acceleration.x") - mean_acceleration_error;


start = 1;
for ind = 200:200:length(acceleration_x)
    if std(acceleration_x(start:ind,1)) < 0.147
        acceleration_x(start:ind,1) = 0;
    else
        acceleration_x(start:ind,1) = acceleration_x(start:ind,1) ;
    end
    start = start+200;
end

velocity_imu_with_error = cumtrapz(time_imu, df_imu.("linear_acceleration.x"));
velocity_x_imu= cumtrapz(time_imu, acceleration_x);

start2 =1;
error = 0;

for ind2 = 19:19:length(acceleration_x)

    if std(velocity_x_imu(start2:ind2,1)) < 0.0001
        error = mean(velocity_x_imu(start2:ind2,1));
        velocity_x_imu(start2:ind2,1) = 0;

    else
        velocity_x_imu(start2:ind2,1) = velocity_x_imu(start2:ind2,1) - error;
    end 
    start2 = start2+19;
end

for l = 1:length(velocity_x_imu)
    if velocity_x_imu(l,1) < 0
        velocity_x_imu(l,1) = 0;
    end
end

acceleration_y = df_imu.("linear_acceleration.y");
gyro_z = df_imu.("angular_velocity.z");

acceleration_y_gyro_velocity_x = gyro_z.*velocity_x_imu ; 

v_easting = zeros(length(time_imu),1);
v_northing = zeros(length(time_imu),1);

for vec = 1:length(time_imu)
    v_easting(vec,1) = (velocity_x_imu(vec,1))*(cos(complementaryfilter_yaw(vec,1) - 1.45)); % use yaw_from_mag and 1.73 instead of complementaryfilter_yaw for yaw from magnetometer
    v_northing(vec,1) = -(velocity_x_imu(vec,1))*(sin(complementaryfilter_yaw(vec,1) - 1.45)); % use yaw_from_mag and 1.73 instead of complementaryfilter_yaw for yaw from magnetometer
end

x_easting = (cumtrapz(time_imu, v_easting));
x_northing = (cumtrapz(time_imu, v_northing));

scale1 = sqrt((easting(118,1)-easting(1,1))^2 + (northing(118,1)-northing(1,1))^2)/sqrt((x_easting(4681,1)-x_easting(1,1))^2 + (x_northing(4681,1)-x_northing(1,1))^2);
x_easting(1:end,1) = x_easting(1:end,1)*scale1 + easting(1,1);
x_northing(1:end,1) = x_northing(1:end,1)*scale1 + northing(1,1);

plot(x_easting, x_northing, 'color','r')
hold on 
plot(easting, northing, 'color','b')
grid on
legend('(easting,northing) obtained from velocity and Complementary filtered yaw', '(easting,northing) obtained from GPS', 'Location','northwest')
title('(easting,northing) obtained from velocity and Complementary filtered yaw vs GPS')
xlabel('Easting')
ylabel('Northing')


gyro_zdot = diff(gyro_z);
xc = linsolve(-gyro_zdot, (gyro_z(2:40147,1).*velocity_x_imu(2:40147,1)));


%{
hold on
plot(time_imu, acceleration_y)
plot(time_imu, acceleration_y_gyro_velocity_x)
grid on
legend('Acceleration Y observed', '\omega*VelocityX')
title('Acceleration Y observed vs Acceleration obtained by multiplying angular velocity with velocity in X')
xlabel('Time(s)')
ylabel('Acceleration(m/s^2)')
%}



