clear all
df_imu = readtable("moving_imu.csv", 'PreserveVariableNames', true);
df_gps = readtable("moving_gps.csv", 'PreserveVariableNames', true);
df_stationary_imu = readtable("stationary_imu.csv", 'PreserveVariableNames', true);

t_stationary_imu = df_stationary_imu.Time;
time_stationary_imu = t_stationary_imu - t_stationary_imu(1,1);
stationary_acceleration = df_stationary_imu.("linear_acceleration.x");
mean_acceleration_error = mean(stationary_acceleration);

t_gps = df_gps.Time;
time_gps = t_gps - t_gps(1,1);
velocity_x_gps = zeros(length(time_gps), 1);
easting = df_gps.("utm_easting.data");
northing = df_gps.("utm_northing.data");

for i = 1:length(time_gps)-1
    velocity_x_gps(i+1,1) = sqrt((easting(i+1,1) - easting(i,1) )^2 + ...
        (northing(i+1,1) - northing(i,1))^2)/(time_gps(i+1, 1) - time_gps(i,1));
end

t_imu = df_imu.Time;
time_imu = t_imu - t_imu(1,1);
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


hold on 
plot(time_imu, velocity_x_imu, 'color', 'red')
plot(time_imu, velocity_imu_with_error,'color', 'green' );
plot(time_gps, velocity_x_gps, 'color', 'black')
grid on
legend('Corrected Integrated Velocity', 'Integrated velocity', 'Velocity estimated from GPS')
title('Corrected Integrated Velocity vs Integrated velocity vs Velocity estimated from GPS')
xlabel('Time(s)')
ylabel('Velocity(m/s)')


