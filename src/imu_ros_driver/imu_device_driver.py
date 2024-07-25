#!/usr/bin/env python3
import rospy
import serial
from imu_ros_driver.msg import imu_message
from tf.transformations import quaternion_from_euler
import math

if __name__ == '__main__':
    serial_port = rospy.get_param('~port','/dev/ttyUSB0')
    serial_baudrate = rospy.get_param('~baudrate',115200)
    imu = serial.Serial(serial_port, serial_baudrate, timeout=None)
    pub=rospy.Publisher('imu_data', imu_message, queue_size=10)
    rospy.init_node('imu_node', anonymous=True)

    try:
        while not rospy.is_shutdown():
            line = imu.readline().decode()
            message_type = line[0:6]
            if line == '':
                rospy.logwarn("IMU: No data")
            else:
                if message_type == '$VNYMR':
                    try:
                        parts = line.split(",")

                        yaw = math.radians(float((parts[1]).split('*')[0])) #Convert to Radians
                        pitch = math.radians(float((parts[2]).split('*')[0])) #Convert to Radians
                        roll = math.radians(float((parts[3]).split('*')[0])) #Convert to Radians
                        quaternions = quaternion_from_euler(roll, pitch, yaw) #Yaw = Z , Roll = X, Pitch = Y

                        magnetic_X = float((parts[4]).split('*')[0])
                        magnetic_Y = float((parts[5]).split('*')[0])
                        magnetic_Z = float((parts[6]).split('*')[0])

                        acceleration_X = float((parts[7]).split('*')[0])
                        acceleration_Y = float((parts[8]).split('*')[0])
                        acceleration_Z = float((parts[9]).split('*')[0])

                        gyro_X = float((parts[10]).split('*')[0])
                        gyro_Y = float((parts[11]).split('*')[0])
                        gyro_Z = float((parts[12]).split('*')[0])

                        message = imu_message()
                        message.header.frame_id = "IMU_Data"
                        message.header.stamp = rospy.Time.now()
                        

                        message.orientation.x = quaternions[0]
                        message.orientation.y = quaternions[1]
                        message.orientation.z = quaternions[2]
                        message.orientation.w = quaternions[3]
                        
                        message.magnetic_field.x = magnetic_X*0.0001 #Gauss to Tesla
                        message.magnetic_field.y = magnetic_Y*0.0001 #Gauss to Tesla
                        message.magnetic_field.z = magnetic_Z*0.0001 #Gauss to Tesla
                        
                        message.acceleration.x = acceleration_X 
                        message.acceleration.y = acceleration_Y
                        message.acceleration.z = acceleration_Z
                        
                        message.gyro.x = gyro_X
                        message.gyro.y = gyro_Y
                        message.gyro.z = gyro_Z
                        
                        rospy.loginfo(message)
                        pub.publish(message)          


                    except:
                        rospy.logwarn("Data exception: "+line)
                        continue
    except  rospy.ROSInterruptException:
            imu.close()


