#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray
import random

def publish_data():
    rospy.init_node('vehicle_data_publisher', anonymous=True)
    pub = rospy.Publisher('vehicle_data', Int32MultiArray, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz corresponds to 100ms

    while not rospy.is_shutdown():
        data = Int32MultiArray()
        data.data = [
            random.randint(0, 255),  # X Position
            random.randint(0, 255),  # Y Position
            random.randint(0, 255),  # X Linear Velocity
            random.randint(0, 255),  # Y Linear Velocity
            random.randint(0, 255),  # X Angular Velocity
            random.randint(0, 255),  # Y Angular Velocity
            random.randint(0, 255),  # Random Number 1
            random.randint(0, 255)   # Random Number 2
        ]
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_data()
    except rospy.ROSInterruptException:
        pass
