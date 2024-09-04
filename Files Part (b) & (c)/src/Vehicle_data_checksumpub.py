#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray

# Define the original data frame
original_data = [101, 102, 103, 104, 105, 106, 107, 108]

# The fixed checksum for the original data
fixed_checksum = sum(original_data) % 256  # This is 68 for the original data

def publish_data():
    rospy.init_node('vehicle_data_publisher', anonymous=True)
    
    pub_data = rospy.Publisher('vehicle_data', Int32MultiArray, queue_size=10)
    
    rate = rospy.Rate(1)  # Publish data every second

    while not rospy.is_shutdown():
        # Case 1: Correct Data (Original Order)
        frame_correct = original_data + [fixed_checksum]

        # Publish Correct Data
        frame_msg_correct = Int32MultiArray()
        frame_msg_correct.data = frame_correct
        pub_data.publish(frame_msg_correct)
        rospy.loginfo("Published Correct Data")

        # Sleep for 1 second
        rate.sleep()

        # Case 2: Jumbled Data (Actual Case 1)
        jumbled_data_case_1 = [101, 102, 103, 104, 105, 107, 108, 106]  # Slightly jumbled data
        frame_jumbled_1 = jumbled_data_case_1 + [fixed_checksum]

        # Publish Actual Case 1 (Slightly Jumbled Data)
        frame_msg_jumbled_1 = Int32MultiArray()
        frame_msg_jumbled_1.data = frame_jumbled_1
        pub_data.publish(frame_msg_jumbled_1)
        rospy.loginfo("Published Actual Case 1 (Jumbled Data)")

        # Sleep for 1 second
        rate.sleep()

        # Case 3: Jumbled Data (Actual Case 2)
        jumbled_data_case_2 = [105, 106, 107, 108, 101, 102, 103, 100]  # Severely jumbled and corrupted data
        frame_jumbled_2 = jumbled_data_case_2 + [fixed_checksum]

        # Publish Actual Case 2 (Severely Jumbled Data)
        frame_msg_jumbled_2 = Int32MultiArray()
        frame_msg_jumbled_2.data = frame_jumbled_2
        pub_data.publish(frame_msg_jumbled_2)
        rospy.loginfo("Published Actual Case 2 (Severely Jumbled Data)")

        # Sleep for 1 second
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_data()
    except rospy.ROSInterruptException:
        pass
