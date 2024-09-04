#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray


original_data = [101, 102, 103, 104, 105, 106, 107, 108]


fixed_checksum = sum(original_data) % 256  # This is 68 for the original data

def publish_data():
    rospy.init_node('vehicle_data_publisher', anonymous=True)
    
    pub_data = rospy.Publisher('vehicle_data', Int32MultiArray, queue_size=10)
    
    rate = rospy.Rate(1) 

    while not rospy.is_shutdown():
        
        frame_correct = original_data + [fixed_checksum]

        
        frame_msg_correct = Int32MultiArray()
        frame_msg_correct.data = frame_correct
        pub_data.publish(frame_msg_correct)
        rospy.loginfo("Published Correct Data")

        
        rate.sleep()

        
        jumbled_data_case_1 = [101, 102, 103, 104, 105, 107, 108, 106]  
        frame_jumbled_1 = jumbled_data_case_1 + [fixed_checksum]

        
        frame_msg_jumbled_1 = Int32MultiArray()
        frame_msg_jumbled_1.data = frame_jumbled_1
        pub_data.publish(frame_msg_jumbled_1)
        rospy.loginfo("Published Actual Case 1 (Jumbled Data)")

        
        rate.sleep()

        
        jumbled_data_case_2 = [105, 106, 107, 108, 101, 102, 103, 100]  
        frame_jumbled_2 = jumbled_data_case_2 + [fixed_checksum]

        
        frame_msg_jumbled_2 = Int32MultiArray()
        frame_msg_jumbled_2.data = frame_jumbled_2
        pub_data.publish(frame_msg_jumbled_2)
        rospy.loginfo("Published Actual Case 2 (Severely Jumbled Data)")

        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_data()
    except rospy.ROSInterruptException:
        pass
