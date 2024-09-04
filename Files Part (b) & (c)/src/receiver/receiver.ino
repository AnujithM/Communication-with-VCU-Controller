
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

ros::NodeHandle nh;

std_msgs::Int32MultiArray received_msg;
ros::Publisher pub("received_data", &received_msg);

void messageCb(const std_msgs::Int32MultiArray& msg) {
    received_msg = msg;  
    pub.publish(&received_msg);  
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("vehicle_data", messageCb);

void setup() {
    Serial.begin(57600);
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(pub);
}

void loop() {
    nh.spinOnce();
    delay(1);
}
