#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;


std_msgs::String received_data_msg;
ros::Publisher pub_received_data("received_data", &received_data_msg);


int calculate_checksum(const std_msgs::Int32MultiArray& msg) {
    int calculated_checksum = 0;
    for (int i = 0; i < 8; i++) {  
        calculated_checksum += msg.data[i];
    }
    return calculated_checksum % 256;  
}


bool detect_jumbled_data(const std_msgs::Int32MultiArray& msg) {
    
    for (int i = 0; i < 7; i++) {  
        if ((msg.data[i + 1] - msg.data[i]) != 1) {
            return true;  
        }
    }
    return false;  
}


void messageCb(const std_msgs::Int32MultiArray& msg) {
    int received_checksum = msg.data[8];  
    int calculated_checksum = calculate_checksum(msg); 

    char status_message[100];  
    bool checksum_error = false;
    bool jumbled_error = false;

    
    if (calculated_checksum != received_checksum) {
        checksum_error = true;
    }

    
    if (detect_jumbled_data(msg)) {
        jumbled_error = true;
    }

    
    if (checksum_error && jumbled_error) {
        snprintf(status_message, sizeof(status_message), "Checksum Mismatch and Jumbled Data Detected");
    } else if (checksum_error) {
        snprintf(status_message, sizeof(status_message), "Checksum Mismatch");
    } else if (jumbled_error) {
        snprintf(status_message, sizeof(status_message), "Jumbled Data Detected");
    } else {
        snprintf(status_message, sizeof(status_message), "Data OK");
    }

    
    received_data_msg.data = status_message;
    pub_received_data.publish(&received_data_msg);
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("vehicle_data", messageCb);

void setup() {
    Serial.begin(57600);
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(pub_received_data);
}

void loop() {
    nh.spinOnce();
    delay(1);
}
