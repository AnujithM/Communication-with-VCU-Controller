#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

// Publisher for received data with error status
std_msgs::String received_data_msg;
ros::Publisher pub_received_data("received_data", &received_data_msg);

// Function to calculate checksum based on the data
int calculate_checksum(const std_msgs::Int32MultiArray& msg) {
    int calculated_checksum = 0;
    for (int i = 0; i < 8; i++) {  // Sum the 8 data bytes (indexes 0 to 7)
        calculated_checksum += msg.data[i];
    }
    return calculated_checksum % 256;  // Apply modulo 256
}

// Function to detect if the data is jumbled by checking the pairwise differences
bool detect_jumbled_data(const std_msgs::Int32MultiArray& msg) {
    // Check if the differences between consecutive elements are not equal to 1
    for (int i = 0; i < 7; i++) {  // Iterate over the first 7 data points (indexes 0 to 6)
        if ((msg.data[i + 1] - msg.data[i]) != 1) {
            return true;  // Jumbled if the difference is not consistent (not 1)
        }
    }
    return false;  // No jumble detected
}

// Callback function for receiving data
void messageCb(const std_msgs::Int32MultiArray& msg) {
    int received_checksum = msg.data[8];  // The checksum is at index 8 (last element)
    int calculated_checksum = calculate_checksum(msg);  // Recalculate the checksum from data

    char status_message[100];  // Buffer size for the message
    bool checksum_error = false;
    bool jumbled_error = false;

    // Check for checksum mismatch
    if (calculated_checksum != received_checksum) {
        checksum_error = true;
    }

    // Check for jumbled data using pairwise difference consistency
    if (detect_jumbled_data(msg)) {
        jumbled_error = true;
    }

    // Create the appropriate error message
    if (checksum_error && jumbled_error) {
        snprintf(status_message, sizeof(status_message), "Checksum Mismatch and Jumbled Data Detected");
    } else if (checksum_error) {
        snprintf(status_message, sizeof(status_message), "Checksum Mismatch");
    } else if (jumbled_error) {
        snprintf(status_message, sizeof(status_message), "Jumbled Data Detected");
    } else {
        snprintf(status_message, sizeof(status_message), "Data OK");
    }

    // Publish the error or status message
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
