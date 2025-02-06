# Part (a): Establishing communication from PC to Microcontroller

You need to send data from the PC to the microcontroller via UART and show the output in any of the following ways:

- Display the change of variables in a watch window in a debugger (if available for the microcontroller you possess).
- Use a built-in LED / external LED / external display (e.g., LCD 16x2 display) to show the data received from the computer (e.g., if data > 50, glow LED 1).
- If none of the above is possible, read the data via UART and send it back to the PC with some modifications (e.g., if 50 is received, send the data back to the PC after adding 10 to it).

---
### Data Types
The data that the PC is sending to the microcontroller should work for different **types of data** as shown below:

- **(A)** Int of size 8 bits (e.g., `128, 255`)
- **(B)** Char (e.g., `'A'`, `'Z'`)
- **(C)** String (e.g., `"Hi there, welcome to AIRL!!!"`)
- **(D)** Array of Integers (e.g., `[100, 60, 123, 30]`)
- **(E)** Int of size 16 bits (e.g., `30212, 723, 9393`)

### Implementation
Write a Python or C++ Script (running on PC) to send the serial data. Data of each type can be sent at a time, or can be sent all at once to display on the microcontroller end. The varying data must be sent at a rate of **500ms** from the PC.

> **Note:** In **Task ‘3(D)’ (Array of Integers)**, the order in which data is sent must be preserved and maintained in the same sequence on the microcontroller side.

# Part (b): Integrating with ROS

Now that we are able to communicate with the PC using a personalized script, the next task is to integrate the microcontroller with ROS to send the data from a ROS node to the microcontroller via UART. This would then be used by the control algorithm built by your colleague running on the same MCU.  

You have the option to use **ROS Serial** or **PySerial** or **C++ Serial** for this task.

Create a ROS node that publishes the random data structured in the following fashion at a rate of 100ms.
### **Array of 8 Integers of 8-bit size:**

| X Position of Vehicle | Y Position of Vehicle | X Linear vel of Vehicle | Y Linear vel of Vehicle | X Angular vel of Vehicle | Y Angular vel of Vehicle | Random Number (Saved for future use) | Random Number (Saved for future use) |
|-----------------------|----------------------|-------------------------|-------------------------|--------------------------|--------------------------|--------------------------------------|--------------------------------------|

**Figure 2: Communication Frame Format**

Write firmware for the microcontroller to either subscribe to the ROS topics or use UART directly and receive the data.  
Display the received data as an array of integers, display each integer sequentially on the external display, or on a second serial port or a debugger window.

---

# Part (c): Solving the Jumble (Optional)

While stress testing your algorithm, you realized that there is a need to ensure robustness in the process as the numbers in the array were getting jumbled. You observed that the order of the integers is changing, and some bytes are getting missed occasionally.  

For example, the **X Angular Velocity** of the vehicle is found in the **3rd byte**, and the **Y Linear Velocity** in the **4th byte**. To rectify this, you decide to implement a **feedback mechanism** where the microcontroller sends an acknowledgment back to the ROS node after it successfully receives and processes the data. The acknowledgment signal could be derived from the processed data.

