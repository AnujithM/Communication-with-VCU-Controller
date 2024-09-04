int ledPin = 13;  // Pin for the built-in LED (Usually pin 13 on most Arduino boards)
char dataType = ' ';
String receivedString = "";

void setup() {
  Serial.begin(9600);  // Initialize UART with 9600 baud rate
  pinMode(ledPin, OUTPUT);  // Set the LED pin as an output
}

void loop() {
  if (Serial.available() > 0) {
    dataType = Serial.read();  // Read the data type indicator ('I', 'L', 'C', 'S', or 'A')

    if (dataType == 'I') {  // Indicates that the following data is an 8-bit integer
      while (Serial.available() <= 0);  // Wait until the actual data arrives
      int receivedData = Serial.read();  // Read the 8-bit integer data

      // Process the 8-bit integer
      if (receivedData >= 0 && receivedData <= 255) {
        if (receivedData > 50) {
          digitalWrite(ledPin, HIGH);  // Turn on the LED
        } else {
          digitalWrite(ledPin, LOW);  // Turn off the LED
        }

        int modifiedData = receivedData + 10;  // Modify the data (e.g., add 10)
        Serial.println(modifiedData);  // Send the modified data back to the PC
      }

    } else if (dataType == 'L') {  // Indicates that the following data is a 16-bit integer
      while (Serial.available() < 2);  // Wait until the full 16-bit integer data arrives
      unsigned int lowByte = Serial.read();  // Read the low byte
      unsigned int highByte = Serial.read();  // Read the high byte
      unsigned int receivedData = (highByte << 8) | lowByte;  // Combine the two bytes into a 16-bit unsigned integer

      // Echo the 16-bit integer back to the PC
      Serial.println(receivedData);

    } else if (dataType == 'C') {  // Indicates that the following data is a character
      while (Serial.available() <= 0);  // Wait until the actual data arrives
      char receivedChar = Serial.read();  // Read the character data

      // Echo the character back to the PC
      Serial.println(receivedChar);

    } else if (dataType == 'S') {  // Indicates that the following data is a string
      receivedString = "";  // Clear the previous string

      // Wait a bit to ensure the full string has been received
      delay(100);

      // Read the incoming string
      while (Serial.available() > 0) {
        char c = Serial.read();
        receivedString += c;
        delay(5);  // Small delay to allow all characters to be read
      }

      // Echo the string back with a custom message
      Serial.print("I heard \"");
      Serial.print(receivedString);
      Serial.println("\"");

    } else if (dataType == 'A') {  // Indicates that the following data is an array
      String receivedArray = "";  // Initialize a string to store the received array

      // Wait for array elements to arrive
      delay(100);

      // Read each byte in the array and append it to the string
      while (Serial.available() > 0) {
        int arrayElement = Serial.read();
        receivedArray += String(arrayElement) + " ";
        delay(5);  // Small delay to ensure all elements are read
      }

      // Trim any trailing spaces from the string
      receivedArray.trim();

      // Echo the array back as a string with the same order
      Serial.print("Array received: ");
      Serial.println(receivedArray);
    }
  }
}
