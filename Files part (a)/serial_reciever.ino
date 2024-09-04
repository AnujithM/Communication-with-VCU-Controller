int ledPin = 13;  
char dataType = ' ';
String receivedString = "";

void setup() {
  Serial.begin(9600);  
  pinMode(ledPin, OUTPUT);  
}

void loop() {
  if (Serial.available() > 0) {
    dataType = Serial.read();  

    if (dataType == 'I') {  
      while (Serial.available() <= 0);  
      int receivedData = Serial.read();  

      
      if (receivedData >= 0 && receivedData <= 255) {
        if (receivedData > 50) {
          digitalWrite(ledPin, HIGH);  
        } else {
          digitalWrite(ledPin, LOW);  
        }

        int modifiedData = receivedData + 10;  
        Serial.println(modifiedData);  
      }

    } else if (dataType == 'L') {  
      while (Serial.available() < 2);  
      unsigned int lowByte = Serial.read();  
      unsigned int highByte = Serial.read();  
      unsigned int receivedData = (highByte << 8) | lowByte;  

      
      Serial.println(receivedData);

    } else if (dataType == 'C') {  
      while (Serial.available() <= 0); 
      char receivedChar = Serial.read();  

      
      Serial.println(receivedChar);

    } else if (dataType == 'S') {  
      receivedString = "";  

      
      delay(100);

      
      while (Serial.available() > 0) {
        char c = Serial.read();
        receivedString += c;
        delay(5);  
      }

      
      Serial.print("I heard \"");
      Serial.print(receivedString);
      Serial.println("\"");

    } else if (dataType == 'A') {  
      String receivedArray = "";  

     
      delay(100);

      
      while (Serial.available() > 0) {
        int arrayElement = Serial.read();
        receivedArray += String(arrayElement) + " ";
        delay(5); 
      }

      
      receivedArray.trim();

      
      Serial.print("Array received: ");
      Serial.println(receivedArray);
    }
  }
}
