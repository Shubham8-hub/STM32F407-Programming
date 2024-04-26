#include "ArduinoJson.h"

#define STM32_RX_PIN 16
#define STM32_TX_PIN 17
#define UART_BAUD_RATE 115200

HardwareSerial STM32Serial(1);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(UART_BAUD_RATE);
  STM32Serial.begin(UART_BAUD_RATE, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
}

void loop() {
  // put your main code here, to run repeatedly:

  // This while loop is working Fine : Solution 1

  // while (STM32Serial.available())
  // {
  //   char receivedChar = STM32Serial.read();
  //   Serial.print(receivedChar);
  //   // Serial.println("Data received from STM32: " + receivedChar);
  // }

  // solution 2
  // while (STM32Serial.available())
  // {
  //   char receivedChar = STM32Serial.read();
  //   String receivedStr = String(receivedChar);
  //   Serial.println("Data received from STM32: " + receivedStr);
  // }
  
 while (STM32Serial.available()) {
    // Read the parameters from STM32Serial
    uint8_t receivedParams[10]; // Adjust size as needed
    STM32Serial.readBytesUntil('\n', receivedParams, sizeof(receivedParams)); // Read until newline character

    // Parse received parameters
    char name[5];
    memcpy(name, receivedParams, 4); // Copy name
    name[4] = '\0'; // Null terminate
    uint16_t rollNo;
    memcpy(&rollNo, &receivedParams[4], sizeof(rollNo)); // Copy roll number
    uint8_t subject1 = receivedParams[7]; // Subject 1
    uint8_t subject2 = receivedParams[8]; // Subject 2

    // Construct JSON string
    StaticJsonDocument
    <100> doc;
    doc["name"] = name;
    doc["rollNo"] = rollNo;
    doc["subject1"] = subject1;
    doc["subject2"] = subject2;

    // Serialize JSON document to a string
    char jsonBuffer[100];
    size_t len = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

    // Print JSON string
    Serial.println(jsonBuffer);
  }

//   /***************************************/
//   String receivedData = "";
// while (STM32Serial.available()) {
//   char receivedChar = STM32Serial.read();
//   if (receivedChar == '\n') {
//     // A newline character was encountered, process the accumulated data
//     Serial.println("Received data: " + receivedData);
//     receivedData = "";  // Clear the buffer
//   } else {
//     // Accumulate the received character
//     receivedData += receivedChar;
//   }
// }
  delay(1000);
}
