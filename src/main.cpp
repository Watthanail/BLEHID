#include "Arduino.h"
#include <BleKeyboard.h>

BleKeyboard bleKeyboard("Mini01_BLE");

#define RXD2 16 //16
#define TXD2 17  //17
#define BLE_STATUS_PIN 32

QueueHandle_t weightQueue = NULL; 
String buffer = ""; 

// BLE task to send data
bool wasConnected = false;
void BLE_task(void *pvParameter) {
    bleKeyboard.begin();

    while (true) {
        if (bleKeyboard.isConnected()) {
            if (!wasConnected) {
                Serial.println("BLE connected.");
                wasConnected = true;
                digitalWrite(BLE_STATUS_PIN,HIGH);
            }

            String receivedValue;
            if (xQueueReceive(weightQueue, &receivedValue, portMAX_DELAY) == pdTRUE) {
                Serial.println("Sending via BLE: " + receivedValue);
                bleKeyboard.print(receivedValue);  // Send via BLE
            }
        } else {
            if (wasConnected) {
                Serial.println("BLE disconnected.");
                wasConnected = false;
                digitalWrite(BLE_STATUS_PIN,LOW);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); 
    }
}

void setup() {

  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  pinMode(BLE_STATUS_PIN,OUTPUT);
  digitalWrite(BLE_STATUS_PIN,LOW);


  
    // Start BLE keyboard
    // bleKeyboard.begin();
    // Serial.println("BLE Keyboard initialized with security settings.");

    weightQueue = xQueueCreate(32, sizeof(String));  

    xTaskCreatePinnedToCore(
        BLE_task,           // BLE task
        "BLE_keyboard",     // Task name
        4096,               // Stack size
        NULL,               // Parameter
        2,                  // Priority
        NULL,               // Task handle
        0                   // Core ID
    );
}

void loop() {
  // if (Serial2.available()) {
  while (Serial2.available()) {
    uint32_t decimalValue = Serial2.read();
    // Serial.println(decimalValue);
    if (decimalValue != 10 && decimalValue != 13) {  // Ignore newlines
      char asciiChar = (char)decimalValue;
      buffer += asciiChar;
      Serial.println(buffer + " length: " + String(buffer.length()));
      if (buffer.startsWith("ST,GS,") && buffer.length() >= 17) {
        Serial.println(buffer + " length: " + String(buffer.length()));
        String value = buffer.substring(7, 14);
        buffer = "";
        Serial.println("Value: " + value);

        // Send to BLE task
        xQueueSend(weightQueue, &value, portMAX_DELAY);  
      }
    }
  }

  delay(100); 
}

// #include "Arduino.h"
// #include <BleKeyboard.h>
// #include <HardwareSerial.h>

// BleKeyboard bleKeyboard("Mini01_BLE");

// #define RXD2   (20)
// #define TXD2   (21)

// HardwareSerial MySerial(0);

// QueueHandle_t weightQueue = NULL; 
// String buffer = ""; 

// // BLE task to send data
// bool wasConnected = false;
// void BLE_task(void *pvParameter) {
//     bleKeyboard.begin();

//     while (true) {
//         if (bleKeyboard.isConnected()) {
//             if (!wasConnected) {
//                 Serial.println("BLE connected.");
//                 wasConnected = true;
//             }

//             String receivedValue;
//             if (xQueueReceive(weightQueue, &receivedValue, portMAX_DELAY) == pdTRUE) {
//                 Serial.println("Sending via BLE: " + receivedValue);
//                 bleKeyboard.print(receivedValue);  // Send via BLE
//             }
//         } else {
//             if (wasConnected) {
//                 Serial.println("BLE disconnected.");
//                 wasConnected = false;
//             }
//         }
//         vTaskDelay(10 / portTICK_PERIOD_MS); 
//     }
// }

// void setup() {

//   Serial.begin(115200);
//   MySerial.begin( 9600, SERIAL_8N1, RXD2, TXD2 );


  
//     // Start BLE keyboard
//     // bleKeyboard.begin();
//     // Serial.println("BLE Keyboard initialized with security settings.");

//     weightQueue = xQueueCreate(32, sizeof(String));  

//     xTaskCreatePinnedToCore(
//         BLE_task,           // BLE task
//         "BLE_keyboard",     // Task name
//         4096,               // Stack size
//         NULL,               // Parameter
//         2,                  // Priority
//         NULL,               // Task handle
//         0                   // Core ID
//     );
// }

// void loop() {
//   // if (Serial2.available()) {
//   while (MySerial.available()) {
//     uint32_t decimalValue = MySerial.read();
//     // Serial.println(decimalValue);
//     if (decimalValue != 10 && decimalValue != 13) {  // Ignore newlines
//       char asciiChar = (char)decimalValue;
//       buffer += asciiChar;
//       Serial.println(buffer + " length: " + String(buffer.length()));
//       if (buffer.startsWith("ST,GS,") && buffer.length() >= 17) {
//         Serial.println(buffer + " length: " + String(buffer.length()));
//         String value = buffer.substring(7, 14);
//         buffer = "";
//         Serial.println("Value: " + value);

//         // Send to BLE task
//         xQueueSend(weightQueue, &value, portMAX_DELAY);  
//       }
//     }
//   }

//   delay(100); 
// }



