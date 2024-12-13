#include "Arduino.h"
#include <BleKeyboard.h>

BleKeyboard bleKeyboard;

#define RXD2 16
#define TXD2 17

QueueHandle_t weightQueue = NULL; 
String buffer = ""; 

// BLE task to send data
void BLE_task(void *pvParameter) {
  bleKeyboard.begin();

  while (true) {
    if (bleKeyboard.isConnected()) {
      String receivedValue;
      if (xQueueReceive(weightQueue, &receivedValue, portMAX_DELAY) == pdTRUE) {
        bleKeyboard.print(receivedValue);  // Send via BLE
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

void setup() {

  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);


  weightQueue = xQueueCreate(32, sizeof(String));  
  

  xTaskCreatePinnedToCore(
      BLE_task,           
      "BLE_keyboard",     
      4096,               
      NULL,               
      2,                  
      NULL,               
      0                   
  );
}

void loop() {

  if (Serial2.available()) {
    char receivedChar = Serial2.read(); 


    if (receivedChar != '\r' && receivedChar != '\n') {
      buffer += receivedChar; 

      if (buffer.startsWith("ST,GS,") && buffer.length() >= 16) {

        Serial.println(buffer + " Length: " + String(buffer.length()));
        String value = buffer.substring(7,14);  
        buffer = "";  

        Serial.println("Value: " + value);

        xQueueSend(weightQueue, &value, portMAX_DELAY);  
      }

    }
  }

  delay(100); 
}