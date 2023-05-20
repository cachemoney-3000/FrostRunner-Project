#include "Arduino.h"
#include <SoftwareSerial.h>

const byte rxPin = 19;
const byte txPin = 18;
SoftwareSerial BTSerial(rxPin, txPin); // RX TX

void setup() {
  // define pin modes for tx, rx:
  Serial2.begin(9600);
  Serial.begin(9600);
}

String messageBuffer = "";
String message = "";

void loop() {
  int state = '2';
  if(Serial2.available() > 0){
    state = Serial2.read();
  }

  if(state == '0'){
    Serial.println("OFF");
    Serial2.println("Sent:0");
    state = 0;
  }
  else if(state == '1'){
    Serial.println("ON");
    Serial2.println("Sent:1");
    state = 0;
  }
}
