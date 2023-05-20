#include <TinyGPS++.h>

// Create a TinyGPS++ object
TinyGPSPlus gps;

void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);

  // Start the software serial port at the GPS's default baud (18, 19)
  Serial1.begin(9600);  // GPS
  Serial2.begin(9600);  // Bluetooth
  Serial.print("Mega up");  // SERIAL PRINTS
}

String location="";
String longitude = "";
String latitude = "";
int flag = 0;
void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial1.available() > 0){
    if (gps.encode(Serial1.read())){
      //Serial.println(gps.encode(Serial1.read()));
      //displayInfo();
    } 
    if (Serial2.available() > 0 && flag == 0) {
    char receivedChar = Serial2.read();

    if (receivedChar == '/') {
        flag = 1;
        if (location != "") {
          // Split the location string into longitude and latitude
          int separatorIndex = location.indexOf('\n');
          if (separatorIndex != -1) {
              longitude = location.substring(0, separatorIndex);
              latitude = location.substring(separatorIndex + 1);
          }
  
          Serial.println("Longitude: " + longitude);
          Serial.println("Latitude: " + latitude);
  
          location = ""; // Reset the location buffer
          flag = 0; // Reset the flag
        }
    } else {
        location += receivedChar;
    }
}
    
  }

  
 

  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected");
    while(true);
  }
}

void displayInfo()
{
  if (gps.location.isValid())
  {
    Serial2.print("Latitude: ");
    Serial2.println(gps.location.lat(), 6);
    Serial2.print("Longitude: ");
    Serial2.println(gps.location.lng(), 6);
    Serial2.print("Altitude: ");
    Serial2.println(gps.altitude.meters());
  }
  else
  {
    Serial2.println("Location: Not Available");
  }
  
  Serial2.print("Date: ");
  if (gps.date.isValid())
  {
    Serial2.print(gps.date.month());
    Serial2.print("/");
    Serial2.print(gps.date.day());
    Serial2.print("/");
    Serial2.println(gps.date.year());
  }
  else
  {
    Serial2.println("Not Available");
  }

  Serial2.print("Time: ");
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial2.print(F("0"));
    Serial2.print(gps.time.hour());
    Serial2.print(":");
    if (gps.time.minute() < 10) Serial2.print(F("0"));
    Serial2.print(gps.time.minute());
    Serial2.print(":");
    if (gps.time.second() < 10) Serial2.print(F("0"));
    Serial2.print(gps.time.second());
    Serial2.print(".");
    if (gps.time.centisecond() < 10) Serial2.print(F("0"));
    Serial2.println(gps.time.centisecond());
  }
  else
  {
    Serial2.println("Not Available");
  }

  Serial2.println();
  Serial2.println();
  delay(3000);
}
