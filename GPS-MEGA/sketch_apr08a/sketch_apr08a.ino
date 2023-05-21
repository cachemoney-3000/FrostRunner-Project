#include <TinyGPS++.h>
#include <math.h>

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
float targetLatitude = 0.0;
float targetLongitude = 0.0;

int flag = 0;

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial1.available() > 0){
    if (gps.encode(Serial1.read())){
      //Serial.println(gps.encode(Serial1.read()));
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
    
                // Convert latitude and longitude to float values
                targetLatitude = latitude.toFloat();
                targetLongitude = longitude.toFloat();
              }
      
              Serial.println("Target Longitude: " + String(targetLongitude, 8));
              Serial.println("Target Latitude: " + String(targetLatitude, 8));
      
              location = ""; // Reset the location buffer
              flag = 0; // Reset the flag

              displayInfo();
            }
          }
          else {
            location += receivedChar;
          }
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
    float currentLatitude = gps.location.lat();
    float currentLongitude = gps.location.lng();

    
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 8);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 8);
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());

    // Calculate distance and heading
    float distance = calculateDistance(currentLatitude, currentLongitude, targetLatitude, targetLongitude);

    Serial.print("Distance to target: ");
    Serial.print(distance, 2);
    Serial.println(" feet");
  }
  else
  {
    Serial.println("Location: Not Available");
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

  Serial.println();
  Serial.println();
  delay(3000);
}


float calculateDistance(float lat1, float lon1, float lat2, float lon2)
{
    // Convert latitude and longitude to radians
    float latRad1 = radians(lat1);
    float lonRad1 = radians(lon1);
    float latRad2 = radians(lat2);
    float lonRad2 = radians(lon2);

    // Earth's radius in feet
    float radius = 20902230; // Approximate value for average Earth radius in feet

    // Calculate differences in latitude and longitude
    float deltaLat = latRad2 - latRad1;
    float deltaLon = lonRad2 - lonRad1;

    // Haversine formula to calculate distance
    float a = sin(deltaLat / 2) * sin(deltaLat / 2) +
              cos(latRad1) * cos(latRad2) * sin(deltaLon / 2) * sin(deltaLon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    float distance = radius * c;

    return distance;
}
