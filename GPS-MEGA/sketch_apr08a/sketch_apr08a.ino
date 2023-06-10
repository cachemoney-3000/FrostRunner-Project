#include <TinyGPS++.h>
#include <math.h>

#include <Wire.h>
#include <QMC5883LCompass.h>

#include "./Definitions.h"

//******************************************************************************************************                                                                  
// GPS Variables & Setup
int GPS_Course;       // variable to hold the gps's determined course to destination
int Number_of_SATS;   // variable to hold the number of satellites acquired
TinyGPSPlus gps;      // gps = instance of TinyGPS


Waypoint waypoint;
int frontIndex = 0;
int rearIndex = -1;
int waypointCounter = 0;

int bluetoothReadFlag = 0;


//******************************************************************************************************     
String location="";
double targetLatitude; // = 28.59108000;
double targetLongitude; // = -81.46820800;
//******************************************************************************************************
// Compass Variables & Setup
QMC5883LCompass compass;


void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(115200);                                            // Serial 0 is for communication with the computer


  // Start the software serial port at the GPS's default baud (18, 19)
  Serial1.begin(9600);  // GPS
  Serial2.begin(9600);  // Bluetooth
  Serial.println("Mega up ");  // SERIAL PRINTS

  Wire.begin();
  compass.init(); // Initialize the Compass.
  
  Startup();  // Startup procedure
  
}


void loop()
{
  while (Serial2.available() > 0){
    char receivedChar = Serial2.read();

    if (receivedChar == '/') {
      if (location != "") {
        // Split the location string into longitude and latitude
        int separatorIndex = location.indexOf('\n');
        if (separatorIndex != -1) {
          String longitudeString = location.substring(0, separatorIndex);
          String latitudeString = location.substring(separatorIndex + 1);

          // Convert latitude and longitude to float values
          targetLatitude = latitudeString.toDouble();
          targetLongitude = longitudeString.toDouble();
        }

        Serial.println("Target Longitude: " + String(targetLongitude, 8));
        Serial.println("Target Latitude: " + String(targetLatitude, 8));

        location = ""; // Reset the location buffer
        
        break; // End transmission
      }
    }
    else {
      location += receivedChar;
    }
  }

  Location phoneLoc;
  phoneLoc.latitude = targetLatitude.toDouble();
  phoneLoc.longitude = targetLongitude.toDouble();

  driveTo(phoneLoc, GPS_WAYPOINT_TIMEOUT);
}

void driveTo(struct GeoLoc &loc, int timeout) {
  Serial1.listen();
  GeoLoc robotLoc = getGPS();
  Serial2.listen();

  if (robotLoc.lat != 0 && robotLoc.lon != 0) {
    float distance = 0;
    //Start move loop here
    do {
      Serial1.listen();
      robotLoc = getGPS();
      Serial2.listen();
      
      // TODO geoDistance
      distance = geoDistance(robotLoc, loc);
      // TODO geoBearing, heoHeading
      float bearing = geoBearing(robotLoc, loc) - geoHeading();
      
      Serial.print("Distance: ");
      Serial.println(distance);
    
      Serial.print("Bearing: ");
      Serial.println(geoBearing(robotLoc, loc));

      Serial.print("Heading: ");
      Serial.println(geoHeading());
      
      // TODO drive
      drive(distance, bearing);
      timeout -= 1;
    } while (distance > 1.0 && timeout > 0);

    // TODO stop
    stop();
  }
}


