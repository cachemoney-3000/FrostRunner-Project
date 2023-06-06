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
double targetLatitude = 28.59108000;
double targetLongitude = -81.46820800;
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
  return 0;
}


