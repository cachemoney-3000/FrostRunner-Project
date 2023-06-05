#include <TinyGPS++.h>
#include <math.h>
#include <Wire.h>
#include <QMC5883LCompass.h>    

//******************************************************************************************************                                                                  
// GPS Variables & Setup
int GPS_Course;       // variable to hold the gps's determined course to destination
int Number_of_SATS;   // variable to hold the number of satellites acquired
TinyGPSPlus gps;      // gps = instance of TinyGPS

unsigned long distanceFromUser;
int arrayCounter = 0;
int waypointCounter = 0;
double userLatitudeArray[10];
double userLongitudeArray[10];

//******************************************************************************************************     
String location="";
String longitude = "";
String latitude = "";
double targetLatitude = 28.59108000;
double targetLongitude = -81.46820800;
//******************************************************************************************************
// Compass Variables & Setup

QMC5883LCompass  compass;     // HMC5883L compass(HMC5883L)
int calibrationData[3][2];
bool changed = false;
bool done = false;
int t = 0;
int c = 0;


void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(115200);                                            // Serial 0 is for communication with the computer


  // Start the software serial port at the GPS's default baud (18, 19)
  Serial1.begin(9600);  // GPS
  Serial2.begin(9600);  // Bluetooth
  Serial.println("Mega up ");  // SERIAL PRINTS

  
  Wire.begin();
  // Initialize the Compass.
  compass.init();

  Startup();
}


void loop()
{
  
  return 0;
}


