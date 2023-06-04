#include <TinyGPS++.h>
#include <math.h>
#include <Wire.h>
#include "HMC5883L.h"       

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

HMC5883L compass;     // HMC5883L compass(HMC5883L)
int16_t mx, my, mz;   // variables to store x,y,z axis from compass (HMC5883L)
int desired_heading;  // initialize variable - stores value for the new desired heading
int compass_heading;  // initialize variable - stores value calculated from compass readings
int compass_dev = 5;  // the amount of deviation that is allowed in the compass heading - Adjust as Needed

int Heading_A;        // variable to store compass heading
int Heading_B;        // variable to store compass heading in Opposite direction
int pass = 0;         // variable to store which pass the robot is on


void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);

  // Start the software serial port at the GPS's default baud (18, 19)
  Serial1.begin(9600);  // GPS
  Serial2.begin(9600);  // Bluetooth
  Serial.println("Mega up ");  // SERIAL PRINTS


  // Compass
  Wire.begin();   // Join I2C bus used for the HMC5883L compass
  compass.begin();  // initialize the compass (HMC5883L)
  compass.setRange(HMC5883L_RANGE_1_3GA); // Set measurement range  
  compass.setMeasurementMode(HMC5883L_CONTINOUS); // Set measurement mode  
  compass.setDataRate(HMC5883L_DATARATE_30HZ);  // Set data rate  
  compass.setSamples(HMC5883L_SAMPLES_8); // Set number of samples averaged  
  compass.setOffset(0, 0, 0); // Set calibration offset 


  Startup();
}


void loop()
{
  return 0;
}


