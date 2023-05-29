#include <TinyGPS++.h>
#include <math.h>

// Create a TinyGPS++ object
TinyGPSPlus gps;

String location="";
String longitude = "";
String latitude = "";
double targetLatitude = 28.59108000;
double targetLongitude = -81.46820800;

int flag = 0;
int Number_of_SATS =0;

void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);

  // Start the software serial port at the GPS's default baud (18, 19)
  Serial1.begin(9600);  // GPS
  Serial2.begin(9600);  // Bluetooth
  Serial.print("Mega up");  // SERIAL PRINTS

  Startup();
}


void loop()
{
  return 0;
}


