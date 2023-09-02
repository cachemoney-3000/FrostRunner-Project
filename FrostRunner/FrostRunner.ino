#include <TinyGPS++.h>
#include <math.h>

#include <Wire.h>
#include <QMC5883LCompass.h>

#include "./Definitions.h"

#include <avr/wdt.h>
#include "Motor_DeviceDriverSet.h"
#include "Motor_AppFunctionSet.cpp"

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

int movementInstruction = 0;

//******************************************************************************************************     
String location="";
float targetLatitude = 0; // = 28.59108000;
float targetLongitude = 0; // = -81.46820800;
//******************************************************************************************************
// Compass Variables & Setup
QMC5883LCompass compass;

// Motor controls
DeviceDriverSet_Motor AppMotor;
Movement FrostRunner_Movements;

void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(115200);                                            // Serial 0 is for communication with the computer

  // Motor
  AppMotor.DeviceDriverSet_Motor_Init();

  // Start the software serial port at the GPS's default baud (18, 19)
  Serial1.begin(9600);  // GPS
  Serial2.begin(9600);  // Bluetooth
  Serial.println("Mega up ");  // SERIAL PRINTS

  Wire.begin();
  compass.init(); // Initialize the Compass.
  //Startup();  // Startup procedure
}


void loop()
{
  while (Serial2.available() > 0){
    String data = Serial2.readStringUntil('\n');
    Serial.println(data);

    // Movement Instructions
    if(data.startsWith("M")){
      String instructionStr = data.substring(1); // Remove the "M" prefix
      movementInstruction = instructionStr.toInt(); // Convert to an integer
      FrostRunnerMovement instruction = convertToMovement(movementInstruction);

      switch (movementInstruction) {
        case 1:
          // Forward
          Serial.println("Moving Forward");
          MovementInstruction(instruction /*direction*/, 255 /*speed*/);
          // Add your code to move forward here
          break;
        case 2:
          // Backward
          Serial.println("Moving Backward");
          MovementInstruction(instruction /*direction*/, 255 /*speed*/);
          // Add your code to move backward here
          break;
        case 3:
          // Left
          Serial.println("Turning Left");
          MovementInstruction(instruction /*direction*/, 255 /*speed*/);
          // Add your code to turn left here
          break;
        case 4:
          // Right
          Serial.println("Turning Right");
          MovementInstruction(instruction /*direction*/, 255 /*speed*/);
          // Add your code to turn right here
          break;
        case 5:
          // LeftForward
          Serial.println("Moving Left Forward");
          MovementInstruction(instruction /*direction*/, 255 /*speed*/);
          // Add your code to move left forward here
          break;
        case 6:
          // LeftBackward
          Serial.println("Moving Left Backward");
          MovementInstruction(instruction /*direction*/, 255 /*speed*/);
          // Add your code to move left backward here
          break;
        case 7:
          // RightForward
          Serial.println("Moving Right Forward");
          MovementInstruction(instruction /*direction*/, 255 /*speed*/);
          // Add your code to move right forward here
          break;
        case 8:
          // RightBackward
          Serial.println("Moving Right Backward");
          MovementInstruction(instruction /*direction*/, 255 /*speed*/);
          // Add your code to move right backward here
          break;
        case 9:
          // Stop
          Serial.println("Stopping");
          MovementInstruction(instruction /*direction*/, 255 /*speed*/);
          // Add your code to stop here
          break;
        default:
          Serial.println("Invalid Movement Instruction");
          break;
      }
    }
    // Summon Instructions, Get the GPS coordinate from user's phone
    else {
      int separatorIndex = data.indexOf('/');
      // Split the location string into longitude and latitude
      if (separatorIndex != -1) {
        String longitudeStr = data.substring(0, separatorIndex);
        String latitudeStr = data.substring(separatorIndex + 1);

        // Convert latitude and longitude to float values
        float newTargetLatitude = latitudeStr.toDouble();
        float newTargetLongitude = longitudeStr.toDouble();

        if (newTargetLatitude != targetLatitude || newTargetLongitude != targetLongitude) {
          targetLatitude = newTargetLatitude;
          targetLongitude = newTargetLongitude;

          Location phoneLoc;
          phoneLoc.latitude = targetLatitude;
          phoneLoc.longitude = targetLongitude;

          Serial.println("Target Longitude: " + String(targetLongitude, 8));
          Serial.println("Target Latitude: " + String(targetLatitude, 8));
        }
      }
    }
    /* if (receivedChar == '/') {
      if (location != "") {
        // Split the location string into longitude and latitude
        int separatorIndex = location.indexOf('\n');
        if (separatorIndex != -1) {
          String longitudeString = location.substring(0, separatorIndex);
          String latitudeString = location.substring(separatorIndex + 1);

          // Convert latitude and longitude to float values
          float newTargetLatitude = latitudeString.toDouble();
          float newTargetLongitude = longitudeString.toDouble();

          if (newTargetLatitude != targetLatitude || newTargetLongitude != targetLongitude) {
            targetLatitude = newTargetLatitude;
            targetLongitude = newTargetLongitude;

            Serial.println("Target Longitude: " + String(targetLongitude, 8));
            Serial.println("Target Latitude: " + String(targetLatitude, 8));

            location = ""; // Reset the location buffer
          }
        }

        break; // End transmission
      }
    }
    else {
      location += receivedChar;
    } */
  }

  /* if (targetLatitude != 0 && targetLongitude != 0){
    Location phoneLoc;
    phoneLoc.latitude = targetLatitude;
    phoneLoc.longitude = targetLongitude;

    driveTo(phoneLoc, GPS_WAYPOINT_TIMEOUT);
  } */
}

