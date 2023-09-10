#include <TinyGPS++.h>
#include <math.h>
#include <AFMotor.h>

#include <Wire.h>
#include <QMC5883LCompass.h>

#include "./Definitions.h"

#include <Servo.h>

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


// Motor
AF_DCMotor rearLeftMotor(1);
AF_DCMotor rearRightMotor(2); 
AF_DCMotor steeringMotor(1);

int steeringSpeed = 150;
int motorSpeed = 255;
int crawlSteerThreshold = 230;
int negativeCrawlSteerThreshold = -230;


void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(115200);                                            // Serial 0 is for communication with the computer

  // Motor
  //AppMotor.DeviceDriverSet_Motor_Init();

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

    // Steering logic
    if (data.startsWith("C")) {
      int servoInput = data.substring(1).toInt();

      int steeringDifference = servoInput - 165;

      // Map the difference to the steering motor's speed
      int steeringInput = map(steeringDifference, -165, 165, -255, 255);

      // Set the steering motor speed
      steeringMotor.setSpeed(steeringSpeed);
      Serial.println("steeringInput");
      Serial.println(steeringInput);
      // Turn Right
      if (steeringInput > 0 && steeringInput < crawlSteerThreshold) {
        steeringMotor.run(FORWARD);
      }
      // Right Crawl
      else if (steeringInput > 0 && steeringInput > crawlSteerThreshold) {
        Serial.println("Super steer right");
      } 
      // Turn Left
      else if (steeringInput < 0 && steeringInput > negativeCrawlSteerThreshold) {
        steeringMotor.run(BACKWARD);
      }
      // Left Crawl
      else if (steeringInput < 0 && steeringInput < negativeCrawlSteerThreshold) {
        Serial.println("Super steer left");
      } 
      // Stop the motor
      else if (steeringInput == 0){
        steeringMotor.run(RELEASE); 
      }
    }

    // Motor speed adjustment
    if(data.startsWith("S")){
      motorSpeed = data.substring(1).toInt();
      Serial.println("Motor Speed = " + String(motorSpeed));
      // Stop the robot to reset the speed
      rearLeftMotor.run(RELEASE);
      rearRightMotor.run(RELEASE);
    }

    // Movement Instructions
    if(data.startsWith("M")){
      String instructionStr = data.substring(1); // Remove the "M" prefix
      movementInstruction = instructionStr.toInt(); // Convert to an integer
      //FrostRunnerMovement instruction = convertToMovement(movementInstruction);
      uint8_t i;
      switch (movementInstruction) {
        case 1:
          // Forward
          rearLeftMotor.run(FORWARD);
          rearRightMotor.run(FORWARD);
          rearLeftMotor.setSpeed(motorSpeed);
          rearRightMotor.setSpeed(motorSpeed);
          // Add your code to move forward here
          break;
        case 2:
          // Backward
	        rearLeftMotor.run(BACKWARD);
          rearRightMotor.run(BACKWARD);
          rearLeftMotor.setSpeed(motorSpeed);  
          rearRightMotor.setSpeed(motorSpeed);
          // Add your code to move backward here
          break;
        case 9:
          // Stop
          rearLeftMotor.run(RELEASE);
          rearRightMotor.run(RELEASE);
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
  }
}

