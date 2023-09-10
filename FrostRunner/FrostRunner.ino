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

int steeringSpeed = 255;
int motorSpeed = 255;

unsigned long motorStartTime = 0;  // Variable to store the time when the steering command was triggered
unsigned long steeringRunDuration = 150;  // Threshold for steering
bool steeringReleased = true;  // Flag to track whether the motor has been released
int steeringLocation = 0;  // Variable to track the steering location


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
  //Startup();  // Startup procedure
}


void loop()
{
  // Check if it's time to stop the motor
  if (!steeringReleased && (millis() - motorStartTime >= steeringRunDuration)) {
    Serial.println("Release");
    steeringMotor.run(RELEASE);  // Stop the motor
    steeringReleased = true;  // Set the steeringReleased flag to true
  }

  while (Serial2.available() > 0){
    String data = Serial2.readStringUntil('\n');
    Serial.println(data);

    // Steering logic
    if (data.startsWith("C")) {
      int servoInput = data.substring(1).toInt();

      switch (servoInput) {
        case 3:
          // Left
          if (steeringLocation > -2) {
            steerLeft(false);
          }
          break;
        case 4:
          if (steeringLocation < 2) {
            steerRight(false);
          }
          break;
        default:
          Serial.println("Invalid Movement Instruction");
          break;
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
          forward();
          break;
        case 2:
          // Reverse
          reverse();
          break;
        case 9:
          // Stop
          stop();
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


void steerLeft(boolean stop){
  // Left
  steeringLocation--;
  steeringMotor.run(FORWARD);
  steeringMotor.setSpeed(motorSpeed);
  motorStartTime = millis();
  steeringReleased = false;

  if(stop){
    Serial.print("steeringLocation before: ");
    Serial.println(steeringLocation);
    steeringRunDuration = (abs(steeringLocation) * 150) + 150;
    Serial.println("steeringRunDuration");
    Serial.println(steeringRunDuration);
  }
  else {
    steeringRunDuration = 150;
  }

  Serial.println("SteerLocation");
  Serial.println(steeringLocation);
}

void steerRight(boolean stop) {
   // Right
  steeringLocation++;
  steeringMotor.run(BACKWARD);
  steeringMotor.setSpeed(motorSpeed);  
  motorStartTime = millis();
  steeringReleased = false;

  if(stop){
    Serial.print("steeringLocation before: ");
    Serial.println(steeringLocation);
    steeringRunDuration = (abs(steeringLocation) * 150) + 150;
    Serial.println("steeringRunDuration");
    Serial.println(steeringRunDuration);
  }
  else {
    steeringRunDuration = 150;
  }

  Serial.println("SteerLocation");
  Serial.println(steeringLocation);
}

void forward() {
  // Forward
  rearLeftMotor.run(FORWARD);
  rearRightMotor.run(FORWARD);
  rearLeftMotor.setSpeed(motorSpeed);
  rearRightMotor.setSpeed(motorSpeed);
}

void reverse() {
  // Backward
  rearLeftMotor.run(BACKWARD);
  rearRightMotor.run(BACKWARD);
  rearLeftMotor.setSpeed(motorSpeed);  
  rearRightMotor.setSpeed(motorSpeed);
}

void stop() {
  rearLeftMotor.run(RELEASE);
  rearRightMotor.run(RELEASE);
  // Straighten the wheels
  if (steeringLocation > 0) {
    steerLeft(true);
    steeringLocation = 0;
  }
  if (steeringLocation < 0) {
    steerRight(true);
    steeringLocation = 0;
  }

  Serial.println("SteerLocation");
  Serial.println(steeringLocation);
}