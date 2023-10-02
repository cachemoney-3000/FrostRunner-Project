#include <TinyGPS++.h>
#include <math.h>

#include <Wire.h>
#include <QMC5883LCompass.h>

#include "./Definitions.h"

#include <Servo.h>

//******************************************************************************************************                                                                  
// GPS Variables & Setup
int GPS_Course;       // variable to hold the gps's determined course to destination
int Number_of_SATS;   // variable to hold the number of satellites acquired
TinyGPSPlus gps;      // gps = instance of TinyGPS

bool arrived = false;
//******************************************************************************************************     
String location="";
float targetLatitude = 0; // = 28.59108000;
float targetLongitude = 0; // = -81.46820800;
int movementInstruction = 0;
//******************************************************************************************************
// Compass Variables & Setup
QMC5883LCompass compass;

// Motor
int steeringSpeed = 255;
int motorSpeed = 150;

unsigned long motorStartTime = 0;  // Variable to store the time when the steering command was triggered
unsigned long steeringRunDuration = 180;  // Threshold for steering
bool steeringReleased = true;  // Flag to track whether the motor has been released
int steeringLocation = 0;  // Variable to track the steering location

// XY 160D Rear Motors
const int IN1 = 5;
const int IN2 = 4;
const int ENA = 6;

// XY 160D Steering Axle Motor
const int IN3 = 8;
const int IN4 = 7;
const int ENB = 9;

// Define flag variable for motor direction
bool motorDirectionForward = false;
bool motorDirectionReverse = false;

bool gradualSpeed = false;

// Smooth Start
unsigned long smoothStartTime = 0;

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

  // Rear Motors
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Steering Axle
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
}


void loop()
{
  // Check if it's time to stop the motor
  if (!steeringReleased && (millis() - motorStartTime >= steeringRunDuration)) {
    Serial.println("Release");
    steeringRelease();  // Stop the motor
    steeringReleased = true;  // Set the steeringReleased flag to true
  }

  // Gradually adjust motor speed based on time intervals
  if ((motorDirectionForward || motorDirectionReverse) && !gradualSpeed) {
    if (millis() - smoothStartTime < 1000) {
      Serial.println("1");
      analogWrite(ENA, 100);
    } 
    else if (millis() - smoothStartTime < 1500) {
      Serial.println("2");
      analogWrite(ENA, 125);
    } 
    else if (millis() - smoothStartTime < 2000) {
      Serial.println("3");
      analogWrite(ENA, 150);

      if (motorSpeed == 150) {
        gradualSpeed = true;
      }
    } 
    // Motor speed is 255
    if (millis() - smoothStartTime > 2000 && motorSpeed == 255){
      if (millis() - smoothStartTime < 2500) {
        Serial.println("4");
        analogWrite(ENA, 180);
      }
      else if (millis() - smoothStartTime < 3000) {
        Serial.println("5");
        analogWrite(ENA, 225);
      }
      else {
        Serial.println("6");
        analogWrite(ENA, motorSpeed);
        gradualSpeed = true;
      }
    }
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
          if (steeringLocation > -1) {
            steerLeft(false);
          }
          break;
        case 4:
          // Right
          if (steeringLocation < 1) {
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
      // Set the new motor speed
      motorSpeed = data.substring(1).toInt();
      Serial.println("Motor Speed = " + String(motorSpeed));
      // Stop the robot to reset the speed
      motorDirectionForward = false;
      motorDirectionReverse = false;
      stop();
    }

    // Movement Instructions
    if(data.startsWith("M")){
      String instructionStr = data.substring(1); // Remove the "M" prefix
      movementInstruction = instructionStr.toInt(); // Convert to an integer

      switch (movementInstruction) {
        case 1:
          // Forward
          if (!motorDirectionForward) {
            forward(motorSpeed);
            motorDirectionForward = true;
            motorDirectionReverse = false;
          }
          break;
        case 2:
          // Reverse
          if (!motorDirectionReverse) {
            reverse(motorSpeed);
            motorDirectionReverse = true;
            motorDirectionForward = false;
          }
          break;
        case 9:
          // Stop
          stop();
          motorDirectionForward = false;
          motorDirectionReverse = false;
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

        // Only re-run this part of code if we have new longitude and latitude values
        /* if (newTargetLatitude != targetLatitude || newTargetLongitude != targetLongitude) {
          targetLatitude = newTargetLatitude;
          targetLongitude = newTargetLongitude;

          Location phoneLoc;
          phoneLoc.latitude = targetLatitude;
          phoneLoc.longitude = targetLongitude;

          Serial.println("Target Longitude: " + String(targetLongitude, 8));
          Serial.println("Target Latitude: " + String(targetLatitude, 8));

          driveTo(phoneLoc, 25);
        } */

        targetLatitude = newTargetLatitude;
          targetLongitude = newTargetLongitude;

          Location phoneLoc;
          phoneLoc.latitude = targetLatitude;
          phoneLoc.longitude = targetLongitude;

          Serial.println("Target Longitude: " + String(targetLongitude, 8));
          Serial.println("Target Latitude: " + String(targetLatitude, 8));

          driveTo(phoneLoc, 25);
      }
    }
  }
}