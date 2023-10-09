#include <TinyGPS++.h>
#include <math.h>

#include <Wire.h>
#include <QMC5883LCompass.h>

#include "./Definitions.h"

#include <Servo.h>
//******************************************************************************************************   
// Ultrasonic Sensons, Collision Avoidance
int trigPin[NUM_ULTRASONIC_SENSORS];  // Array of trigger pins
int echoPin[NUM_ULTRASONIC_SENSORS];  // Array of echo pins

long ultrasonic_duration, ultrasonic_cm;
//******************************************************************************************************                                                                  
// GPS Variables & Setup
int GPS_Course;       // variable to hold the gps's determined course to destination
int Number_of_SATS;   // variable to hold the number of satellites acquired
TinyGPSPlus gps;      // gps = instance of TinyGPS

bool arrived = false;
    
String location="";
float targetLatitude = 0; // = 28.59108000;
float targetLongitude = 0; // = -81.46820800;
int movementInstruction = 0;
//******************************************************************************************************
// Compass Variables & Setup
QMC5883LCompass compass;

//******************************************************************************************************
// Motor Variables & Setup
int steeringSpeed = 255;
int motorSpeed = 150;

unsigned long motorStartTime = 0;  // Variable to store the time when the steering command was triggered
bool steeringReleased = true;  // Flag to track whether the motor has been released
int steeringLocation = 0;  // Variable to track the steering location
unsigned long steeringRunDuration = 200;  // Threshold for steering

// Define flag variable for motor direction
bool motorDirectionForward = false;
bool motorDirectionReverse = false;

bool gradualSpeed = false;
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
  pinMode(REAR_MOTOR_IN1, OUTPUT);
  pinMode(REAR_MOTOR_IN2, OUTPUT);
  pinMode(REAR_MOTOR_ENA, OUTPUT);

  // Steering Axle
  pinMode(STEERING_MOTOR_IN3, OUTPUT);
  pinMode(STEERING_MOTOR_IN4, OUTPUT);
  pinMode(STEERING_MOTOR_ENB, OUTPUT);

  // Ultrasonic Sensors:
  pinMode(TRIG_PIN_FRONT_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_FRONT_RIGHT, INPUT);

  pinMode(TRIG_PIN_FRONT_LEFT, OUTPUT);
  pinMode(ECHO_PIN_FRONT_LEFT, INPUT);

  pinMode(TRIG_PIN_BACK, OUTPUT);
  pinMode(ECHO_PIN_BACK, INPUT);

  // Initialize trigger and echo pins for each sensor
  trigPin[0] = TRIG_PIN_FRONT_RIGHT;
  trigPin[1] = TRIG_PIN_FRONT_LEFT;
  trigPin[2] = TRIG_PIN_BACK;
  echoPin[0] = ECHO_PIN_FRONT_RIGHT;
  echoPin[1] = ECHO_PIN_FRONT_LEFT;
  echoPin[2] = ECHO_PIN_BACK;

}


void loop()
{
  // Check if it's time to stop the steering motor
  if (!steeringReleased && (millis() - motorStartTime >= steeringRunDuration)) {
    Serial.println("Release");
    steeringRelease();  // Stop the motor
    steeringReleased = true;  // Set the steeringReleased flag to true
  }

  // Gradually adjust motor speed based on time intervals
  if ((motorDirectionForward || motorDirectionReverse) && !gradualSpeed) {
    if (millis() - smoothStartTime < 1000) {
      Serial.println("1");
      analogWrite(REAR_MOTOR_ENA, 100);
    } 
    else if (millis() - smoothStartTime < 1500) {
      Serial.println("2");
      analogWrite(REAR_MOTOR_ENA, 125);
    } 
    else if (millis() - smoothStartTime< 2000) {
      Serial.println("3");
      analogWrite(REAR_MOTOR_ENA, 150);

      if (motorSpeed == 150) {
        gradualSpeed = true;
      }
    } 
    // Motor speed is 255
    if (millis() - smoothStartTime > 2000 && motorSpeed == 255){
      if (millis() - smoothStartTime < 2500) {
        Serial.println("4");
        analogWrite(REAR_MOTOR_ENA, 180);
      }
      else if (millis() - smoothStartTime < 3000) {
        Serial.println("5");
        analogWrite(REAR_MOTOR_ENA, 225);
      }
      else {
        Serial.println("6");
        analogWrite(REAR_MOTOR_ENA, motorSpeed);
        gradualSpeed = true;
      }
    }
  }

  // When we are in reverse, read the back sensor
  if (motorDirectionReverse) {
    // Read the back sensor
    float distance = readUltrasonicSensor(trigPin[2], echoPin[2]);
    Serial.print("Back Sensor: ");
    Serial.print(distance);
    Serial.println(" ultrasonic_cm");

    // Check if the back sensor reading is below the collision threshold
    if (distance < COLLISION_THRESHOLD) {
      // Call the stop function immediately
      stop();
    }
  } 
  // Forward
  else if (motorDirectionForward) {
    bool obstacleDetected = false;
    // Read the front sensors
    for (int i = 0; i < 2; i++) { // Loop through front sensors (0 and 1)
      float distance = readUltrasonicSensor(trigPin[i], echoPin[i]);
      Serial.print("Front Sensor ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(distance);
      Serial.println(" ultrasonic_cm");

      if (distance < COLLISION_THRESHOLD) {
        obstacleDetected = true;
        break; // Exit the loop early if an obstacle is detected
      }
    }

     // If an obstacle is detected by any front sensor, call the stop function
    if (obstacleDetected) {
      stop();
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
            forward();
          }
          break;
        case 2:
          // Reverse
          if (!motorDirectionReverse) {
            reverse();
          }
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

float readUltrasonicSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  pinMode(echoPin, INPUT);
  unsigned long ultrasonic_duration = pulseIn(echoPin, HIGH);

  // Convert the time into a distance
  float ultrasonic_cm = (ultrasonic_duration / 2) / 29.1; // Divide by 29.1 or multiply by 0.0343
  return ultrasonic_cm;
}