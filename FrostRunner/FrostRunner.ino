#include <TinyGPS++.h>
#include <math.h>

#include <Wire.h>
#include <QMC5883LCompass.h>
#include <Servo.h>

#include "DHT.h"
#include <avr/wdt.h>

#include "./Definitions.h"

//******************************************************************************************************   
// Temperature
DHT dht(DHTPIN, DHTTYPE);

unsigned long lastTemperatureTime = 0; // Variable to track the last temperature send time
unsigned long temperatureInterval = 5000; // Interval for sending temperature data (5 seconds)
//******************************************************************************************************   
// Ultrasonic Sensons, Collision Avoidance
int trigPin[NUM_ULTRASONIC_SENSORS];  // Array of trigger pins
int echoPin[NUM_ULTRASONIC_SENSORS];  // Array of echo pins

long ultrasonic_duration, ultrasonic_cm;

boolean followEnabled = false;
const unsigned long sensorReadInterval = 500; // Interval to read sensors (in milliseconds)
unsigned long lastSensorReadTime = 0; // Variable to store the last time sensors were read
//******************************************************************************************************                                                                  
// GPS Variables & Setup
int Number_of_SATS;   // variable to hold the number of satellites acquired
TinyGPSPlus gps;      // gps = instance of TinyGPS

// Self Driving Variables
int movementInstruction = 0;
bool selfDrivingInProgress = false;
int globalTimeout = GLOBAL_SELF_DRIVING_TIMEOUT;
Location phoneLoc;
//******************************************************************************************************
// Compass Variables & Setup
QMC5883LCompass compass;

//******************************************************************************************************
// Motor Variables & Setup
int steeringSpeed = 255;
int motorSpeed = 255;

unsigned long motorStartTime = 0;  // Variable to store the time when the steering command was triggered

// Steering
bool steeringReleased = true;  // Flag to track whether the motor has been released
int steeringLocation = 0;  // Variable to track the steering location
unsigned int steeringRunDuration = STEERING_TIME_THRESHOLD;  // Threshold for steering

// Define flag variable for motor direction
bool motorDirectionForward = false;
bool motorDirectionReverse = false;

// Self Driving
bool isVehicleTurning = false;
float targetLatitude = 0; // = 28.59108000;
float targetLongitude = 0; // = -81.46820800;

void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(115200);                                            // Serial 0 is for communication with the computer

  // Start the software serial port at the GPS's default baud (18, 19)
  Serial1.begin(9600);  // GPS
  Serial2.begin(9600);  // Bluetooth, Communication with Phone 
  

  Wire.begin();
  compass.init(); // Initialize the Compass.
  

  Serial.println("Mega up ");  // SERIAL PRINTS

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

  // Temperature
  dht.begin();

  compass.setSmoothing(3,true);  

  wdt_enable(WDTO_2S); // Initialize the watchdog timer with a 1-second timeout
}

String result = "";
void loop()
{
  compass.read(); // Keep reading the compass
  wdt_reset(); // Prevent the reset
  /**
   * Steering release
   * Check if it's time to stop the steering motor
  */
  if (!steeringReleased && (millis() - motorStartTime >= steeringRunDuration)) {
    //Serial.println("Release");
    steeringRelease();  // Stop the motor
    steeringReleased = true;  // Set the steeringReleased flag to true
  }

  /**
   * Object avoidance logic
   * 
   */
  unsigned long currentTime = millis();
  if (!selfDrivingInProgress && motorDirectionReverse) {
    lastSensorReadTime = currentTime;
    // Read the back sensor
    float distance = readUltrasonicSensor(trigPin[2], echoPin[2]);
    /* Serial.print("Back Sensor: ");
    Serial.print(distance);
    Serial.println(" ultrasonic_cm"); */

    // Check if the back sensor reading is below the collision threshold
    if (distance < COLLISION_THRESHOLD) {
      // Call the stop function immediately
      stop();
      Serial2.println("Obstacle Detected!");
    }
  } 
  // Forward
  else if (!selfDrivingInProgress && motorDirectionForward) {
    lastSensorReadTime = currentTime;
    bool obstacleDetected = false;
    // Read the front sensors
    for (int i = 0; i < 2; i++) { // Loop through front sensors (0 and 1)
      float distance = readUltrasonicSensor(trigPin[i], echoPin[i]);
      /* Serial.print("Front Sensor ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(distance);
      Serial.println(" ultrasonic_cm"); */

      if (distance < COLLISION_THRESHOLD) {
        Serial.print("Collision detected at");
        Serial.print(i + 1);
        obstacleDetected = true;
        break; // Exit the loop early if an obstacle is detected
      }
    }
    // If an obstacle is detected by any front sensor, call the stop function
    if (obstacleDetected) {
      stop();
      Serial2.println("Obstacle Detected!");
    }
  }

  
  /**
   * Bluetooth communication, controls
   * 
   */
  while (Serial2.available() > 0 || selfDrivingInProgress){
    wdt_reset(); // Prevent the reset
    String data = Serial2.readStringUntil('\n');
    Serial.println(data);
    // Trigger a watchdog timer reset
    if(data.startsWith("R")){
      stop();
      straightenWheel();
      delay(3000); // Force a reset
    }

    if(selfDrivingInProgress){
      Serial.println("SELF DRIVING IN PROGRESS");
      checkForObstacle();
      driveTo(phoneLoc);
    }

    // Movement Instructions
    if(data.startsWith("M")){
      String instructionStr = data.substring(1); // Remove the "M" prefix
      movementInstruction = instructionStr.toInt(); // Convert to an integer

      switch (movementInstruction) {
        case 1:
          // Forward
          if (!motorDirectionForward  && !selfDrivingInProgress) {
            forward(motorSpeed);
          }
          break;

        case 2:
          // Reverse
          if (!motorDirectionReverse  && !selfDrivingInProgress) {
            reverse(motorSpeed);
          }
          break;

        case 9:
          // Stop
          stop();
          if (selfDrivingInProgress){
            selfDrivingInProgress = false;
            globalTimeout = GLOBAL_SELF_DRIVING_TIMEOUT;
            isVehicleTurning = false;
          }
          break;

        default:
          break;
      }
    }
    // Steering logic
    else if (data.startsWith("C") && !selfDrivingInProgress) { 
      int servoInput = data.substring(1).toInt();

      switch (servoInput) {
        case 3:
          // Left
          if (steeringLocation > -1) {
            steeringLocation = steerLeft(false, steeringLocation);
          }
          break;
        case 4:
          // Right
          if (steeringLocation < 1) {
            steeringLocation = steerRight(false, steeringLocation);
          }
          break;
        default:
          break;
      }
    }

    if (data.length() > 0){
      // Motor speed adjustment
      if(data.startsWith("S") && !selfDrivingInProgress){
        // Set the new motor speed
        motorSpeed = data.substring(1).toInt();
        Serial.println("Motor Speed = " + String(motorSpeed));
        // Stop the robot to reset the speed
        motorDirectionForward = false;
        motorDirectionReverse = false;
        stop();
      }

      // Temperature
      else if(data.startsWith("P")){
        // Call the function to read temperature in Fahrenheit
        float temperatureFahrenheit = readTemperatureFahrenheit();
        int temperatureInteger = int(floor(temperatureFahrenheit));
        // Print the temperature to the primary serial port (for debugging)
        Serial.print(F("Temperature (Fahrenheit): "));
        Serial.println(temperatureInteger);

        // Send the temperature data to Phone
        Serial2.println("T" + String(temperatureInteger));
      }

      // Steering Threshold Adjustment
      /* else if(data.startsWith("T")  && !selfDrivingInProgress){
        // Set the new motor speed
        steeringRunDuration += data.substring(1).toInt();
        // Limit the range of steeringRunDuration to 50-300
        steeringRunDuration = constrain(steeringRunDuration, 50, 300);
        Serial.println("Steering Speed = " + String(steeringRunDuration));

        Serial2.println(steeringRunDuration);
        // Stop the robot to reset the steering threshold
        motorDirectionForward = false;
        motorDirectionReverse = false;
        stop();
      } */

      // Summon Instructions, Get the GPS coordinate from user's phone
      else if(data.startsWith("X")){
        Startup();  // Startup GPS Procedure

        Location loc = getGPS();
        bool locationAcquired = loc.latitude != 0 && loc.longitude != 0;

        String receivedCoordinates = data.substring(1);
        int separatorIndex = receivedCoordinates.indexOf('/');
        
        Serial.println(" Longitude: " + String(loc.longitude , 8));
        Serial.println(" Latitude: " + String(loc.latitude, 8)); 
        // Split the location string into longitude and latitude
        if (!selfDrivingInProgress && locationAcquired && separatorIndex != -1) {
          Serial2.println(String(Number_of_SATS) + " Satellites Acquired");     

          String longitudeStr = data.substring(1, separatorIndex);
          String latitudeStr = data.substring(separatorIndex + 2);

          // Convert latitude and longitude to float values
          float newTargetLatitude = latitudeStr.toFloat();
          float newTargetLongitude = longitudeStr.toFloat();

          phoneLoc.latitude = newTargetLatitude;
          phoneLoc.longitude = newTargetLongitude;

          //phoneLoc.latitude = 28.59109411244622;
          //phoneLoc.longitude = -81.46732786360467;

          // Apply smoothing to try to increase the precision of the coordinates
          phoneLoc = applyMovingAverageFilter(phoneLoc);

          Serial.println("Target Longitude: " + String(phoneLoc.longitude , 8));
          Serial.println("Target Latitude: " + String(phoneLoc.latitude, 8));

          selfDrivingInProgress = true;
          driveTo(phoneLoc);
        }
        else {
          Serial2.println("No Satellites Found");   
        }
      }
    }
  }
}

