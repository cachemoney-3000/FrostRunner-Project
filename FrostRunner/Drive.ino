// Go to the location specified by phoneLoc (latitude, longitude)
void driveTo(struct Location &phoneLoc) {
  Location robotLoc = getGPS();
  //robotLoc.latitude = 28.59108000;
  //robotLoc.longitude = -81.46820800;

  if (robotLoc.latitude != 0 && robotLoc.longitude != 0) {
    float distance = gps.distanceBetween(phoneLoc.latitude, phoneLoc.longitude, robotLoc.latitude, robotLoc.longitude);
    robotLoc = getGPS(); // Get robot coordinates
    //robotLoc.latitude = 28.59108000;
    //robotLoc.longitude = -81.46820800;

    /** Calculate the azimuths */
    byte locationAzimuth = calculateAzimuth(robotLoc, phoneLoc);
    compass.read();
    byte compassAzimuth = compass.getAzimuth();
    Serial.println("Azimuth Loc = " + String(locationAzimuth) + " Azimuth Compass =" + String(compassAzimuth));
    
    drive(distance, locationAzimuth, compassAzimuth);

    globalTimeout -= 1; // Decrement timeout

    if ((distance > 1000.0 && distance < 1.0) || globalTimeout == 0){
      Serial.println("Self Driving: TIMEOUT");
      stop();
      selfDrivingInProgress = false;
      isVehicleTurning = false;
      globalTimeout = GLOBAL_SELF_DRIVING_TIMEOUT;
    }
  }
}

// Convert degrees to radians
double toRadians(double degrees) {
  return degrees * (3.14159265358979323846 / 180.0);
}

// Calculate the azimuth between two GPS coordinates and return it as a byte
byte calculateAzimuth(struct Location &robotLoc, struct Location &phoneLoc) {
  phoneLoc.latitude = toRadians(phoneLoc.latitude);
  phoneLoc.longitude = toRadians(phoneLoc.longitude);
  robotLoc.latitude = toRadians(robotLoc.latitude);
  robotLoc.longitude = toRadians(robotLoc.longitude);

  double deltaLon = robotLoc.longitude - phoneLoc.longitude;

  double y = sin(deltaLon) * cos(robotLoc.latitude);
  double x = cos(phoneLoc.latitude) * sin(robotLoc.latitude) - sin(phoneLoc.latitude) * cos(robotLoc.latitude) * cos(deltaLon);

  double azimuth = atan2(y, x);

  // Convert radians to degrees
  azimuth = fmod((azimuth * 180.0 / PI + 360), 360);

  // Convert the azimuth to a byte (0-255)
  byte azimuthByte = static_cast<byte>(azimuth * 255 / 360);

  return azimuthByte;
}

void drive(float distance, byte locationAzimuth, byte compassAzimuth) {
  // For the delay timer
  unsigned long startTime = millis(); // Store the start time
  unsigned long delayTime = SELF_DRIVING_STEERING_DELAY; 

  int headingTolerance = 30;
  float distanceTolerance = 1.0;

  // Normalize azimuthDifference to the range [-180, 180] degrees
  int azimuthDifference = locationAzimuth - compassAzimuth;

  // Ensure azimuthDifference is within [0, 360] range
  if (azimuthDifference < 0) {
    azimuthDifference += 360;
  }

  // Check if you are facing the correct direction
  if (azimuthDifference <= headingTolerance || azimuthDifference >= 360 - headingTolerance) {
    stop();

    Serial.println("Facing the correct direction: " + String(azimuthDifference));
    isVehicleTurning = false;
    if (steeringLocation != 0){
      straightenWheel();
    }
  } 
  else {
    stop();

    if (azimuthDifference < 180) {
      // Steer Right
      Serial.println("Steer Right: " + String(azimuthDifference));
      steeringLocation = steerRight(false, steeringLocation);
    }
    else {
      // Steer Left
      Serial.println("Steer Left: " + String(azimuthDifference));
      steeringLocation = steerLeft(false, steeringLocation);
    }
    forward(SELF_DRIVING_FORWARD_SPEED);
    isVehicleTurning = true;

    unsigned long currentTime = millis(); // Get the current time
    // Add a 1.5 delay timer
    while (currentTime - startTime < delayTime) {
      currentTime = millis(); // Update the current time
      checkForObstacle();
    }

    Serial.println("Delay done");
    stop();
  }
  
  // Move forward and reverse
  if (!isVehicleTurning) {
      // Move forward or backward based on distance
      if (distance > distanceTolerance) {
          Serial.println("Self Driving: Forward");
          forward(SELF_DRIVING_FORWARD_SPEED);
      } 
      else if (distance < -distanceTolerance) {
          Serial.println("Self Driving: Reverse");
          reverse(SELF_DRIVING_FORWARD_SPEED);
      } 
      else {
          Serial.println("Self Driving: Destination Reached");
          stop();
          selfDrivingInProgress = false;
            isVehicleTurning = false;
            globalTimeout = GLOBAL_SELF_DRIVING_TIMEOUT;
      }
  }
}

void checkForObstacle() {
    if (motorDirectionReverse) {
          // Read the back sensor
          float distance = readUltrasonicSensor(trigPin[2], echoPin[2]);
          // Check if the back sensor reading is below the collision threshold
          if (distance < COLLISION_THRESHOLD) {
              // Call the stop function immediately
              stop();
              selfDrivingInProgress = false;
              Serial2.println("Obstacle Detected!");
              Serial.println("Obstacle Detected!");
              return; // Exit the loop early if an obstacle is detected
          }
      } 
      // Forward
      else if (motorDirectionForward) {
          bool obstacleDetected = false;
          int ultrasensorTriggered = -1;
          // Read the front sensors
          // 0 = TRIG_PIN_FRONT_RIGHT
          // 1 = TRIG_PIN_FRONT_LEFT
          for (int i = 0; i < 2; i++) { // Loop through front sensors (0 and 1)
              float distance = readUltrasonicSensor(trigPin[i], echoPin[i]);
              
              if (distance < COLLISION_THRESHOLD) {
                  ultrasensorTriggered = i;
                  obstacleDetected = true;
                  break; // Exit the loop early if an obstacle is detected
              }
          }

          // If an obstacle is detected by any front sensor, call the stop function
          if (obstacleDetected) {
              stop();
              selfDrivingInProgress = false;
              Serial2.println("Obstacle Detected!");
              Serial.println("Obstacle Detected!");
              return; // Exit the loop early if an obstacle is detected
          }
      }
}