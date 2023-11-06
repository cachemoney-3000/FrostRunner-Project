// Go to the location specified by phoneLoc (latitude, longitude)
void driveTo(struct Location &phoneLoc) {
  wdt_reset(); // Prevent the reset
  Location robotLoc = getGPS();
  /* Location robotLoc;
  robotLoc.latitude = 28.59109411244622;
  robotLoc.longitude = -81.46732786360467; */

  if (robotLoc.latitude != 0 && robotLoc.longitude != 0) {
    robotLoc = getGPS();

    float distance = gps.distanceBetween(phoneLoc.latitude, phoneLoc.longitude, robotLoc.latitude, robotLoc.longitude);

    Serial.println("Robot Longitude: " + String(robotLoc.longitude , 8));
    Serial.println("Robot Latitude: " + String(robotLoc.latitude, 8));

    /** Calculate the azimuths */
    byte locationAzimuth = calculateAzimuth(robotLoc, phoneLoc);
    compass.read();
    byte compassAzimuth = compass.getAzimuth();
    Serial.println("Azimuth Loc = " + String(locationAzimuth) + " Azimuth Compass =" + String(compassAzimuth)+ " Distance =" + String(distance));
    
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
double degreesToRadians(double degrees) {
  return degrees * M_PI / 180.0;
}

double radiansToDegrees(double radians) {
  return radians * 180.0 / M_PI;
}

// Calculate the azimuth between two GPS coordinates and return it as a byte
byte calculateAzimuth(Location currentLocation, Location targetLocation) {
  double lat1 = degreesToRadians(currentLocation.latitude);
  double lon1 = degreesToRadians(currentLocation.longitude);
  double lat2 = degreesToRadians(targetLocation.latitude);
  double lon2 = degreesToRadians(targetLocation.longitude);

  double deltaLon = lon2 - lon1;

  double y = sin(deltaLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(deltaLon);

  double azimuth = atan2(y, x);
  azimuth = radiansToDegrees(azimuth);
  Serial.println("Azimuth Not Normalized: " + String(azimuth));
  azimuth = fmod((azimuth + 360.0), 360.0);  // Normalize to [0, 360) degrees

  // Convert the azimuth to a byte (0-255)
  byte azimuthByte = byte(azimuth);

  return azimuthByte;
}

void drive(float distance, byte locationAzimuth, byte compassAzimuth) {
  // For the delay timer
  unsigned long startTime = millis(); // Store the start time
  unsigned long delayTime = SELF_DRIVING_STEERING_DELAY; 

  // Normalize azimuthDifference to the range [-180, 180] degrees
  int azimuthDifference = locationAzimuth - compassAzimuth;

  // Ensure azimuthDifference is within [0, 360] range
  if (azimuthDifference < 0) {
    azimuthDifference += 360;
  }

  // Check if you are facing the correct direction
  if (azimuthDifference <= SELF_DRIVING_HEADING_TOLERANCE || azimuthDifference >= 360 - SELF_DRIVING_HEADING_TOLERANCE) {
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
      wdt_reset(); // Prevent the reset
      //checkForObstacle();
    }

    Serial.println("Delay done");
    stop();
  }
  
  // Move forward and reverse
  if (!isVehicleTurning) {
      // Move forward or backward based on distance
      if (distance > SELF_DRIVING_DISTANCE_TOLERANCE) {
          Serial.println("Self Driving: Forward");
          forward(SELF_DRIVING_FORWARD_SPEED);
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