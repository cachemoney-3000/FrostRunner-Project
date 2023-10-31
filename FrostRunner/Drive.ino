// Go to the location specified by phoneLoc (latitude, longitude)
void driveTo(struct Location &phoneLoc) {
  Location robotLoc = getGPS();
  //robotLoc.latitude = 28.59108000;
  //robotLoc.longitude = -81.46820800;

  if (robotLoc.latitude != 0 && robotLoc.longitude != 0) {
    float distance = geoDistance(robotLoc, phoneLoc);;

    robotLoc = getGPS(); // Get robot coordinates
    //robotLoc.latitude = 28.59108000;
    //robotLoc.longitude = -81.46820800;

    Serial.println("Robot Longitude: " + String(robotLoc.longitude, 8));
    Serial.println("Robot Latitude: " + String(robotLoc.latitude, 8));
    
    if (robotLoc.latitude != 0 && robotLoc.longitude != 0){
        if (!steeringReleased && (millis() - motorStartTime >= steeringRunDuration)) {
            Serial.println("Release in GPS");
            steeringRelease();  // Stop the motor
            steeringReleased = true;  // Set the steeringReleased flag to true
        }

        // TODO geoDistance
        distance = geoDistance(robotLoc, phoneLoc);
        // Get the heading angle in degrees
        float heading = geoHeading();
        float bearing = geoBearing(robotLoc, phoneLoc) - heading;
        
        /* Serial.print("Distance: ");
        Serial.println(distance);

        Serial.print("Bearing: ");
        Serial.println(bearing);

        Serial.print("Heading: ");
        Serial.println(heading); */
        
        // TODO drive -> Motor controls
        drive(distance, bearing);
        //Serial.println("Drive: distance =" + String(distance) + " bearing =" + String(bearing));

        globalTimeout -= 1;

        if ((distance > 1000.0 && distance < 1.0) || globalTimeout == 0){
            Serial.println("Self Driving: TIMEOUT");
            stop();
            straightenWheel(); 
            selfDrivingInProgress = false;
            globalTimeout = GLOBAL_SELF_DRIVING_TIMEOUT;
        }

    }

  }
}

void drive(float distance, float bearing) {
    float bearingTolerance = 10.0;   // Define a tolerance of 10 degrees for heading
    float distanceTolerance = 1.0;  // Define a tolerance of 1 meters for distance

    // Ensure the heading difference is within the range of -180 to 180 degrees
    if (bearing > 180.0) {
        bearing -= 360.0;
    } else if (bearing < -180.0) {
        bearing += 360.0;
    }

    // TODO
    // Turning the vehicle
    if (fabs(bearing) > bearingTolerance) {
        if (bearing > 0) {
            // Turn right (clockwise)
            if (steeringLocation < 1) {
                Serial.println("Self Driving: Turn Right");
                steeringLocation = steerRight(false, steeringLocation);
            }
        } 
        else {
            // Turn left (counterclockwise)
            if (steeringLocation > -1) {
                Serial.println("Self Driving: Turn Left");
                steeringLocation = steerLeft(false, steeringLocation);
            }
        }
        
        forward(SELF_DRIVING_FORWARD_SPEED - 20);  // Adjust the speed as needed
        isVehicleTurning = true;
    } 
    else {
        Serial.println("Self Driving: Stop Turning");
        straightenWheel(); 
        stop();
        isVehicleTurning = false;
    }

    // Move forward and reverse
    if (!isVehicleTurning) {
        // Move forward or backward based on distance
        if (distance > distanceTolerance) {
            Serial.println("Self Driving: Forward");
            forward(SELF_DRIVING_FORWARD_SPEED);  // Adjust the speed as needed
        } 
        else if (distance < -distanceTolerance) {
            Serial.println("Self Driving: Reverse");
            reverse(SELF_DRIVING_REVERSE_SPEED);  // Adjust the speed as needed
        } 
        else {
            Serial.println("Self Driving: Destination Reached");
            stop();
        }
    }
}

float geoDistance(struct Location &robotLoc, struct Location &phoneLoc){
    const float R = 6371000; // radius of earth in metres
    float p1 = robotLoc.latitude * DEGTORAD;
    float p2 = phoneLoc.latitude * DEGTORAD;
    float dp = (phoneLoc.latitude - robotLoc.latitude) * DEGTORAD;
    float dl = (phoneLoc.longitude - robotLoc.longitude) * DEGTORAD;

    float x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
    float y = 2 * atan2(sqrt(x), sqrt(1-x));

    // returns distance in meters
    return R * y;
}

float geoBearing(struct Location &robotLoc, struct Location &phoneLoc) {
    float y = sin(phoneLoc.longitude - robotLoc.longitude) * cos(phoneLoc.latitude);
    float x = cos(robotLoc.latitude) * sin(phoneLoc.latitude) - sin(robotLoc.latitude) * cos(phoneLoc.latitude) * cos(phoneLoc.longitude - robotLoc.longitude);
    
    float bearing = atan2(y, x) * RADTODEG;
    
    // Ensure the bearing is within the range of 0 to 360 degrees
    if (bearing < 0) {
        bearing += 360.0;
    }
    return bearing;
}


float geoHeading() {
    // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    compass.read();
    float heading = atan2(compass.getY(), compass.getX()) * RAD_TO_DEG;

    // Offset
    heading -= DECLINATION_ANGLE;
    heading -= COMPASS_OFFSET;

    // Correct for when signs are reversed.
    if(heading < 0)
    heading += 2 * PI;

    // Check for wrap due to addition of declination.
    if(heading > 2 * PI)
    heading -= 2 * PI;

    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180 / PI; 

    // Map to -180 - 180
    while (headingDegrees < -180) headingDegrees += 360;
    while (headingDegrees >  180) headingDegrees -= 360;

    return headingDegrees;
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