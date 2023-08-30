
bool isQueueEmpty() {
  return (waypointCounter == 0);
}

bool isQueueFull() {
  return (waypointCounter == MAX_WAYPOINTS);
}

void enqueueWaypoint(double latitude, double longitude) {
  if (isQueueFull()) {
    Serial.println("Queue is Full");
    return;
  }
  
  rearIndex = (rearIndex + 1) % MAX_WAYPOINTS;
  waypoint.LatitudeArray[rearIndex] = latitude;
  waypoint.LongitudeArray[rearIndex] = longitude;
  waypointCounter++;
}

void dequeueWaypoint() {
  if (isQueueEmpty()) {
    Serial.println("Queue is Empty");
    return;
  }
  
  frontIndex = (frontIndex + 1) % MAX_WAYPOINTS;
  waypointCounter--;
}

void setWaypoint() {
  char receivedChar = Serial2.read();
  
  if (receivedChar == '/') {
    bluetoothReadFlag = 1;
    if (location != "") {
      // Split the location string into longitude and latitude
      int separatorIndex = location.indexOf('\n');
      if (separatorIndex != -1) {
        String longitude = location.substring(0, separatorIndex);
        String latitude = location.substring(separatorIndex + 1);

        // Convert latitude and longitude to double values
        double targetLatitude = latitude.toDouble();
        double targetLongitude = longitude.toDouble();

        // Enqueue the waypoint
        enqueueWaypoint(targetLatitude, targetLongitude);

        Serial.println("Target Longitude: " + String(targetLongitude, 8));
        Serial.println("Target Latitude: " + String(targetLatitude, 8));

        location = ""; // Reset the location buffer
        bluetoothReadFlag = 0; // Reset the flag

        //displayInfo();
      }
    }
  }
  else {
    location += receivedChar;
  }
}

void moveToWaypoint(double targetLatitude, double targetLongitude) {
  getGPS();
  double currentLatitude = gps.location.lat();
  double currentLongitude = gps.location.lng();
  
  // Calculate the desired heading (direction) towards the target waypoint
  //double desiredHeading = calculateHeading(currentLatitude, currentLongitude, targetLatitude, targetLongitude);
  
  // Adjust the movement of the cooler based on the desired heading
  // For example, you can control the motors or other mechanisms to move the cooler towards the target waypoint
  
  // Update the current location of the cooler based on the movement
  
  // Print any relevant information or feedback
  
  // Delay or control the timing of the movements, if necessary
}

