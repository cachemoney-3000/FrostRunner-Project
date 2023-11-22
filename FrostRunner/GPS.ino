// Get Latest GPS coordinates
bool checkGPS() {
  while (Serial1.available() > 0) {
    wdt_reset(); // Prevent the reset
    if (gps.encode(Serial1.read())) {
      return true;
    }
  }
  return false;
}


Location getGPS() {
  Location coordinates;

  unsigned long start = millis();
  while (millis() - start < GPS_TIMEOUT) {
    wdt_reset(); // Prevent the reset
    // If we recieved new location then take the coordinates and pack them into a struct
    if (checkGPS()){
      coordinates.latitude = gps.location.lat();
      coordinates.longitude = gps.location.lng();
      return coordinates;
    }
      
  }

  coordinates.latitude = 0;
  coordinates.longitude = 0;

  return coordinates;
}

Location applyMovingAverageFilter(Location newLocation) {
  newLocation.latitude = (1 - GPS_FILTER_WEIGHT) * newLocation.latitude + GPS_FILTER_WEIGHT * newLocation.latitude;
  newLocation.longitude = (1 - GPS_FILTER_WEIGHT) * newLocation.longitude + GPS_FILTER_WEIGHT * newLocation.longitude;
  
  return newLocation;
}

bool checkSatellites() {
  checkGPS();
  if (Number_of_SATS <= 4) {
    Serial2.println("No Satellites Found");
    return false;
  }
  else {
    Serial2.println(String(Number_of_SATS) + " Satellites Acquired");     
    return true;
  }
}
