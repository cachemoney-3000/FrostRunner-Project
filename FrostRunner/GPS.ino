// Get Latest GPS coordinates
bool checkGPS()
{
  while (Serial1.available()){
    if(gps.encode(Serial1.read())){
      return true;
    };
  }
  return false;
} 

// Get and process GPS data
Location gpsdump() {
  Location robotLoc;
  robotLoc.latitude = gps.location.lat();
  robotLoc.longitude = gps.location.lng();

  return robotLoc;
}

Location getGPS() {
  unsigned long start = millis();
  while (millis() - start < GPS_UPDATE_INTERVAL) {
    // If we recieved new location then take the coordinates and pack them into a struct
    if (checkGPS())
      return gpsdump();

    delay(100); // Small delay to allow the GPS module to provide updated data
  }

  Location robotLoc;
  robotLoc.latitude = 0.0;
  robotLoc.longitude = 0.0;
  
  return robotLoc;
}

