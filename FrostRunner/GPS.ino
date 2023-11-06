// Get Latest GPS coordinates
void checkGPS()                                                 // Get Latest GPS coordinates
{
  while (Serial1.available() > 0)
    gps.encode(Serial1.read());
} 


Location getGPS() {
  checkGPS();
  Location coordinates;

  coordinates.latitude = gps.location.lat();
  coordinates.longitude = gps.location.lng();

  return coordinates;
}

Location applyMovingAverageFilter(Location newLocation) {
  newLocation.latitude = (1 - GPS_FILTER_WEIGHT) * newLocation.latitude + GPS_FILTER_WEIGHT * newLocation.latitude;
  newLocation.longitude = (1 - GPS_FILTER_WEIGHT) * newLocation.longitude + GPS_FILTER_WEIGHT * newLocation.longitude;
  
  return newLocation;
}

