// Go to the location specified by phoneLoc (latitude, longitude)
void driveTo(struct Location &phoneLoc, int timeout) {
  // Serial1.listen();
  Location robotLoc = getGPS();
  // Serial2.listen();

  if (robotLoc.latitude != 0 && robotLoc.longitude != 0) {
    float distance = geoDistance(robotLoc, phoneLoc);;
    //Start move loop here
    do {
        // Serial1.listen();
        robotLoc = getGPS();
        // Serial2.listen();
        
        // TODO geoDistance
        distance = geoDistance(robotLoc, phoneLoc);
        // TODO geoBearing, heoHeading
        float bearing = geoBearing(robotLoc, phoneLoc) - geoHeading();
        
        Serial.print("Distance: ");
        Serial.println(distance);

        Serial.print("Bearing: ");
        Serial.println(geoBearing(robotLoc, phoneLoc));

        Serial.print("Heading: ");
        Serial.println(geoHeading());
        
        // TODO drive -> Motor controls
        drive(distance, bearing);
        //Serial.println("Drive: distance =" + String(distance) + " bearing =" + String(bearing));

        timeout -= 1;
    } while (distance > 1.0 && timeout > 0);

    // TODO stop
    Serial.println("stop");
    // stop();
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
    return atan2(y, x) * RADTODEG;
}

float geoHeading() {
    // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = compass.getAzimuth();

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