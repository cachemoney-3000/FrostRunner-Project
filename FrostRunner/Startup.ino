void Startup()
{
  Serial.println("Searching for Satellites"); 
  Serial2.println("Searching for Satellites"); 
      
  while (Number_of_SATS <= 3)                         // Wait until x number of satellites are acquired before starting main loop
  {       
    checkGPS();
    Number_of_SATS = (int)(gps.satellites.value());   // Query Tiny GPS for the number of Satellites Acquired
  }    
  
  Serial.println(String(Number_of_SATS) + " Satellites Acquired");
  Serial2.println(String(Number_of_SATS) + " Satellites Acquired");     
}    

bool checkSatellites() {
  checkGPS();
  int numSatellite = (int)(gps.satellites.value());
  if (numSatellite <= 3) {
    Serial2.println("No Satellites Found");
    return false;
  }
  else {
    Serial2.println(String(numSatellite) + " Satellites Acquired");     
    return true;
  }
}


void displayInfo()
{
  //gps.location.isValid()
  if (gps.location.isValid())
  {
    float currentLatitude = gps.location.lat();
    float currentLongitude = gps.location.lng();
    //float currentLatitude = 28.59111022;
    //float currentLongitude = -81.46820831;

    
    Serial.print("Latitude: ");
    Serial.println(currentLatitude, 8);
    Serial.print("Longitude: ");
    Serial.println(currentLongitude, 8);

    float Distance_To_Home = TinyGPSPlus::distanceBetween(currentLatitude, currentLongitude, targetLatitude, targetLongitude);  //Query Tiny GPS for Distance to Destination
    int GPS_Course = TinyGPSPlus::courseTo(currentLatitude, currentLongitude, targetLatitude, targetLongitude);                           //Query Tiny GPS for Course to Destination   

    Serial.print("Distance to target (2): ");
    Serial.print(Distance_To_Home, 2);
    Serial.println(" meters");

    Serial2.print("Distance to target (2): ");
    Serial2.print(Distance_To_Home, 2);
    Serial2.println(" meters");

    Serial.print("GPS COURSE: ");
    Serial.print(GPS_Course);
  }
  else
  {
    Serial.println("Location: Not Available");
    Serial2.println("Location: Not Available");
  }
  
  
  Serial2.print("Date: ");
  if (gps.date.isValid())
  {
    Serial2.print(gps.date.month());
    Serial2.print("/");
    Serial2.print(gps.date.day());
    Serial2.print("/");
    Serial2.println(gps.date.year());
  }
  else
  {
    Serial2.println("Not Available");
  }

  Serial2.print("Time: ");
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial2.print(F("0"));
    Serial2.print(gps.time.hour());
    Serial2.print(":");
    if (gps.time.minute() < 10) Serial2.print(F("0"));
    Serial2.print(gps.time.minute());
    Serial2.print(":");
    if (gps.time.second() < 10) Serial2.print(F("0"));
    Serial2.print(gps.time.second());
    Serial2.print(".");
    if (gps.time.centisecond() < 10) Serial2.print(F("0"));
    Serial2.println(gps.time.centisecond());
  }
  else
  {
    Serial2.println("Not Available");
  }
  

  Serial.println();
  Serial.println();
  delay(1000);
}