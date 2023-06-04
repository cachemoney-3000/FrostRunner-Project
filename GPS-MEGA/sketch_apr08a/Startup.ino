void Startup()
{
  //myservo.detach(); 
  //Serial.println("Pause for Startup... ");
  
  /*
  for (int i=5; i >= 1; i--)                       // Count down for X seconds
  {         
    Serial1.print("Pause for Startup... "); 
    Serial1.print(i);
    delay(1000);                                   // Delay for X seconds
  }
  */
    
  Serial.println("Searching for Satellites "); 
  Serial2.println("Searching for Satellites "); 
      
  while (Number_of_SATS <= 4)                         // Wait until x number of satellites are acquired before starting main loop
  {       
    getGPS();                                         // Update gps data
    Number_of_SATS = (int)(gps.satellites.value());   // Query Tiny GPS for the number of Satellites Acquired
  }    
  //setWaypoint();                                      // set intial waypoint to current location
  //wpCount = 0;                                        // zero waypoint counter
  //ac = 0;                                             // zero array counter
  
  Serial.print(Number_of_SATS);
  Serial.print(" Satellites Acquired");    

  Serial2.print(Number_of_SATS);
  Serial2.print(" Satellites Acquired");    
}    





void displayInfo()
{
  //gps.location.isValid()
  if (gps.location.isValid())
  {
    double currentLatitude = gps.location.lat();
    double currentLongitude = gps.location.lng();
    //double currentLatitude = 28.59111022;
    //double currentLongitude = -81.46820831;

    
    Serial.print("Latitude: ");
    Serial.println(currentLatitude, 8);
    Serial.print("Longitude: ");
    Serial.println(currentLongitude, 8);

    double Distance_To_Home = TinyGPSPlus::distanceBetween(currentLatitude, currentLongitude, targetLatitude, targetLongitude);  //Query Tiny GPS for Distance to Destination
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
  delay(5000);
}