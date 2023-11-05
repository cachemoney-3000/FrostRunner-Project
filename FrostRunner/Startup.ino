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
  if (numSatellite <= 2) {
    Serial2.println("No Satellites Found");
    return false;
  }
  else {
    Serial2.println(String(numSatellite) + " Satellites Acquired");     
    return true;
  }
}