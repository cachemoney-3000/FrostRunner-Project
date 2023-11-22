void Startup()
{
  Serial.println("Searching for Satellites"); 
  Serial2.println("Searching for Satellites"); 

  while (Number_of_SATS <= 4)                         // Wait until x number of satellites are acquired before starting main loop
  {       
    wdt_reset(); // Prevent the reset
    bool check = checkGPS();
    Number_of_SATS = (int)(gps.satellites.value());   // Query Tiny GPS for the number of Satellites Acquired
  }    
  
  Serial.println(String(Number_of_SATS) + " Satellites Acquired");
  Serial2.println(String(Number_of_SATS) + " Satellites Acquired");     
}    

