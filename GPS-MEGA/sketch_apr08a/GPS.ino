void calibrateCompass()                                           // Experimental Use Only to Calibrate Magnetometer/ Compass
{
  int minX = 0;
  int maxX = 0;
  int offX = 0;

  int minY = 0;
  int maxY = 0;
  int offY = 0; 

  int minZ = 0;
  int maxZ = 0;
  int offZ = 0; 

  for (int i=1000; i >= 1; i--) 
  {
    Vector mag = compass.readRaw();                                 // Read compass data
    
    // Determine Min / Max values
    if (mag.XAxis < minX) minX = mag.XAxis;
    if (mag.XAxis > maxX) maxX = mag.XAxis;
    if (mag.YAxis < minY) minY = mag.YAxis;
    if (mag.YAxis > maxY) maxY = mag.YAxis;

    if (mag.ZAxis < minZ) minZ = mag.ZAxis;
    if (mag.ZAxis > maxZ) maxZ = mag.ZAxis;
    
    offX = (maxX + minX)/2;                                         // Calculate offsets
    offY = (maxY + minY)/2;
    offZ = (maxZ + minZ)/2;
    
    delay(10);
  }

  Serial1.print("Compass X & Y offset: ");
  Serial1.print(offX);
  Serial1.print(" ");
  Serial1.print(offY);
  Serial.print("\n");
  compass.setOffset(offX, offY, offZ);                                  // Set calibration offset
}

void getGPS()                                                 // Get Latest GPS coordinates
{
  while (Serial1.available() > 0)
    gps.encode(Serial1.read());
} 