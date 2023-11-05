// Function to read temperature in Fahrenheit
float readTemperatureFahrenheit() {
  // Reading temperature or humidity takes about 250 milliseconds.
  // Read temperature as Fahrenheit
  float f = dht.readTemperature(true);

  // Check if the read failed and return a placeholder value (-999)
  if (isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return -999.0;
  }

  return f;
}

float readUltrasonicSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  pinMode(echoPin, INPUT);
  unsigned long ultrasonic_duration = pulseIn(echoPin, HIGH);

  // Convert the time into a distance
  float ultrasonic_cm = (ultrasonic_duration / 2) / 29.1; // Divide by 29.1 or multiply by 0.0343
  return ultrasonic_cm;
}

void calibrateCompass() {
  Serial.println("This will provide calibration settings for your QMC5883L chip. When prompted, move the magnetometer in all directions until the calibration is complete.");
  Serial.println("Calibration will begin in 5 seconds.");
  delay(5000);

  Serial.println("CALIBRATING. Keep moving your sensor...");
  compass.calibrate();

  Serial.println("DONE. Copy the lines below and paste it into your projects sketch.);");
  Serial.println();
  Serial.print("compass.setCalibrationOffsets(");
  Serial.print(compass.getCalibrationOffset(0));
  Serial.print(", ");
  Serial.print(compass.getCalibrationOffset(1));
  Serial.print(", ");
  Serial.print(compass.getCalibrationOffset(2));
  Serial.println(");");
  Serial.print("compass.setCalibrationScales(");
  Serial.print(compass.getCalibrationScale(0));
  Serial.print(", ");
  Serial.print(compass.getCalibrationScale(1));
  Serial.print(", ");
  Serial.print(compass.getCalibrationScale(2));
  Serial.println(");");
}