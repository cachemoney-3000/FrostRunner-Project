// Function to read temperature in Fahrenheit
float readTemperatureFahrenheit() {
  // Reading temperature or humidity takes about 250 milliseconds.
  // Sensor readings may also be up to 2 seconds 'old' (it's a slow sensor)
  delay(2000);

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
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  pinMode(echoPin, INPUT);
  unsigned long ultrasonic_duration = pulseIn(echoPin, HIGH);

  // Convert the time into a distance
  float ultrasonic_cm = (ultrasonic_duration / 2) / 29.1; // Divide by 29.1 or multiply by 0.0343
  return ultrasonic_cm;
}