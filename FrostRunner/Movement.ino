
void steerLeft(boolean stop){
  // Left
  steeringLocation--;

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, steeringSpeed);

  motorStartTime = millis();
  steeringReleased = false;

  if(stop){
    Serial.print("steeringLocation before: ");
    Serial.println(steeringLocation);
    steeringRunDuration = 120;
    Serial.println("steeringRunDuration");
    Serial.println(steeringRunDuration);
  }
  else {
    steeringRunDuration = 180;
  }

  Serial.println("SteerLocation");
  Serial.println(steeringLocation);
}

void steerRight(boolean stop) {
   // Right
  steeringLocation++;
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, steeringSpeed);

  motorStartTime = millis();
  steeringReleased = false;

  if(stop){
    Serial.print("steeringLocation before: ");
    Serial.println(steeringLocation);
    steeringRunDuration = 180;
    Serial.println("steeringRunDuration");
    Serial.println(steeringRunDuration);
  }
  else {
    steeringRunDuration = 180;
  }

  Serial.println("SteerLocation");
  Serial.println(steeringLocation);
}

void forward() {
  // Start time
  smoothStartTime = millis();
  gradualSpeed = false;
  // Forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 100);

  motorDirectionForward = true;
  motorDirectionReverse = false;
}

void reverse() {
  // Start time
  smoothStartTime = millis();
  gradualSpeed = false;
  // Backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 100);

  motorDirectionReverse = true;
  motorDirectionForward = false;
}

void straightenWheel() {
  // Straighten the wheels
  if (steeringLocation > 0) {
    steerLeft(true);
    steeringLocation = 0;
  }
  if (steeringLocation < 0) {
    steerRight(true);
    steeringLocation = 0;
  }
}

void stop() {
  gradualSpeed = false;
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  
  straightenWheel();

  Serial.println("Stop");

  motorDirectionForward = false;
  motorDirectionReverse = false;
}

void steeringRelease() {
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
}