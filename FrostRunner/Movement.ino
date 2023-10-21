
int steerLeft(boolean stop, int steerLocation){
  // Left
  steerLocation--;

  digitalWrite(STEERING_MOTOR_IN3, HIGH);
  digitalWrite(STEERING_MOTOR_IN4, LOW);
  analogWrite(STEERING_MOTOR_ENB, steeringSpeed);

  motorStartTime = millis();
  steeringReleased = false;

  if(stop){
    Serial.print("steeringLocation before: ");
    Serial.println(steerLocation);
    steeringRunDuration = 120;
    Serial.println("steeringRunDuration");
    Serial.println(steeringRunDuration);
  }
  else {
    steeringRunDuration = 180;
  }

  Serial.println("SteerLocation");
  Serial.println(steerLocation);

  return steerLocation;
}

int steerRight(boolean stop, int steerLocation) {
   // Right
  steerLocation++;
  
  digitalWrite(STEERING_MOTOR_IN3, LOW);
  digitalWrite(STEERING_MOTOR_IN4, HIGH);
  analogWrite(STEERING_MOTOR_ENB, steeringSpeed);

  motorStartTime = millis();
  steeringReleased = false;

  if(stop){
    Serial.print("steerLocation before: ");
    Serial.println(steerLocation);
    steeringRunDuration = 180;
    Serial.println("steeringRunDuration");
    Serial.println(steeringRunDuration);
  }
  else {
    steeringRunDuration = 180;
  }

  Serial.println("SteerLocation");
  Serial.println(steerLocation);

  return steerLocation;
}

void forward(int speed) {
  // Start time
  smoothStartTime = millis();
  gradualSpeed = false;
  // Forward
  digitalWrite(REAR_MOTOR_IN1, HIGH);
  digitalWrite(REAR_MOTOR_IN2, LOW);
  analogWrite(REAR_MOTOR_ENA, speed);

  motorDirectionForward = true;
  motorDirectionReverse = false;
}

void reverse(int speed) {
  // Start time
  smoothStartTime = millis();
  gradualSpeed = false;
  // Backward
  digitalWrite(REAR_MOTOR_IN1, LOW);
  digitalWrite(REAR_MOTOR_IN2, HIGH);
  analogWrite(REAR_MOTOR_ENA, speed);

  motorDirectionReverse = true;
  motorDirectionForward = false;
}

void straightenWheel() {
  // Straighten the wheels
  if (steeringLocation > 0) {
    steeringLocation = steerLeft(true, steeringLocation);
    steeringLocation = 0;
  }
  if (steeringLocation < 0) {
    steeringLocation = steerRight(true, steeringLocation);
    steeringLocation = 0;
  }
}

void stop() {
  gradualSpeed = false;
  digitalWrite(REAR_MOTOR_IN1,LOW);
  digitalWrite(REAR_MOTOR_IN2,LOW);
  
  straightenWheel();

  Serial.println("Stop");

  motorDirectionForward = false;
  motorDirectionReverse = false;
}

void steeringRelease() {
  digitalWrite(STEERING_MOTOR_IN3,LOW);
  digitalWrite(STEERING_MOTOR_IN4,LOW);
}