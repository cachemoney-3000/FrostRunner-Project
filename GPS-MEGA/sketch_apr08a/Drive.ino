void drive(int distance, float turn) {
    int fullSpeed = 230;
    int stopSpeed = 0;

    // drive to location
    int s = fullSpeed;
    // Slowing down??? Needed???
    if ( distance < 8 ) {
    int wouldBeSpeed = s - stopSpeed;
    wouldBeSpeed *= distance / 8.0f;
    s = stopSpeed + wouldBeSpeed;
    }

    int autoThrottle = constrain(s, stopSpeed, fullSpeed);
    autoThrottle = 230;

    float t = turn;
    while (t < -180) t += 360;
    while (t >  180) t -= 360;

    Serial.print("turn: ");
    Serial.println(t);
    Serial.print("original: ");
    Serial.println(turn);

    float t_modifier = (180.0 - abs(t)) / 180.0;
    float autoSteerA = 1;
    float autoSteerB = 1;

    if (t < 0) {
    autoSteerB = t_modifier;
    } else if (t > 0){
    autoSteerA = t_modifier;
    }

    Serial.print("steerA: "); Serial.println(autoSteerA);
    Serial.print("steerB: "); Serial.println(autoSteerB);

    //  int speedA = (int) (((float) map(autoThrottle, stopSpeed, fullSpeed, RC_NEUTRAL, RC_MAX)) * autoSteerA);
    //  int speedB = (int) (((float) map(autoThrottle, stopSpeed, fullSpeed, RC_NEUTRAL, RC_MIN)) * autoSteerB);
    int speedA = (int) (((float) map(autoThrottle, stopSpeed, fullSpeed, RC_NEUTRAL, RC_MIN)) * autoSteerA);
    int speedB = (int) (((float) map(autoThrottle, stopSpeed, fullSpeed, RC_NEUTRAL, RC_MAX)) * autoSteerB);

    //setSpeedMotorA(speedA);
    //setSpeedMotorB(speedB);
}