// You must then add your 'Declination Angle' to the compass, which is the 'Error' of the magnetic field in your location.
// Find yours here: http://www.magnetic-declination.com/
// Mine is: 13° 24' E (Positive), which is ~13 Degrees, or (which we need) 0.23 radians
#define DECLINATION_ANGLE -0.07f

// The offset of the mounting position to true north
// It would be best to run the /examples/magsensor sketch and compare to the compass on your smartphone
#define COMPASS_OFFSET 0.01f

// How often the GPS should update in MS
// Keep this above 1000
#define GPS_UPDATE_INTERVAL 1000

// Number of changes in movement to timeout for GPS waypoints
// Keeps the robot from driving away if there is a problem
#define GPS_WAYPOINT_TIMEOUT 25

// Struct to combine our coordinates into one struct for ease of use
struct Location {
  float latitude;
  float longitude;
};

// Conversion
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

// Ultrasonic Sensors Pins
#define TRIG_PIN_FRONT_RIGHT 7
#define ECHO_PIN_FRONT_RIGHT 6
#define TRIG_PIN_FRONT_LEFT 5
#define ECHO_PIN_FRONT_LEFT 4
#define TRIG_PIN_BACK 3
#define ECHO_PIN_BACK 2

#define NUM_ULTRASONIC_SENSORS 3
#define COLLISION_THRESHOLD 10.0f // 10.0 cm, can be change

// Motors Pins
#define REAR_MOTOR_IN1 5
#define REAR_MOTOR_IN2 4
#define REAR_MOTOR_ENA 6
#define STEERING_MOTOR_IN3 8
#define STEERING_MOTOR_IN4 7
#define STEERING_MOTOR_ENB 9

#define STEERING_TIME_THRESHOLD 200 // Controls how long the motor will run when steering
#define SELF_DRIVING_FORWARD_SPEED 100
#define SELF_DRIVING_REVERSE_SPEED 50