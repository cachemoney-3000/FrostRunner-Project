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
#define COLLISION_THRESHOLD 70.0f // CM

#define FOLLOW_ME_DISTANCE 15.0f

// Motors Pins
#define REAR_MOTOR_IN1 9
#define REAR_MOTOR_IN2 8
#define REAR_MOTOR_ENA 10
#define STEERING_MOTOR_IN3 12
#define STEERING_MOTOR_IN4 11
#define STEERING_MOTOR_ENB 13

#define STEERING_TIME_THRESHOLD 250 // Controls how long the motor will run when steering
#define SELF_DRIVING_FORWARD_SPEED 200
#define SELF_DRIVING_REVERSE_SPEED 200
#define GLOBAL_SELF_DRIVING_TIMEOUT 15 
#define DISTANCE_TOLERANCE 50.0
#define BEARING_TOLERANCE 5.0

#define SELF_DRIVING_STEERING_DELAY 1500 // ms delay
#define GPS_FILTER_WEIGHT 0.2f

// Temperature Sensor Pins
#define DHTPIN 50   // Digital pi
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321