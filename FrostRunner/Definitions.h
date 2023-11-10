// How often the GPS should update in MS
// Keep this above 1000
#define GPS_UPDATE_INTERVAL 1000

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
#define GLOBAL_SELF_DRIVING_TIMEOUT 50 

// Self Driving
#define SELF_DRIVING_STEERING_DELAY 1500 // ms delay
#define GPS_FILTER_WEIGHT 0.2f  // Filter weight (high = noisy, low = stable)
#define SELF_DRIVING_HEADING_TOLERANCE 60 // degrees
#define SELF_DRIVING_DISTANCE_TOLERANCE 1.5f // meters
#define GPS_TIMEOUT 1000

// Temperature Sensor Pins
#define DHTPIN 50   // Digital pi
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321