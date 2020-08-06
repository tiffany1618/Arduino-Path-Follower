#include <PID_Controller.h>
#include <ECE3.h>
#include <Bump_Switch.h>

// Pin values
const int LEFT_NSLP_PIN = 31;
const int RIGHT_NSLP_PIN = 11;
const int LEFT_DIR_PIN = 29;
const int RIGHT_DIR_PIN = 30;
const int LEFT_PWM_PIN = 40;
const int RIGHT_PWM_PIN = 39;

const int RIGHT_BUTTON = PUSH1;
const int LEFT_BUTTON = PUSH2;

const int LED_FL = 51;
const int LED_FR = 41;
const int LED_BL = 57;
const int LED_BR = 58;

// PID Terms
const double K_P = 0.05;
const double K_I = 0.0;
const double K_D = 4.9;
const double ERROR_THRESHOLD = 0; // Set error to 0 if the absolute value of the error is less than this threshold, to prevent small oscillations

// Encoder values
//const int STRAIGHT_COUNT_LEFT = ;
//const int STRAIGHT_COUNT_RIGHT = ;
//const int RIBBON_COUNT_LEFT = ;
//const int RIBBON_COUNT_RIGHT = ;
const int ABOUT_FACE_COUNT_LEFT = 330;

// Other constants
const int NUM_SENSORS = 8; // Number of sensors on the car
const int BASE_SPEED = 130; // Default wheel speed
const int SENSOR_THRESHOLD_WHITE = 100;
const int SENSOR_THRESHOLD_DIFF = 300;

// Weighting schemes for sensor fusion. Last number is the divisor.
double WEIGHTS_8421_4[] = {-8, -4, -2, -1, 1, 2, 4, 8, 4};
double WEIGHTS_1514128_8[] = {-15, -14, -12, -8, 8, 12, 14, 15, 8};
double WEIGHTS_LINEAR[] = {-4, -3, -2, -1, 1, 2, 3, 4, 4};

int sensor_max_vals[NUM_SENSORS];
int sensor_min_vals[NUM_SENSORS];
uint16_t sensor_vals[NUM_SENSORS];

double norm_val; // Stores normalized sensor value
double speed_change; // Stores output of PID controller
double error;

// Determine whether the car should be stopped
double sensor_sum;
double sensor_min;
double sensor_max;

// Initialize PID controller
PID_Controller pid(&error, &speed_change, ERROR_THRESHOLD, K_P, K_I, K_D);

char serialChar;
double serialNum;

void setup() {
  ECE3_Init();
  
  // Initialize motor pins
  pinMode(LEFT_NSLP_PIN, OUTPUT);
  pinMode(RIGHT_NSLP_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(LEFT_BUTTON, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON, INPUT_PULLUP);

  digitalWrite(LEFT_NSLP_PIN, HIGH);
  digitalWrite(RIGHT_NSLP_PIN, HIGH);
  digitalWrite(LEFT_DIR_PIN, LOW);
  digitalWrite(RIGHT_DIR_PIN, LOW);

  Serial.begin(9600); // Set the data rate in bits per second for serial data transmission
}


void loop() {
  // Get button input
  if (digitalRead(RIGHT_BUTTON) == LOW) {
    run_calibration();
  }

  if (digitalRead(LEFT_BUTTON) == LOW) {
    follow_path();
  }

  // Get PID input from serial monitor
  if (Serial.available() > 0) {
    serialChar = Serial.read();
    serialNum = (double) Serial.parseFloat();

    switch (serialChar) {
      case 'P':
        pid.set_gains(serialNum, K_I, K_D);
        break;
      case 'I':
        pid.set_gains(K_P, serialNum, K_D);
        break;
      case 'D':
        pid.set_gains(K_P, K_I, serialNum);
        break;
    }

    Serial.print("K_P: ");
    Serial.print(pid.get_K_P(), 5);
    Serial.print(", K_D: ");
    Serial.println(pid.get_K_D(), 5);
  }
}

void run_calibration() {
  digitalWrite(LED_FR, HIGH);
  calibrate(sensor_min_vals);

  // Delay for 5 seconds to allow time to position car.
  delay_milli(5000);
  
  calibrate(sensor_max_vals);
  digitalWrite(LED_FR, LOW);
}

void follow_path() {
  Serial.print("K_P: ");
  Serial.print(pid.get_K_P(), 5);
  Serial.print(", K_D: ");
  Serial.print(pid.get_K_D(), 5);
  Serial.print(", BASE_SPEED: ");
  Serial.println(BASE_SPEED);

  bool drive = true;

  delay_milli(1000);
  digitalWrite(LED_FL, HIGH);
  reset_car();
 
  while (drive) {
    // Read raw sensor values
    ECE3_read_IR(sensor_vals);

    // Calculate error
    error = 0;
    sensor_sum = 0;
    sensor_min = 2500;
    sensor_max = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      norm_val = (((double) sensor_vals[i] - sensor_min_vals[i]) * 1000) / sensor_max_vals[i];
      error += norm_val * WEIGHTS_LINEAR[i] / WEIGHTS_LINEAR[NUM_SENSORS];

      sensor_sum += norm_val;
      if (sensor_min > sensor_vals[i]) {
        sensor_min = sensor_vals[i];
      }
      if (sensor_max < sensor_vals[i]) {
        sensor_max = sensor_vals[i];
      }
    }
    
    if ((sensor_sum <= SENSOR_THRESHOLD_WHITE) && (sensor_max - sensor_min <= SENSOR_THRESHOLD_DIFF)) {
      set_speed(0);
      drive = false;
    } else {
//      pid.calculate();
      pid.tune();

//        Serial.print(getEncoderCount_left());
//        Serial.print(", ");
//        Serial.println(getEncoderCount_right());
          
      // Adjust wheel speeds based on the output of the PID controller
      analogWrite(LEFT_PWM_PIN, BASE_SPEED - speed_change);
      analogWrite(RIGHT_PWM_PIN, BASE_SPEED + speed_change);
    }
  }

  digitalWrite(LED_FL, LOW);
}

// Reset PID and Encoder values
void reset_car() {
  speed_change = 0;

  resetEncoderCount_left();
  resetEncoderCount_right();

  pid.reset_values();
  pid.reset_time();
}

// Set both wheels to the same speed
void set_speed(int speed) {
  analogWrite(LEFT_PWM_PIN, speed);
  analogWrite(RIGHT_PWM_PIN, speed);
}

// Turn car 180 degrees around
void about_face() {
  digitalWrite(LEFT_DIR_PIN, HIGH);
  resetEncoderCount_left();
  resetEncoderCount_right();

  while (getEncoderCount_left() <= ABOUT_FACE_COUNT_LEFT) {
    set_speed(BASE_SPEED);
  }

  digitalWrite(LEFT_DIR_PIN, LOW);
}

// Calibrates maximum and minimum values for each of the sensors
void calibrate(int *vals) {
  zero(vals);
  
  for (int i = 0; i < 10; i++) {
    ECE3_read_IR(sensor_vals);

    for (int j = 0; j < NUM_SENSORS; j++) {
      vals[j] += (double) sensor_vals[j] / 10;
    }
  }

  blink_twice(LED_BL);
}

// Sets all values of array to 0
void zero(int *vals) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    vals[i] = 0;
  }
}

// Blink LED twice at half second interval
void blink_twice(int pin) {
  bool stop_loop = false;
  unsigned long start_time = millis();
  unsigned long interval = 250;

  while (!stop_loop) {
    if (millis() >= start_time + (interval * 4)) {
      digitalWrite(pin, LOW);
      stop_loop = true;
    } else if (millis() >= start_time + (interval * 3)) {
      digitalWrite(pin, HIGH);
    } else if (millis() >= start_time + (interval * 2)) {
      digitalWrite(pin, LOW);
    } else if (millis() >= start_time + interval) {
      digitalWrite(pin, HIGH);
    }
  }
}


// The built-in delay() function appeared to alter the behavior of the millis() function, so I wrote a custom delay function instead
void delay_milli(unsigned long interval) {
  bool stop_loop = false;
  unsigned long start_time = millis();

  while (!stop_loop) {
    if (millis() >= start_time + interval) {
      stop_loop = true;
    }
  }
}
