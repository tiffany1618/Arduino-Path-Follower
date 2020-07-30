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

const int BUMP_0 = 24;
const int BUMP_1 = 25;
const int BUMP_2 = 6;
const int BUMP_3 = 27;
const int BUMP_4 = 8;
const int BUMP_5 = 28;

// PID Terms
double K_P = 0.027;
const double K_I = 0.0;
const double K_D = 0.0;
const double ERROR_THRESHOLD = 0; // Set error to 0 if the absolute value of the error is less than this threshold, to prevent small oscillations

// Encoder values
//const int STRAIGHT_COUNT;
//const int RIBBON_COUNT;
//const int ABOUT_FACE_COUNT;

// Other constants
const int NUM_SENSORS = 8; // Number of sensors on the car
int BASE_SPEED = 150; // Default wheel speed
const int SENSOR_THRESHOLD_WHITE = 100;
const int SENSOR_THRESHOLD_DIFF = 300;

// Weighting schemes for sensor fusion. Last number is the divisor.
float WEIGHTS_8421_4[] = { -8, -4, -2, -1, 1, 2, 4, 8, 4};
float WEIGHTS_1514128_8[] = { -15, -14, -12, -8, 8, 12, 14, 15, 8};
float WEIGHTS_LINEAR[] = {-1, -1, -1, -1, 1, 1, 1, 1, 1};

// 11 am readings
//int sensor_max_vals[] = {1916, 1813, 1903, 1232, 1321, 1870, 1657, 1715};
//int sensor_min_vals[] = {576, 537, 597, 620, 506, 630, 597, 785};
// 6 pm readings
int sensor_max_vals[] = {2500, 2500, 2500, 2453, 2353, 2500, 2500, 2500};
int sensor_min_vals[] = {576, 555, 613, 622, 542, 647, 613, 756};
//int sensor_max_vals[NUM_SENSORS];
//int sensor_min_vals[NUM_SENSORS];
uint16_t sensor_vals[NUM_SENSORS];

double norm_val; // Stores normalized sensor value
double speed_change; // Stores output of PID controller
double error;

// Determine whether the car should be stopped
double sensor_sum;
double sensor_min;
double sensor_max;

// Keep track of button states
bool right_button_state;
bool left_button_state;

// Initialize PID controller
PID_Controller pid(&error, &speed_change, ERROR_THRESHOLD, K_P, K_I, K_D);

// Bumper switches
Bump_Switch bump_switch[6];

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

  bump_switch[0].begin(BUMP_0, INPUT_PULLUP);
  bump_switch[1].begin(BUMP_1, INPUT_PULLUP);
  bump_switch[2].begin(BUMP_2, INPUT_PULLUP);
  bump_switch[3].begin(BUMP_3, INPUT_PULLUP);
  bump_switch[4].begin(BUMP_4, INPUT_PULLUP);
  bump_switch[5].begin(BUMP_5, INPUT_PULLUP);

  digitalWrite(LEFT_NSLP_PIN, HIGH);
  digitalWrite(RIGHT_NSLP_PIN, HIGH);
  digitalWrite(LEFT_DIR_PIN, LOW);
  digitalWrite(RIGHT_DIR_PIN, LOW);

  Serial.begin(9600); // set the data rate in bits per second for serial data transmission

  right_button_state = false;
  left_button_state = false;
}


void loop() {
  about_face();
  
  // Get button input
  if (digitalRead(RIGHT_BUTTON) == LOW) {
    toggle_state(&right_button_state, LED_FR);
  }

  if (digitalRead(LEFT_BUTTON) == LOW) {
    toggle_state(&left_button_state, LED_FL);

    speed_change = 0;

    resetEncoderCount_left();
    resetEncoderCount_right();

    pid.reset_values();
    pid.reset_time();
  }

  // Get bumper switch input (for PID tuning)
  for (int i = 0; i < 6; i++) {
    if (bump_switch[i].read() == 0) {
      switch(i) {
        case 0:
          K_P += 0.005;
          blink_twice(LED_FR);
          break;
        case 1:
          K_P -= 0.005;
          blink_twice(LED_FL);
          break;
        case 2:
          K_P += 0.001;
          blink_twice(LED_FR);
          break;
        case 3:
          K_P -= 0.001;
          blink_twice(LED_FL);
          break;
        case 4:
          BASE_SPEED += 10;
          blink_twice(LED_FR);
          break;
        case 5:
          BASE_SPEED -= 10;
          blink_twice(LED_FL);
          break;
      }
    }
  }

  if (right_button_state) {
    calibrate(sensor_min_vals);

    // Delay for 5 seconds to allow time to position car.
    delay_milli(5000);
    
    calibrate(sensor_max_vals);
    toggle_state(&right_button_state, LED_FR);
  } else if (left_button_state) {
    // Read raw sensor values
    ECE3_read_IR(sensor_vals);

    // Calculate error
    error = 0;
    sensor_sum = 0;
    sensor_min = 2500;
    sensor_max = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      norm_val = ((sensor_vals[i] - sensor_min_vals[i]) * 1000) / sensor_max_vals[i];
      error += norm_val * WEIGHTS_8421_4[i] / WEIGHTS_8421_4[NUM_SENSORS];

      sensor_sum += norm_val;
      if (sensor_min > sensor_vals[i]) {
        sensor_min = sensor_vals[i];
      }
      if (sensor_max < sensor_vals[i]) {
        sensor_max = sensor_vals[i];
      }
    }
    
    if ((sensor_sum <= SENSOR_THRESHOLD_WHITE) && (sensor_max - sensor_min <= SENSOR_THRESHOLD_DIFF)) {
      stop_car();
      toggle_state(&left_button_state, LED_FL);
    } else {
//      pid.calculate();
        pid.tune();
          
      // Adjust wheel speeds based on the output of the PID controller
      analogWrite(LEFT_PWM_PIN, BASE_SPEED - speed_change);
      analogWrite(RIGHT_PWM_PIN, BASE_SPEED + speed_change);
    }
  }
}

// Stop the car
void stop_car() {
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
}

// Turn car 180 degrees around
void about_face() {
  digitalWrite(LEFT_DIR_PIN, HIGH);
  resetEncoderCount_left();
  resetEncoderCount_right();

  Serial.print(getEncoderCount_left());
  Serial.print(", ");
  Serial.println(getEncoderCount_right());

  while (true) {
    analogWrite(LEFT_PWM_PIN, 90);
    analogWrite(RIGHT_PWM_PIN, 90);
  }

  digitalWrite(LEFT_DIR_PIN, LOW);
}

// Calibrates maximum and minimum values for each of the sensors
void calibrate(int *vals) {
  zero(vals);
  
  for (int i = 0; i < 10; i++) {
    ECE3_read_IR(sensor_vals);

    for (int j = 0; j < NUM_SENSORS; j++) {
      vals[j] += sensor_vals[j] / 10;
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

// Toggle state of button and corresponding LED indicator
void toggle_state(bool *state, int led_pin) {
  if (*state) {
    *state = false;
    digitalWrite(led_pin, LOW);
  } else {
    *state = true;
    digitalWrite(led_pin, HIGH);
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
