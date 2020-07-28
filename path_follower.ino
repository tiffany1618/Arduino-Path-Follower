#include <PID_Controller.h>
#include <ECE3.h>

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
const double K_P = 0.01;
const double K_I = 0.0;
const double K_D = 0.0;
const double ERROR_THRESHOLD = 100; // Set error to 0 if the absolute value of the error is less than this threshold, to prevent small oscillations

// Other constants
const int NUM_SENSORS = 8; // Number of sensors on the car
const int BASE_SPEED = 60; // Default wheel speed
const int SENSOR_THRESHOLD = 10; // 1% margin of error for determining whether sensors read black or white

// Weighting schemes for sensor fusion. Last number is the divisor.
float WEIGHTS_8421_4[] = { -8, -4, -2, -1, 1, 2, 4, 8, 4};
float WEIGHTS_1514128_8[] = { -15, -14, -12, -8, 8, 12, 14, 15, 8};

uint16_t sensor_max_vals[] = {1916, 1813, 1903, 1232, 1321, 1870, 1657, 1715};
uint16_t sensor_min_vals[] = {576, 537, 597, 620, 506, 630, 597, 785};
uint16_t sensor_vals[NUM_SENSORS];

double norm_val; // Stores normalized sensor value
double speed_change; // Stores output of PID controller
double error;

// Determine whether the car should be stopped
bool should_stop;
bool was_over_black;
double sensor_avg;

// Keep track of button states
bool right_button_state;
bool left_button_state;

// Initialize PID controller
PID_Controller pid(&error, &speed_change, ERROR_THRESHOLD, K_P, K_I, K_D);

void setup() {
   ECE3_Init();
  
  // Initialize motor pins
  pinMode(LEFT_NSLP_PIN, OUTPUT);
  pinMode(RIGHT_NSLP_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);

  digitalWrite(LEFT_NSLP_PIN, HIGH);
  digitalWrite(RIGHT_NSLP_PIN, HIGH);
  digitalWrite(LEFT_DIR_PIN, LOW);
  digitalWrite(RIGHT_DIR_PIN, LOW);

  Serial.begin(9600); // set the data rate in bits per second for serial data transmission

//  min_calibrate();
//  delay(5000);
//  max_calibrate();
//  delay(3000);

  // Initialize important values
  speed_change = 0;
  
  should_stop = false;
  was_over_black = false;
  sensor_avg = 0;

  right_button_state = false;
  left_button_state = false;
  
  pid.set_time();
}


void loop() {
  // deal with buttons
  if (digitalRead(RIGHT_BUTTON) == LOW) {
    toggle_state(right_button_state, LED_FR);
  }

  if (digitalRead(LEFT_BUTTON) == LOW) {
    toggle_state(left_button_state, LED_FL);
  }
  
  if (should_stop) {
    stop();
  } else if (right_button_state) {
    min_calibrate();
    delay(5000);
    max_calibrate();
    delay(5000);
  } else if (left_button_state) {
    // Read raw sensor values
    ECE3_read_IR(sensor_vals);

    // Calculate error
    error = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      norm_val = ((sensor_vals[i] - sensor_min_vals[i]) * 1000) / sensor_max_vals[i];
      sensor_avg += norm_val / NUM_SENSORS;
      error += norm_val * WEIGHTS_8421_4[i] / WEIGHTS_8421_4[NUM_SENSORS];
    }

    // Determine if car should stop
    if ((1000 - sensor_avg) < SENSOR_THRESHOLD) {
      was_over_black = true;
    } else {
      was_over_black = false;
    }
    
    if ((sensor_avg < SENSOR_THRESHOLD) && was_over_black) {
      should_stop = true;
    }
  
    Serial.println(error);
  
    // Adjust wheel speeds based on the output of the PID controller
    pid.calculate();
    analogWrite(LEFT_PWM_PIN, BASE_SPEED - speed_change);
    analogWrite(RIGHT_PWM_PIN, BASE_SPEED + speed_change);
  }
}

// Stop the car
void stop() {
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
}

void reset(uint16_t *vals) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    vals[i] = 0;
  }
}

void min_calibrate() {
  reset(sensor_min_vals);
  
  for (int i = 0; i < 5; i++) {
    ECE3_read_IR(sensor_vals);

    for (int j = 0; j < NUM_SENSORS; j++) {
      sensor_min_vals[j] += sensor_vals[j] / 5;
    }
  }

  blink_twice(LED_BL);
}

void max_calibrate() {
  reset(sensor_max_vals);
  
  for (int i = 0; i < 5; i++) {
    ECE3_read_IR(sensor_vals);

    for (int j = 0; j < NUM_SENSORS; j++) {
      sensor_max_vals[j] += sensor_vals[j] / 5;
    }
  }

  blink_twice(LED_BR);
}

// Toggle state of button
void toggle_state(bool *state, int led_pin) {
  if (*state) {
    (*state) = false;
    digitalWrite(led_pin, LOW);
  } else {
    (*state) = true;
    digitalWrite(led_pin, HIGH);
  }
}

void blink_twice(int pin) {
  digitalWrite(pin, HIGH);
  delay(500);
  digitalWrite(pin, LOW);
  delay(500);
  digitalWrite(pin, HIGH);
  delay(500);
  digitalWrite(pin, LOW);
}
