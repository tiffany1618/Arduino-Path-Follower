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
const double K_P = 0.2;
const double K_I = 0;
const double K_D = 15;

// Encoder values
const int ABOUT_FACE_COUNT_LEFT = 250;
const double TICKS_PER_MM = 360.0 / (70.0 * 3.1416);
const int STOP_LINE_WIDTH = 18; // mm

// Other constants
const int NUM_SENSORS = 8; // Number of sensors on the car
const int BASE_SPEED = 150; // Default wheel speed
const int NUM_TRAVERSALS = 1; // Number of times car should traverse path
const int SENSOR_THRESHOLD_BLACK = 7500;
const int SENSOR_WHITE = 200;
const int SENSOR_DIFF = 300;

// Weighting scheme for sensor fusion. Last number is the divisor.
double WEIGHTS_LINEAR[] = {-4, -3, -2, -1, 1, 2, 3, 4, 4};

// Arrays storing sensor values
int sensor_max_vals[NUM_SENSORS];
int sensor_min_vals[NUM_SENSORS];
uint16_t sensor_vals[NUM_SENSORS];

double norm_val; // Stores normalized sensor value
double speed_change; // Output of PID controller
double error; // Input of PID controller

// Determine whether the car should be stopped
double sensor_sum;

// Interrupts
volatile bool left_interrupt = false;
volatile bool right_interrupt = false;

// Initialize PID controller
PID_Controller pid(&error, &speed_change, 0, K_P, K_I, K_D);

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

  attachInterrupt(digitalPinToInterrupt(LEFT_BUTTON), left_button, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_BUTTON), right_button, FALLING);

  Serial.begin(9600); // Set the data rate in bits per second for serial data transmission
}

void loop() {
  // Get button input
  if (right_interrupt) {
    self_calibrate();
  }

  if (left_interrupt) {
    follow_path();
  }

  // Get PID input from serial monitor
  if (Serial.available() > 0) {
    serialChar = Serial.read();
    serialNum = (double) Serial.parseFloat();

    switch (serialChar) {
      case 'P':
        pid.set_gains(serialNum, pid.get_K_I(), pid.get_K_D());
        break;
      case 'I':
        pid.set_gains(pid.get_K_P(), serialNum, pid.get_K_D());
        break;
      case 'D':
        pid.set_gains(pid.get_K_P(), pid.get_K_I(), serialNum);
        break;
    }

    Serial.print("K_P: ");
    Serial.print(pid.get_K_P(), 5);
    Serial.print(", K_D: ");
    Serial.println(pid.get_K_D(), 5);
  }
}

void left_button() {
  left_interrupt = true;
}

void right_button() {
  right_interrupt = true;
}

// TIFFANY CHIEU
// AUTO CALIBRATION CODE
void self_calibrate() {
  digitalWrite(LED_FR, HIGH);
  calibrate_min();
  delay_milli(5000);
  calibrate_max();
  digitalWrite(LED_FR, LOW);
  right_interrupt = false;

//  Serial.println("Max");
//  for (int i = 0; i < NUM_SENSORS; i++) {
//    Serial.println(sensor_max_vals[i]);
//  }
//  Serial.println("Min");
//  for (int i = 0; i < NUM_SENSORS; i++) {
//    Serial.println(sensor_min_vals[i]);
//  }
}

// Calibrates minimum values for each of the sensors
void calibrate_min() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensor_min_vals[i] = 2500;
  }
  
  for (int i = 0; i < 20; i++) {
    ECE3_read_IR(sensor_vals);

    for (int j = 0; j < NUM_SENSORS; j++) {
      if (sensor_vals[i] < sensor_min_vals[i]) {
        sensor_min_vals[i] = sensor_vals[i];
      }
    }
  }

  blink_twice(LED_BL);
}

// Calibrates maximum values for each of the sensors
void calibrate_max() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensor_max_vals[i] = 0;
  }
  
  for (int i = 0; i < 20; i++) {
    ECE3_read_IR(sensor_vals);

    for (int j = 0; j < NUM_SENSORS; j++) {
      if (sensor_vals[i] > (sensor_max_vals[i] + sensor_min_vals[i])) {
        sensor_max_vals[i] = sensor_vals[i] - sensor_min_vals[i];
      }
    }
  }

  blink_twice(LED_BL);
}
// END AUTO CALIBRATION CODE

// PATH FOLLOWING CODE WITH PID CONTROLLER
void follow_path() {
  Serial.print("K_P: ");
  Serial.print(pid.get_K_P(), 5);
  Serial.print(", K_D: ");
  Serial.print(pid.get_K_D(), 5);
  Serial.print(", BASE_SPEED: ");
  Serial.println(BASE_SPEED);

  int i = NUM_TRAVERSALS;
  int left_speed, right_speed;
  int current_ticks, dist_since_black, black_start_ticks = 0;
  int last_trigger_time = 0, black_trigger_timeout = 0;

  delay_milli(1000);
  digitalWrite(LED_FL, HIGH);
  reset_car();

  unsigned long curr_time = millis();
 
  while (i > 0 && millis() - curr_time < 5000) {
    // Read raw sensor values
    ECE3_read_IR(sensor_vals);

    // Calculate error
    error = 0;
    sensor_sum = 0;
    int sensor_min = 2500, sensor_max = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      norm_val = ((double) (sensor_vals[i] - sensor_min_vals[i]) * 1000.0) / sensor_max_vals[i];             
      error += norm_val * WEIGHTS_LINEAR[i] / WEIGHTS_LINEAR[NUM_SENSORS];
      sensor_sum += norm_val;

      if (sensor_min > sensor_vals[i]) {
        sensor_min = sensor_vals[i];
      }
      if (sensor_max < sensor_vals[i]) {
        sensor_max = sensor_vals[i];
      }
    }

    if (sensor_sum <= SENSOR_WHITE && sensor_max - sensor_min <= SENSOR_DIFF) {
      set_speed(0);
      i = 0;
    }

    // Check if car has crossed a black area wide enough to be the end line using encoder ticks
    // black_trigger_timeout prevents the car from stopping after immediately crossing the end line again after turning around
    if (sensor_sum >= SENSOR_THRESHOLD_BLACK && (millis() - last_trigger_time) > black_trigger_timeout) {
      current_ticks = (getEncoderCount_left() + getEncoderCount_right()) / 2;
      dist_since_black = (current_ticks - black_start_ticks) / TICKS_PER_MM;
      black_trigger_timeout = 0;
      last_trigger_time = millis();

      if (dist_since_black > (STOP_LINE_WIDTH + 5)) {
        black_start_ticks = current_ticks;
      } else if (dist_since_black >= (STOP_LINE_WIDTH - 10)) {
        if (i > 1) {
          about_face();
          drive_forward(360);
          reset_car();
          black_start_ticks = -30;
          black_trigger_timeout = 1000;
        }
         
        i--;
      }
    } else {
      pid.calculate();
      Serial.println(error);
      
      // Adjust wheel speeds based on the output of the PID controller
      left_speed = BASE_SPEED - speed_change;
      right_speed = BASE_SPEED + speed_change;

      // Ensure the PID Controller does not give speed values over 255
      if (left_speed > 255) left_speed = 255;
      if (right_speed > 255) right_speed = 255;

      analogWrite(LEFT_PWM_PIN, left_speed);
      analogWrite(RIGHT_PWM_PIN, right_speed);
    }
  }

  set_speed(0);
  digitalWrite(LED_FL, LOW);
  left_interrupt = false;
}

// Reset PID values
void reset_car() {
  speed_change = 0;

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
  set_speed(0);
  delay_milli(50);
  digitalWrite(LEFT_DIR_PIN, HIGH);
  resetEncoderCount_left();
  resetEncoderCount_right();

  while (getEncoderCount_left() <= ABOUT_FACE_COUNT_LEFT) {
    set_speed(150);
  }

  digitalWrite(LEFT_DIR_PIN, LOW);
  delay_milli(50);
}

// Drive forward a certain amount of encoder ticks
void drive_forward(int ticks) {
  resetEncoderCount_left();
  resetEncoderCount_right();

  while (getEncoderCount_left() <= ticks && getEncoderCount_right() <= ticks) {
    set_speed(150);
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
