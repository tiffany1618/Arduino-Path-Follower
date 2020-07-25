#include <ECE3.h>

// Pin values
const int LEFT_NSLP_PIN = 31;
const int RIGHT_NSLP_PIN = 11;
const int LEFT_DIR_PIN = 29;
const int RIGHT_DIR_PIN = 30;
const int LEFT_PWM_PIN = 40;
const int RIGHT_PWM_PIN = 39;

const int LED_FL = 51;
const int LED_FR = 41;
const int LED_BL = 57;
const int LED_BR = 58;

// PID Terms
const float K_P = 0.01;
const float K_I = 0.0;
const float K_D = 0.0;
const float ERROR_THRESHOLD = 100;

const int NUM_SENSORS = 8;
float WEIGHTS_8421_4[] = { -8, -4, -2, -1, 1, 2, 4, 8, 4};
float WEIGHTS_1514128_8[] = { -15, -14, -12, -8, 8, 12, 14, 15, 8};
const int BASE_SPEED = 60;

uint16_t sensor_max_vals[] = {1916, 1813, 1903, 1232, 1321, 1870, 1657, 1715};
uint16_t sensor_min_vals[] = {576, 537, 597, 620, 506, 630, 597, 785};
uint16_t sensor_vals[NUM_SENSORS];
float speed_change;
int current_time;
int time_per_loop;
float curr_error;
float prev_error = 0;
float total_error = 0;

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

  min_calibrate();
  delay(5000);
//  max_calibrate();
//  delay(3000);
}


void loop() {
  current_time = millis();
  
  // Read raw sensor values
  ECE3_read_IR(sensor_vals);

  // Calculate error
  curr_error = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    curr_error += (((sensor_vals[i] - sensor_min_vals[i]) * 1000) / sensor_max_vals[i]) * WEIGHTS_8421_4[i] / WEIGHTS_8421_4[NUM_SENSORS];
  }

  Serial.print(curr_error);
  Serial.print('\n');

  if (curr_error < ERROR_THRESHOLD) {
    curr_error = 0;
  }

  time_per_loop = millis() - current_time;
  speed_change = K_P * curr_error + K_I * total_error + K_D * ((curr_error - prev_error) / time_per_loop);

  analogWrite(LEFT_PWM_PIN, BASE_SPEED - speed_change);
  analogWrite(RIGHT_PWM_PIN, BASE_SPEED + speed_change);

  total_error += curr_error * time_per_loop;
  prev_error = curr_error;
}

void reset(uint16_t *vals) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    vals[i] = 0;
  }
}

void min_calibrate() {
  for (int i = 0; i < 5; i++) {
    ECE3_read_IR(sensor_vals);

    for (int j = 0; j < NUM_SENSORS; j++) {
      sensor_min_vals[j] += sensor_vals[j] / 5;
    }
  }

  digitalWrite(LED_FR, HIGH);
  delay(500);
  digitalWrite(LED_FR, LOW);
}

void max_calibrate() {
  for (int i = 0; i < 5; i++) {
    ECE3_read_IR(sensor_vals);

    for (int j = 0; j < NUM_SENSORS; j++) {
      sensor_max_vals[j] += sensor_vals[j] / 5;
    }
  }

  digitalWrite(LED_BR, HIGH);
  delay(500);
  digitalWrite(LED_BR, LOW);
  delay(500);
  digitalWrite(LED_BR, HIGH);
  delay(500);
  digitalWrite(LED_BR, LOW);
}
