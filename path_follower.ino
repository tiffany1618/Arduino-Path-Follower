#include <ECE3.h>

// Pin values
const int LEFT_NSLP_PIN = 31;
const int RIGHT_NSLP_PIN = 11;
const int LEFT_DIR_PIN = 29;
const int RIGHT_DIR_PIN = 30;
const int LEFT_PWM_PIN = 40;
const int RIGHT_PWM_PIN = 39;

const int LED_RF = 41;
const int LED_BL = 57;

// PID Terms
const float K_P = 60.0;
const float K_I = 0.0;
const float K_D = 0.0;

int NUM_SENSORS = 8;
float WEIGHTS_8421_4[] = {-8, -4, -2, -1, 1, 2, 4, 8, 4};
float WEIGHTS_1514128_8[] = {-15, -14, -12, -8, 8, 12, 14, 15, 8};

uint16_t sensor_vals[8];
int left_speed;
int right_speed;
float curr_error;
float prev_error = 0;

void setup() {
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
  digitalWrite(RIGHT_DIR_PIN, HIGH);
  
  ECE3_Init();
  
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
}


void loop() {
  // Read raw sensor values
  ECE3_read_IR(sensor_vals);

  // Calculate error
  curr_error = 0;
  for(int i = 0; i < NUM_SENSORS; i++) {
    curr_error += (sensor_vals[i] * WEIGHTS_8421_4[i]) / WEIGHTS_8421_4[NUM_SENSORS];
  }

  

  analogWrite(LEFT_PWM_PIN, left_speed);
  analogWrite(RIGHT_PWM_PIN, right_speed);

  prev_error = curr_error;
  
//  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
//  // 2500 means minimum reflectance
//  for (unsigned char i = 0; i < 8; i++)
//  {
//    Serial.print(sensorValues[i]);
//    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
//  }
//  Serial.println();
}

double pid_controller(double k_p, double k_i, double k_d) {
  
}
