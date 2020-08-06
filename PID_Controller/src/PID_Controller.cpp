//
// Created by tiffany on 7/23/20.
//

#include "Arduino.h"
#include "PID_Controller.h"

#include <cmath>

PID_Controller::PID_Controller(double *error, double *output, double threshold) {
    this->K_P = 0;
    this->K_I = 0;
    this->K_D = 0;
    this->threshold = threshold;

    this->error = error;
    this->output = output;
    this->prev_error = 0;
    this->tot_error = 0;
}

PID_Controller::PID_Controller(double *error, double *output, double threshold, double K_P, double K_I, double K_D) {
    this->K_P = K_P;
    this->K_I = K_I;
    this->K_D = K_D;
    this->threshold = threshold;

    this->error = error;
    this->output = output;
    this->prev_error = 0;
    this->tot_error = 0;
}

void PID_Controller::reset_time() {
    initial_time = millis();
    prev_time = initial_time;
}

void PID_Controller::set_gains(double K_P, double K_I, double K_D) {
    this->K_P = K_P;
    this->K_I = K_I;
    this->K_D = K_D;
}

void PID_Controller::reset_values() {
    prev_error = 0;
    tot_error = 0;
}

void PID_Controller::tune() {
    calculate();

    Serial.print(prev_time - initial_time);
    Serial.print(",");
    Serial.println(*error);
}

void PID_Controller::set_tuned_gains(double K_u, double T_u) {
    K_P = 0.6 * K_u;
    K_I = 1.2 * K_u / T_u;
    K_D = 3.0 * K_u * T_u / 40.0;
}

void PID_Controller::calculate() {
    double curr_error = *error;

    if (std::abs(curr_error) < threshold) {
        curr_error = 0;
    }

    unsigned long curr_time = millis();
    double elapsed_time = static_cast<double>(curr_time - prev_time);
    *output = (K_P * curr_error) + (K_I * tot_error) + (K_D * (curr_error - prev_error) / elapsed_time);

    prev_error = curr_error;
    tot_error += curr_error * elapsed_time;
    prev_time = curr_time;
}

double PID_Controller::get_K_P() {
    return K_P;
}

double PID_Controller::get_K_I() {
    return K_I;
}
double PID_Controller::get_K_D() {
    return K_D;
}

double PID_Controller::get_threshold() {
    return threshold;
}
