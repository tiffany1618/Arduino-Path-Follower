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

void PID_Controller::set_time() {
    prev_time = millis();
}

void PID_Controller::set_gains(double K_P, double K_I, double K_D) {
    this->K_P = K_P;
    this->K_I = K_I;
    this->K_D = K_D;
}

// Ziegler-Nichols Method
double PID_Controller::tune(double step) {
    return 0;
}

void PID_Controller::set_tuned_gains() {
    K_P = 0.6 * K_u;
    K_I = 1.2 * K_u / T_u;
    K_D = 3.0 * K_u * T_u / 40.0;
}

void PID_Controller::calculate() {
    double curr_error = *error;

    if (std::abs(curr_error) < threshold) {
        error = 0;
    }

    double curr_time = millis();
    double elapsed_time = curr_time - prev_time;
    (*output) = (K_P * curr_error) + (K_I * tot_error) + (K_D * (curr_error - prev_error) / elapsed_time);

    prev_error = curr_error;
    tot_error += curr_error * elapsed_time;
    prev_time = curr_time;
}