//
// Created by tiffany on 7/23/20.
//

#ifndef ROBOT_PID_CONTROLLER_H
#define ROBOT_PID_CONTROLLER_H

#include "Arduino.h"

class PID_Controller {
public:
    PID_Controller(double *error, double *output, double threshold);
    PID_Controller(double *error, double *output, double threshold, double K_P, double K_I, double K_D);

    void set_time();
    void set_gains(double K_P, double K_I, double K_D);

    double tune(double step);
    void set_tuned_gains();

    void calculate();

private:
    double K_P;
    double K_I;
    double K_D;
    double threshold;

    double *error;
    double *output;
    double prev_error;
    double tot_error;
    long prev_time;

    // Tuning
    double K_u; // Ultimate gain
    double T_u; // Period of oscillation at K_u
};

#endif //ROBOT_PID_CONTROLLER_H
