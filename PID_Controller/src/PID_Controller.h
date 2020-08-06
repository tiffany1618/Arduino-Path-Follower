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

    void set_gains(double K_P, double K_I, double K_D);
    void reset_time();
    void reset_values();

    void tune();
    void set_tuned_gains(double K_u, double T_u);

    void calculate();

    double get_K_P();
    double get_K_I();
    double get_K_D();
    double get_threshold();

private:
    double K_P;
    double K_I;
    double K_D;
    double threshold;

    double *error;
    double *output;
    double prev_error;
    double tot_error;
    unsigned long prev_time;
    unsigned long initial_time;
};

#endif //ROBOT_PID_CONTROLLER_H
