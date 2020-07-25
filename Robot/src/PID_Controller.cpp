//
// Created by tiffany on 7/23/20.
//

#include "PID_Controller.h"

PID_Controller::PID_Controller(double threshold) {
    this->threshold = threshold;
    this->K_P = 1;
    this->K_I = 0;
    this->K_D = 0;
}

void PID_Controller::tune() {

}

double PID_Controller::loop(double error) {
    
}