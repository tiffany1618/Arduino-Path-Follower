//
// Created by tiffany on 7/23/20.
//

#ifndef ROBOT_PID_CONTROLLER_H
#define ROBOT_PID_CONTROLLER_H

class PID_Controller {
private:
    double K_P;
    double K_I;
    double K_D;
    double threshold;

public:
    PID_Controller(double threshold);

    void tune();
    double loop(double error);
};

#endif //ROBOT_PID_CONTROLLER_H
