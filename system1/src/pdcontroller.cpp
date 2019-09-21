#include "pdcontroller.h"

PDController::PDController( double dt, double max, double min, double Kp, double Kd){
    dt_ = pre_error_ = dt;
    max_ = max;
    min_ = min;
    Kp_ = Kp;
    Kd_ = Kd;
}

double PDController::getVelocity( Position *r, Position *g ){
    double x, y;
    // Calculate error
    x = r->getPositionX() - g->getPositionX();
    y = r->getPositionY() - g->getPositionY();
    double error = sqrt(x*x + y*y);

    // Proportional term
    double Pout = Kp_ * error;

    // Derivative term
    double derivative = (error - pre_error_) / dt_;
    double Dout = Kd_ * derivative;

    // Calculate total output
    double output = Pout + Dout;

    // Restrict to max/min
    if( output > max_ )
        output = max_;
    else if( output < min_ )
        output = min_;

    // Save error to previous error
    pre_error_ = error;

    return output;
}

PDController::~PDController() {
    
}