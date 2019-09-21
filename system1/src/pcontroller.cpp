#include "pcontroller.h"

PController::PController( double dt, double max, double min, double Kp){
    dt_ = pre_error_ = dt;
    max_ = max;
    min_ = min;
    Kp_ = Kp;
}

double PController::getVelocity( Position* robot, Position* goal ){
    float error = getError( robot, goal );

    // Proportional term
    double Pout = Kp_ * error;

    // Calculate total output
    double output = Pout;

    // Restrict to max/min
    if( output > max_ )
        output = max_;
    else if( output < min_ )
        output = min_;

    // Save error to previous error
    pre_error_ = error;

    return output;
}

float PController::getError( Position* robot, Position* goal ){
    float Ex, Ey, dest;
    // Create error vector
    Ex = goal->getPositionX() - robot->getPositionX();                                   // Error X. X component 
    Ey = goal->getPositionY() - robot->getPositionY();                                   // Error Y. Y component 

    // Get desire angle
    dest = atan2f(Ey, Ex);                                        // use float version to get arc tangent

    // Calculate error
    return (dest - robot->getPositionA());
}

PController::~PController() {
    
}