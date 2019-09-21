#include <iostream>
#include <cmath>

class PController{
    private:
        double dt_;
        double max_;
        double min_;
        double Kp_;
        double pre_error_;

    public:
        PController( double dt, double max, double min, double Kp);
        ~PController();
        double getVelocity( Position* robot, Position* goal );
        float getError( Position* robot, Position* goal );
};