#include <iostream>
#include <cmath>

class PDController{
    private:
        double dt_;
        double max_;
        double min_;
        double Kp_;
        double Kd_;
        double pre_error_;

    public:
        PDController( double dt, double max, double min, double Kp, double Kd);
        ~PDController();
        double getVelocity( Position *r, Position *g );
};