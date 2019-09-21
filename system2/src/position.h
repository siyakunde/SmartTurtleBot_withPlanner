#include "ros/ros.h"
#include <tf/transform_datatypes.h>

class Position{
private:
    double x, y, a_in_degree;
    geometry_msgs::Quaternion a;
public:
	Position();
    void setPosition(double position_x, double position_y, double position_a);
    double getPositionX();
    double getPositionY();
    double getPositionAInDegree();
    geometry_msgs::Quaternion getPositionA();
};