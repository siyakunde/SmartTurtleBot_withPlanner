#include "ros/ros.h"
#include "position.h"

//Constructor
Position::Position(){
	
}

void Position::setPosition(double position_x, double position_y, double degree_a){
	a_in_degree = degree_a;
	// Convert the Euler angle to quaternion
    double radians_a = degree_a * (M_PI/180);
    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(radians_a);

    geometry_msgs::Quaternion position_a;
    tf::quaternionTFToMsg(quaternion, position_a);

	x = position_x;
	y = position_y;
	a = position_a;
}

double Position::getPositionX(){
	return x;
}

double Position::getPositionY(){
	return y;
}

double Position::getPositionAInDegree(){
	return a_in_degree;
}

geometry_msgs::Quaternion Position::getPositionA(){
	return a;
}