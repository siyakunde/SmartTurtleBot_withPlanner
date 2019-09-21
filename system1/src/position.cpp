#include "ros/ros.h"
#include "position.h"

//Constructor
Position::Position(){
	x = y = a = grid_x = grid_y = 0;
	f = g = h = 0;
	closed = opened = false;
	penalty = 1.0;
}

void Position::setGridPosition(int position_grid_x, int position_grid_y){
	grid_x = position_grid_x;
	grid_y = position_grid_y;
}

void Position::setPosition(double position_x, double position_y, double position_a){
	a_in_degree = position_a * (180/M_PI);
	// Convert the Euler angle to quaternion
    // double radians_a = degree_a * (M_PI/180);
    // tf::Quaternion quaternion;
    // quaternion = tf::createQuaternionFromYaw(radians_a);

    // geometry_msgs::Quaternion position_a;
    // tf::quaternionTFToMsg(quaternion, position_a);

	x = position_x;
	y = position_y;
	a = position_a;
}

Position* Position::getPosition(){
	return this;
}

int Position::getPositionGridX(){
	return grid_x;
}

double Position::getPositionX(){
	return x;
}

int Position::getPositionGridY(){
	return grid_y;
}

double Position::getPositionY(){
	return y;
}

double Position::getPositionAInDegree(){
	return a_in_degree;
}

double Position::getPositionA(){
	return a;
}

Position* Position::getParent(){
	return parent;
}

void Position::setParent(Position *p){
	parent = p;
}

bool Position::hasParent(){
    return parent != NULL;
}

float Position::getGScore(Position *p){
	return p->g + 
	((grid_x == p->grid_x || grid_y == p->grid_y) ? 1.0 : 1.4);
}

float Position::getHScore(Position *p){
	return  (
		abs (p->grid_x - grid_x)
		+ abs (p->grid_y - grid_y)
		) * penalty;// + ((double)rand() / ((double)RAND_MAX*(double)1000));
}

float Position::getGScore(){
    return g;
}

float Position::getHScore(){
    return h;
}

float Position::getFScore(){
    return f;
}

void Position::computeScores(Position *end){
    g = getGScore(parent);
    h = getHScore(end);
    f = g + h;
    // ROS_INFO("g=%f, h=%f, f=%f, %d,%d p= %d,%d e= %d,%d"
    // 	, g
    // 	, h
    // 	, f
    // 	, getPositionGridX()
    // 	, getPositionGridY()
    // 	, parent->getPositionGridX()
    // 	, parent->getPositionGridY()
    // 	, end->getPositionGridX()
    // 	, end->getPositionGridY());
}

void Position::setOpened(bool o){
	opened = o;
}

bool Position::getOpened(){
	return opened;
}

void Position::setClosed(bool c){
	closed = c;
}

bool Position::getClosed(){
	return closed;
}

void Position::setWalkable(bool w){
	walkable = w;
}

bool Position::getWalkable(){
	return walkable;
}

void Position::incrementPenalty(){
	penalty *=penalty;
}