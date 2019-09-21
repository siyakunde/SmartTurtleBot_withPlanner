#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

class Position{
private:
	int grid_x, grid_y;

    double x, y, a_in_degree, a;

    Position *parent;
    float f, g, h;
    bool closed, opened, walkable;

    double penalty;

public:
	Position();
    void setGridPosition(int position_grid_x, int position_grid_y);
    void setPosition(double position_x, double position_y, double position_a);
    Position* getPosition();
    int getPositionGridX();
    double getPositionX();
    int getPositionGridY();
    double getPositionY();
    double getPositionAInDegree();
    double getPositionA();

    Position* getParent();
    void setParent(Position *p);
    bool hasParent();

    float getGScore(Position *p);
    float getHScore(Position *p);
    float getGScore();
    float getHScore();
    float getFScore();
    void computeScores(Position *end);

    void setOpened(bool o);
    bool getOpened();
    void setClosed(bool c);
    bool getClosed();
    void setWalkable(bool w);
    bool getWalkable();

    void incrementPenalty();
};