#include "localplanner.h"

LocalPlanner::LocalPlanner(){
    robot_position_ = new Position();
    goal_ = new Position();
}

void LocalPlanner::initLocalPlanner(ros::NodeHandle &n, float oscillation_distance, float oscillation_timeout){
    ROS_INFO("LocalPlanner::initLocalPlanner----------------------------------------------");
    oscillation_distance_ = oscillation_distance;
    oscillation_timeout_ = oscillation_timeout;
    n_ = n;
    sensor_.initSensor(n_);
    //register to advertise over /cmd_vel_mux/input/navi topic. This is used to turn the robot away from obstacles.
    pub_move_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
}

void LocalPlanner::setRobotPosition(double rp_x, double rp_y, double rp_a, int row = 0, int col = 0){
    // ROS_INFO("LocalPlanner::setRobotPosition----------------------------------------------");
    robot_position_->setPosition(rp_x, rp_y, rp_a);
    robot_position_->setGridPosition(row, col);
}

int LocalPlanner::executePlan(std::vector<Position*> plan, Position* goal, Position* robot_position){
    ROS_INFO("LocalPlanner::executePlan--------------------------------------------------");
    plan_ = plan;
    goal_ = goal;
    robot_position_ = robot_position;

    ros::Time stop, start;
    int i, zone_code;
    bool notReached = false;

    start = ros::Time::now();
    for(i=((int)plan_.size()-1); i>=0; i--){
        if((zone_code = sensor_.isRobotFacingObstacle()) > 0){
            ROS_INFO("Obstacle ahead");
            moveToSafety(zone_code);
            return -1;
        }
        if(i>=3){
            while(checkIfOnSameLine(plan_[i], plan_[i-1], plan_[i-2]) && i>=3){
                i--;
            }
        }
        ROS_INFO("Moving to point i=%d x=%lf y=%lf", i, plan_[i]->getPositionX(), plan_[i]->getPositionY());
        do{
            notReached = false;
            if(rotateRobot(plan_[i])){
                //Face the goal and then move ahead.
                if(moveRobot(plan_[i])){
                    if((robot_position_->getPositionX() - plan_[i]->getPositionX() > oscillation_distance_)
                        || (robot_position_->getPositionY() - plan_[i]->getPositionY() > oscillation_distance_))
                        notReached = true;
                }
            }
        } while(notReached);
        notReached = false;
    }
    stop = ros::Time::now();
    ROS_INFO("time to go from A to B = %f seconds", (stop - start).toSec());
    return 1;
}

bool LocalPlanner::checkIfOnSameLine(Position* A, Position* B, Position* C){
    // ROS_INFO("LocalPlanner::checkIfOnSameLine-------------------------------------------");
    if(A->getPositionX() && A->getPositionY() && 
        B->getPositionX() && B->getPositionY() && 
        C->getPositionX() && C->getPositionY()){

        return ( (( A->getPositionX() * (B->getPositionY() - C->getPositionY()) 
        + B->getPositionX() * (C->getPositionY() - A->getPositionY()) 
        + C->getPositionX() * (A->getPositionY() - B->getPositionY()) )
        / 2) == 0 );
    }
    else{
        return 0;
    }
}

bool LocalPlanner::moveRobot(Position* goal){
    ROS_INFO("LocalPlanner::moveRobot---fromX=%lf fromY=%lf---toX=%lf toY=%lf-----------------",
        robot_position_->getPositionX(), robot_position_->getPositionY(),
        goal->getPositionX(), goal->getPositionY());
    PDController pd = PDController(0.1, 0.5, -0.5, 0.4, 0.01);
    double val, speed, distance;

    geometry_msgs::Twist move;

    double t0 = ros::Time::now().toSec(), t1, fromStart = ros::Time::now().toSec();
    double current_distance = 0.0, x, y;
    ros::Rate loop_rate(100);
    do{
        x = robot_position_->getPositionX() - goal->getPositionX();
        y = robot_position_->getPositionY() - goal->getPositionY();
        distance = sqrt(x*x + y*y);
        //fabs(robot_position_->getPositionX() - goal->getPositionX());
                
        speed = pd.getVelocity(robot_position_, goal);

        move.linear.x = speed;
        move.linear.y = 0;
        move.linear.z = 0;

        move.angular.x = 0;
        move.angular.y = 0;
        move.angular.z = 0;

        pub_move_.publish(move);
        t1 = ros::Time::now().toSec();
        current_distance = current_distance + fabs(speed) * (t1-t0);
        t0 = ros::Time::now().toSec();

        // ROS_INFO("speed = %lf current_distance = %lf distance = %lf-----------------------", 
        // speed, current_distance, distance);

        ros::spinOnce();
        loop_rate.sleep();

        // if(fromStart > oscillation_timeout_)
        //     return false;
    }while(current_distance < distance - oscillation_distance_);
    return true;
}

bool LocalPlanner::rotateRobot(Position *goal){
    // ROS_INFO("LocalPlanner::rotateRobot------------------------------------------------");
    geometry_msgs::Twist move;
    double angle, fromStart = ros::Time::now().toSec();
    float Ex, Ey, dest, error;
    ros::Rate loop_rate(100);

    do{
        ros::spinOnce();
        // create error vector
        Ex = goal->getPositionX() - robot_position_->getPositionX();                                   // Error X. X component 
        Ey = goal->getPositionY() - robot_position_->getPositionY();                                   // Error Y. Y component 

        // get desire angle
        dest = atan2f(Ey, Ex);                                        // use float version to get arc tangent

        // get angle error
        error = dest - robot_position_->getPositionA();

        if(error > 0.5){
            angle = 0.5;
        }
        else{
            angle = error;
        }
        // angle = 0.5 * error;

        move.angular.z = angle;
        pub_move_.publish(move);
        ROS_INFO("LocalPlanner::rotateRobot----z=%lf error=%f----------------------------", 
            angle, error);

        loop_rate.sleep();

        // if(fromStart > oscillation_timeout_)
        //     return false;
    }while(fabs(error) > 0.05);
    
    return true;
}

//Based on zone data, move the robot
void LocalPlanner::moveToSafety(int zone_code){
    // ROS_INFO("LocalPlanner::moveToSafety----------------------------------------------");
    geometry_msgs::Twist move;
    move.linear.x = sensor_.getObstacleFreeLinearVelocity(zone_code);
    move.angular.z = sensor_.getObstacleFreeAngularVelocity(zone_code);
    pub_move_.publish(move);
}

LocalPlanner::~LocalPlanner() {}