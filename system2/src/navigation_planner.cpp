#include "ros/ros.h"

#include "navigation_planner.h"

//Constructor
NavigationPlanner::NavigationPlanner(){
    total_distance_travelled_ = 0;
    prev_pose_X_ = prev_pose_Y_ = curr_pose_X_ = curr_pose_Y_ = 0;
	//subscribe to /odom topic to receive odometry data. This is used to calculate total distance travelled.
    pose_sub_ = n_.subscribe("odom", 1000, &NavigationPlanner::onRobotPosition, this);
}

void NavigationPlanner::setHome(double goal_x, double goal_y, double degree_w){
    home_.setPosition(goal_x, goal_y, degree_w);
}

void NavigationPlanner::setGoal(double goal_x, double goal_y, double degree_w){
    goal_.setPosition(goal_x, goal_y, degree_w);
}

void NavigationPlanner::setPlan(std::vector<Position> newPlan){
    plan_ = newPlan;
}

void NavigationPlanner::setGoalTolerance(double newGoalTolerance){
    goal_tolerance_ = newGoalTolerance;
}

double NavigationPlanner::getDistanceTravelled(){
    return total_distance_travelled_;
}

void NavigationPlanner::goToWaypoint(){
    ros::Time stop, start = ros::Time::now();
    start = ros::Time::now();
    NavigationPlanner::sendGoal(goal_, true);
    stop = ros::Time::now();
    ROS_INFO("time to go from A to B = %lf %lf %lf seconds", start.toSec(), stop.toSec(), (stop - start).toSec());
}

bool NavigationPlanner::sendGoal(Position g, bool final){
    ros::Rate r(100);
    // create the action client  
    MoveBaseClient ac("move_base", true);
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ROS_INFO("Connected to move_base action server");

    // Send a goal to move_base
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();   

    goal.target_pose.pose.position.x = g.getPositionX();
    goal.target_pose.pose.position.y = g.getPositionY();
    goal.target_pose.pose.orientation = g.getPositionA();

    // Send the goal command
    ROS_INFO("Sending robot to: x = %f, y = %f, w = %f", g.getPositionX(), g.getPositionY(), g.getPositionAInDegree());
    ac.sendGoal(goal);
    // ROS_INFO("----------------------%s-----------------", ac.getState().toString().c_str());
    while(ros::ok() && (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE
            || ac.getState() == actionlib::SimpleClientGoalState::PENDING)){
        ros::spinOnce();
        r.sleep();
    }
    // // Wait for the action to return
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        if(final)
            ROS_INFO("You have reached the goal!");
        return true;
    } 
    else{
        ROS_INFO("The base failed for some reason");
        return false;
    }
}

//Callback for pose_sub_ subsciber. It stores the position
void NavigationPlanner::onRobotPosition (const nav_msgs::Odometry::ConstPtr& poseData){
    // ROS_INFO("NavigationPlanner::onRobotPosition-------------------------------------");

    prev_pose_X_ = curr_pose_X_;
    curr_pose_X_ = poseData->pose.pose.position.x;

    prev_pose_Y_ = curr_pose_Y_;
    curr_pose_Y_ = poseData->pose.pose.position.y;

    float x = prev_pose_X_ - curr_pose_X_;
    float y = prev_pose_Y_ - curr_pose_Y_;
    double current_distance_travelled_ = sqrt(x*x + y*y);

    total_distance_travelled_ = total_distance_travelled_ + current_distance_travelled_;

}