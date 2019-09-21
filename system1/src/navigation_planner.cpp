#include "ros/ros.h"

#include "navigation_planner.h"

//Constructor
NavigationPlanner::NavigationPlanner(ros::NodeHandle &n, float robot_radius, double goal_tolerance, float oscillation_timeout, float oscillation_distance){
    // ROS_INFO("NavigationPlanner::NavigationPlanner---------------------------------------");
    n_ = n;
    lp_.initLocalPlanner(n_, oscillation_distance, oscillation_timeout);

    goal_tolerance_ = goal_tolerance;
    robot_radius_ = robot_radius;
    oscillation_distance_ = oscillation_distance;
    oscillation_timeout_ = oscillation_timeout;

    total_distance_travelled_ = 0;
    prev_pose_X_ = prev_pose_Y_ = curr_pose_X_ = curr_pose_Y_ = 0;
	home_ = new Position();
    goal_ = new Position();
    robot_position_ = new Position();
    robot_offset_ = new Position();
    //subscribe to /odom topic to receive odometry data. This is used to calculate total distance travelled.
    pose_sub_ = n_.subscribe("odom", 1000, &NavigationPlanner::onRobotPosition, this);
}

void NavigationPlanner::setHome(double goal_x, double goal_y, double goal_a){
    // ROS_INFO("NavigationPlanner::setHome-------------------------------------------------");
    home_->setPosition(goal_x, goal_y, goal_a);
    robot_offset_->setPosition(goal_x, goal_y, goal_a);
}

Position* NavigationPlanner::getHome(){
    return home_;
}

void NavigationPlanner::setGoal(double goal_x, double goal_y, double goal_a){
    // ROS_INFO("NavigationPlanner::setGoal-------------------------------------------------");
    goal_->setPosition(goal_x, goal_y, goal_a);
}

Position* NavigationPlanner::getGoal(){
    return goal_;
}

void NavigationPlanner::setRobotPosition(double rp_x, double rp_y, double rp_a){
    // ROS_INFO("NavigationPlanner::setRobotPosition---------------------------------------");
    robot_position_->setPosition(rp_x, rp_y, rp_a);
    lp_.setRobotPosition(rp_x, rp_y, rp_a);
}

Position* NavigationPlanner::getRobotPosition(){
    return robot_position_;
}

void NavigationPlanner::setPlan(std::vector<Position*> newPlan){
    plan_ = newPlan;
}

//Calculates the distance driven by the robot
void NavigationPlanner::onDistanceTravelled (){
    float x = prev_pose_X_ - curr_pose_X_;
    float y = prev_pose_Y_ - curr_pose_Y_;
    double current_distance_travelled_ = sqrt(x*x + y*y);

    total_distance_travelled_ = total_distance_travelled_ + current_distance_travelled_;
}

double NavigationPlanner::getDistanceTravelled(){
    return total_distance_travelled_;
}

void NavigationPlanner::goToWaypoint(){
    // ROS_INFO("NavigationPlanner::goToWaypoint-----------------------------------------------");
    
    // status 0 is no plan and 1 is plan successfully executed
    int status;
    Position *start = home_;

    robot_position_->setPosition((robot_position_->getPositionX() - grid_origin_.position.x), 
                    (robot_position_->getPositionY() - grid_origin_.position.y), 
                    (robot_position_->getPositionA()));
    robot_position_->setGridPosition((robot_position_->getPositionX()*20), 
                    (robot_position_->getPositionY()*20));

    // ROS_INFO("Robot_position_ ----------------i=%d j=%d x=%lf y=%lf a=%lf--------------",
    //     robot_position_->getPositionGridX(), robot_position_->getPositionGridY(),
    //     robot_position_->getPositionX(), robot_position_->getPositionY(),
    //     robot_position_->getPositionAInDegree());

    start->setPosition((start->getPositionX() - grid_origin_.position.x), 
                    (start->getPositionY() - grid_origin_.position.y), 
                    (start->getPositionA()));
    start->setGridPosition((start->getPositionX()*20), 
                    (start->getPositionY()*20));

    goal_->setPosition((goal_->getPositionX() - grid_origin_.position.x), 
                    (goal_->getPositionY() - grid_origin_.position.y), 
                    (goal_->getPositionA()));
    goal_->setGridPosition((goal_->getPositionX()*20), 
                    (goal_->getPositionY()*20));

    do{
        if(status == -1){
            ROS_INFO("New plan needed");
            start = robot_position_;
        }
        plan_ = gp_.getPlan(map_, grid_origin_, grid_resolution_, robot_radius_, start, goal_);
        if(plan_.size() > 0){
            ROS_INFO("Got a new plan!");
            status = lp_.executePlan(plan_, goal_, robot_position_);
        }
        else{
            ROS_ERROR("Could not generate a plan.");
            status = 0;
        }
    }while(status != 1 && status != 0);
}

//Callback for pose_sub_ subsciber. It stores the position
void NavigationPlanner::onRobotPosition (const nav_msgs::Odometry::ConstPtr& poseData){
    // ROS_INFO("NavigationPlanner::onRobotPosition-------------------------------------");

    tf::Quaternion q(poseData->pose.pose.orientation.x, poseData->pose.pose.orientation.y,
                    poseData->pose.pose.orientation.z, poseData->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    prev_pose_X_ = curr_pose_X_;
    curr_pose_X_ = poseData->pose.pose.position.x;

    prev_pose_Y_ = curr_pose_Y_;
    curr_pose_Y_ = poseData->pose.pose.position.y;

    robot_position_->setPosition((poseData->pose.pose.position.x), 
                    (poseData->pose.pose.position.y), 
                    (yaw));
    
    robot_position_->setPosition((robot_position_->getPositionX() - grid_origin_.position.x + robot_offset_->getPositionX()), 
                    (robot_position_->getPositionY() - grid_origin_.position.y + robot_offset_->getPositionY()), 
                    (robot_position_->getPositionA()));
    robot_position_->setGridPosition((robot_position_->getPositionX()*20), 
                    (robot_position_->getPositionY()*20));
    // ROS_INFO("Robot_position_ ----------------i=%d j=%d x=%lf y=%lf a=%lf--------------",
    //     robot_position_->getPositionGridX(), robot_position_->getPositionGridY(),
    //     robot_position_->getPositionX(), robot_position_->getPositionY(),
    //     robot_position_->getPositionAInDegree());
    lp_.setRobotPosition(robot_position_->getPositionX(), robot_position_->getPositionY(), 
        robot_position_->getPositionA(), robot_position_->getPositionGridX(), robot_position_->getPositionGridY());

    float x = prev_pose_X_ - curr_pose_X_;
    float y = prev_pose_Y_ - curr_pose_Y_;
    double current_distance_travelled_ = sqrt(x*x + y*y);

    total_distance_travelled_ = total_distance_travelled_ + current_distance_travelled_;
}

bool NavigationPlanner::getGrid(){
    // ROS_INFO("NavigationPlanner::getGrid-----------------------------------------------");
    nav_msgs::GetMap::Request req;
    nav_msgs::GetMap::Response res;
    while (!ros::service::waitForService("static_map", ros::Duration(3.0))) {
        ROS_INFO("Waiting for service static_map to become available");
    }
    ROS_INFO("Requesting the map...");
    ros::ServiceClient map_client = n_.serviceClient<nav_msgs::GetMap>("static_map");
    if (map_client.call(req, res)) {
        setGrid(res.map);
        return true;
    }
    else {
        ROS_ERROR("Failed to call map service");
        return false;
    }
}

void NavigationPlanner::setGrid(const nav_msgs::OccupancyGrid& map){
    // ROS_INFO("NavigationPlanner::setGrid-----------------------------------------------");

    grid_rows_ = map.info.height;
    grid_cols_ = map.info.width;
    grid_resolution_ = map.info.resolution;
    grid_origin_ = map.info.origin;

    ROS_INFO("Received a %d X %d map @ %.3f m/px with origin=(%f, %f)",
    grid_cols_,
    grid_rows_,
    grid_resolution_,
    grid_origin_.position.x,
    grid_origin_.position.y);

    grid_.resize(grid_rows_);
    // map_.resize(grid_rows_);
    for (int i = 0; i < grid_rows_; i++) {
        grid_[i].resize(grid_cols_);
        // map_.resize(grid_cols_);
    }

    // map_ = new std::vector<std::vector<Position*>>();
    map_.resize(grid_rows_, std::vector<Position*>(grid_cols_, new Position()));

    int currCell = 0, i, j;
    double x_coord = 0, y_coord = 0;

    for (i = 0; i < grid_rows_; i++) {
        for (j = 0; j < grid_cols_; j++) {
            if (map.data[currCell] == 0)
                grid_[j][i] = false;
            else
                grid_[j][i] = true;
            currCell++;
        }
    }

    for (i = 0; i < grid_rows_; i++) {
        x_coord = i * grid_resolution_;
        for (j = 0; j < grid_cols_; j++) {
            y_coord = j * grid_resolution_;
            map_[i][j] = new Position();
            map_[i][j]->setPosition(x_coord, y_coord, 0.0);
            map_[i][j]->setGridPosition(i,j);

            // if (map.data[currCell] == 0)
            //     grid_[i][j] = false;
            // else
            //     grid_[i][j] = true;
            map_[i][j]->setWalkable(!grid_[i][j]);
            // currCell++;
        }
    }
    // for (j = 2090; j < 2111; j++) {
    //     i = 1930;
    //         ROS_INFO("i=%d j=%d walkable=%d", i, j, map_[i][j]->getWalkable());
    // }
}

void NavigationPlanner::printGrid(){
    printf("Grid map:\n");
    for (int i = 1880; i < 2150; i+=1)
    {
        // printf("Row no. %d\n", i);
        for (int j = 1880; j < 2150; j+=1)
        {
            if(i>=1920 && i<=2110 && j==1930){
                printf("%c ", grid_[i][j] ? '2' : '2');
            }
            else{
                printf("%c ", grid_[i][j] ? '|' : ' ');
            }
        }
        printf("\n");
    }
}