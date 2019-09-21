#include "ros/ros.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <string>
using std::string;

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

#include "position.cpp"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NavigationPlanner{
private:
	ros::NodeHandle n_;
	ros::Subscriber pose_sub_;
	Position home_, goal_;
    std::vector<Position> plan_;
    double goal_tolerance_;

    double total_distance_travelled_;
	//Record the current and previous positions of the robot
	double prev_pose_X_, prev_pose_Y_, curr_pose_X_, curr_pose_Y_;
    
    // void readMap(const nav_msgs::OccupancyGrid& msg);
public:
	NavigationPlanner();
	void setHome(double goal_x, double goal_y, double degree_w);
	void setGoal(double goal_x, double goal_y, double degree_w);
	void setPlan(std::vector<Position> newPlan);
	void setGoalTolerance(double newGoalTolerance);
	void goToWaypoint();
	bool sendGoal(Position g, bool final);

	double getDistanceTravelled();
    void onRobotPosition (const nav_msgs::Odometry::ConstPtr& poseData);
};