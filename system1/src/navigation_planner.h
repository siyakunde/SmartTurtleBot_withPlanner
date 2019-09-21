#include "ros/ros.h"

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <vector>

#include <string>
#include <math.h>
using std::string;
#include <list>

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

#include "position.cpp"
#include "pdcontroller.cpp"
#include "pcontroller.cpp"
#include "globalplanner.cpp"
#include "localplanner.cpp"

class NavigationPlanner{
private:
	ros::NodeHandle n_;

	GlobalPlanner gp_;
	LocalPlanner lp_;

	Position *home_, *goal_, *robot_position_, *robot_offset_;
	double total_distance_travelled_;
	//Record the current and previous positions of the robot
	double prev_pose_X_, prev_pose_Y_, curr_pose_X_, curr_pose_Y_;
    std::vector<Position*> plan_;
    double goal_tolerance_;
    float robot_radius_, oscillation_timeout_, oscillation_distance_;

    int grid_rows_;
    int grid_cols_;
    double grid_resolution_;
    geometry_msgs::Pose grid_origin_;
    std::vector<std::vector<bool> > grid_;
    std::vector<std::vector<Position*> > map_;

	ros::Subscriber pose_sub_;

	void onRobotPosition(const nav_msgs::Odometry::ConstPtr& poseData);
    void setGrid(const nav_msgs::OccupancyGrid& msg);
public:
	NavigationPlanner(ros::NodeHandle &n, float robot_radius, double goal_tolerance, 
		float oscillation_timeout, float oscillation_distance);

	void setHome(double goal_x, double goal_y, double goal_a);
	Position* getHome();
	void setGoal(double goal_x, double goal_y, double goal_a);
	Position* getGoal();
	void setRobotPosition(double rp_x, double rp_y, double rp_a);
	Position* getRobotPosition();
	void onDistanceTravelled ();
	double getDistanceTravelled();

	void setPlan(std::vector<Position*> newPlan);

    bool getGrid();
    void printGrid();

	void goToWaypoint();
};