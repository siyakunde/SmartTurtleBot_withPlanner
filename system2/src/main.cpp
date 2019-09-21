#include <ros/ros.h>

#include "navigation_planner.cpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "system2_node");

    NavigationPlanner np;

    // Get the goal's x, y and angle from the launch file
    double initial_x, initial_y, initial_a,
            goal_x, goal_y, degree_a, 
            goal_tolerance;
    ros::NodeHandle n;
    
    n.getParam("initial_x", initial_x);
    n.getParam("initial_y", initial_y);
    n.getParam("initial_a", initial_a);
    np.setHome(initial_x, initial_y, initial_a);

    n.getParam("goal_x", goal_x);
    n.getParam("goal_y", goal_y);
    n.getParam("goal_a", degree_a);
    np.setGoal(goal_x, goal_y, degree_a);

    n.getParam("xy_goal_tolerance", goal_tolerance);
    np.setGoalTolerance(goal_tolerance);

    np.goToWaypoint();
    ROS_INFO("Total distance travelled = %lf", np.getDistanceTravelled());
    
    return 0;
}





