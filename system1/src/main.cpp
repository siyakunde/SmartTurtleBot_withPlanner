#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "navigation_planner.cpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "system1_node");
    ROS_INFO("MAIN---------------------------------------------------------------------");

    // tf::TransformListener listener; 
    // listener.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(10.0));
    // try {            
    //     tf::StampedTransform transform;   
    //     listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
    //     double x = transform.getOrigin().x();
    //     double y = transform.getOrigin().y();
    //     ROS_INFO("Current robot position: x=%lf",x);
    // } catch (tf::TransformException &ex) {
    //     ROS_ERROR("%s",ex.what());
    // }

    // Get the goal's x, y and angle from the launch file
    double initial_x, initial_y, initial_a,
            goal_x, goal_y, goal_a, 
            goal_tolerance;
    float robot_radius, oscillation_timeout, oscillation_distance;
    ros::NodeHandle n;

    n.getParam("initial_x", initial_x);
    n.getParam("initial_y", initial_y);
    n.getParam("initial_a", initial_a);

    n.getParam("goal_x", goal_x);
    n.getParam("goal_y", goal_y);
    n.getParam("goal_a", goal_a);

    n.getParam("xy_goal_tolerance", goal_tolerance);

    n.getParam("robot_radius", robot_radius);

    n.getParam("oscillation_timeout", oscillation_timeout);

    n.getParam("oscillation_distance", oscillation_distance);

    NavigationPlanner np = NavigationPlanner(n, robot_radius, goal_tolerance, oscillation_timeout, oscillation_distance);

    np.setHome(initial_x, initial_y, initial_a);
    np.setRobotPosition(initial_x, initial_y, initial_a);
    np.setGoal(goal_x, goal_y, goal_a);

    np.getGrid();
    // np.printGrid();
    ros::Duration(2).sleep();
    do{
        np.goToWaypoint();
        np.setHome(np.getRobotPosition()->getPositionX(), 
                    np.getRobotPosition()->getPositionY(), 
                    np.getRobotPosition()->getPositionA());
    }while(0);
    // (np.getRobotPosition()->getPositionX() - np.getGoal()->getPositionX() 
    //         > goal_tolerance)
    //     || (np.getRobotPosition()->getPositionY() - np.getGoal()->getPositionY() 
    //         > goal_tolerance));
    ROS_INFO("Total distance travelled = %lf", np.getDistanceTravelled());

    return 0;
}





