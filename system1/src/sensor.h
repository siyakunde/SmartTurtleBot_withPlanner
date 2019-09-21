#include <sensor_msgs/LaserScan.h>
#include <kobuki_msgs/BumperEvent.h>

class Sensor{
    private:
        ros::NodeHandle n_;
        //Laser Scanner subscriber
        ros::Subscriber scan_sub_;
        //BumperEvent subscriber
        ros::Subscriber bumper_sub_;

        //The values from ranges array, returned by scan data, should fall between these values
        float RANGE_MIN, RANGE_MAX;
        //Divides the robots field of vision into 3 zones to detect where the obstacle is
        int zone1_, zone2_, zone3_;
        //These maps define how the robot will act if there is/isn't an obstacle
        std::map<int,float> angle_;
        std::map<int,float> distance_;

    public:
        Sensor();
        void initSensor(ros::NodeHandle &n);

        //Callback for the scan_sub_ subscriber, so that once laser scanner has some data, we can process it to avoid obstacles.
        void onScan (const sensor_msgs::LaserScan::ConstPtr& scanData);
        //Callback for bumper_sub_ subscriber. If bumper is pressed, GET THE ROBOT OUT OF THERE!
        void onBumperHit (const kobuki_msgs::BumperEvent::ConstPtr& bumperData);

        int isRobotFacingObstacle();
        float getObstacleFreeLinearVelocity(int zone_code);
        float getObstacleFreeAngularVelocity(int zone_code);
        //Reset the zone values
        void reset();
        //Converts integer to string
        std::string IntToString (int a);
        //Converts string to integer
        int StringToInt(std::string s);
        ~Sensor();
};