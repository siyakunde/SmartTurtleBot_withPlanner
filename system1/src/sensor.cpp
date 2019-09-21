#include "sensor.h"

Sensor::Sensor(){
    ROS_INFO("Sensor::Sensor----------------------------------------------");
    
    angle_[0]=0; distance_[0]=0.4;
    angle_[1]=1.2; distance_[1]=0;
    angle_[10]=1.57; distance_[10]=0;
    angle_[11]=1.57; distance_[11]=-0.2;
    angle_[100]=-1.2; distance_[100]=0;
    angle_[101]=3; distance_[101]=0;
    angle_[110]=-1.57; distance_[110]=-0.2;
    angle_[111]=3; distance_[111]=-0.5;
    angle_[3]=3; distance_[3]=-0.5;

    zone1_ = zone2_ = zone3_ = 0;
}

void Sensor::initSensor(ros::NodeHandle &n){
    // ROS_INFO("Sensor::initSensor----------------------------------------------");
    n_ = n;
    //subscribe to /scan topic to receive laser scanner data. This is used to detect obstacles.
    scan_sub_ = n_.subscribe("scan", 1000, &Sensor::onScan, this);
    //subscribe to the /mobile_base/events/bumper topic. This is used to detect if the robot ran head-first into an obstacle.
    bumper_sub_ = n_.subscribe("mobile_base/events/bumper", 1000, &Sensor::onBumperHit, this);
    
}

//Callback for the scan_sub_ subscriber, so that once laser scanner has some data, we can process it to avoid obstacles.
void Sensor::onScan (const sensor_msgs::LaserScan::ConstPtr& scanData){
    // ROS_INFO("Sensor::onScan----------------------------------------------");
    RANGE_MIN = scanData->range_min;
    RANGE_MAX = scanData->range_max;

    int i = 0, r_size = (scanData->ranges).size();
    float r = 0.0, check_distance = 0.25;
    reset();
    for(i=0; i<r_size; i++){
        r = scanData->ranges[i];

        if(r > RANGE_MIN && r < RANGE_MAX){
            zone1_ = (i > r_size/2 && i < r_size && r <= check_distance) ? 1 : 0;
            zone2_ = (i >= r_size/3 && i <= r_size/2 && r <= check_distance) ? 1 : 0;
            zone3_ = (i > 0 && i < r_size/3 && r <= check_distance) ? 1 : 0;
        }
    }
}

//Callback for bumper_sub_ subscriber. If bumper is pressed, GET THE ROBOT OUT OF THERE!
void Sensor::onBumperHit (const kobuki_msgs::BumperEvent::ConstPtr& bumperData){
    // ROS_INFO("Sensor::onBumperHit----------------------------------------------");
    
    if(bumperData->state == kobuki_msgs::BumperEvent::PRESSED){
        int k;
        reset();
        for(k=0; k<=5000; k++){
            zone1_ = 0;
            zone2_ = 0;
            zone3_ = 3;
        }   
    }
}

int Sensor::isRobotFacingObstacle(){
    // ROS_INFO("Sensor::isRobotFacingObstacle----------------------------------------------");
    return (StringToInt(IntToString(zone1_) + IntToString(zone2_) + IntToString(zone3_)));
}

float Sensor::getObstacleFreeLinearVelocity(int zone_code){
    // ROS_INFO("Sensor::getObstacleFreeLinearVelocity--------------------------------------");
    return distance_[zone_code];
}

float Sensor::getObstacleFreeAngularVelocity(int zone_code){
    // ROS_INFO("Sensor::getObstacleFreeAngularVelocity-------------------------------------");
    return angle_[zone_code];
}

//Reset the zone values
void Sensor::reset(){
    zone1_ = 0;
    zone2_ = 0;
    zone3_ = 0;
}

//Converts integer to string
std::string Sensor::IntToString(int a){
    std::ostringstream temp;
    temp<<a;
    return temp.str();
}

//Converts string to integer
int Sensor::StringToInt(std::string s){
    int i;
    std::istringstream ss(s);
    ss >> i;
    return i;
}

Sensor::~Sensor() {
    
}