#include "sensor.cpp"

class LocalPlanner{
    private:
        ros::NodeHandle n_;
        Sensor sensor_;
        //Linear and angular velocity publisher
        ros::Publisher pub_move_;

        float oscillation_timeout_, oscillation_distance_;
        Position *goal_, *robot_position_;
        std::vector<Position*> plan_;

    public:
        LocalPlanner();
        void initLocalPlanner(ros::NodeHandle &n, float oscillation_distance, float oscillation_timeout);
        int executePlan(std::vector<Position*> plan, Position* goal, Position* robot_position);
        bool checkIfOnSameLine(Position* A, Position* B, Position* C);
        bool moveRobot(Position* goal);
        bool rotateRobot(Position *goal);
        void setRobotPosition(double rp_x, double rp_y, double rp_a, int row, int col);
        void moveToSafety(int zone_code);
        ~LocalPlanner();
};