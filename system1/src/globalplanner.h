class GlobalPlanner{
    private:
        std::vector<std::vector<Position*> > map_;
        std::vector<Position*> plan_;
        geometry_msgs::Pose grid_origin_;
        double grid_resolution_;
        float robot_radius_;

        Position *home_, *goal_;

    public:
        GlobalPlanner();
        std::vector<Position*> getPlan(std::vector<std::vector<Position*> > map, 
            geometry_msgs::Pose grid_origin, double resolution, float robot_radius,
            Position* home, Position* goal);
        std::vector<Position*> generatePlan();
        Position* getPositionAt(int x, int y);
        bool isWalkableAt(int x, int y);
        ~GlobalPlanner();
};