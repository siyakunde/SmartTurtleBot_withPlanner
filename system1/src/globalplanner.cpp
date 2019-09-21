#include "globalplanner.h"

GlobalPlanner::GlobalPlanner(){}

std::vector<Position*> GlobalPlanner::getPlan(std::vector<std::vector<Position*> > map, 
    geometry_msgs::Pose grid_origin, double resolution, float robot_radius,
    Position* home, Position* goal){

    ROS_INFO("GlobalPlanner::getPlan-----------------------------------------------");
    map_ = map;
    grid_origin_ = grid_origin;
    grid_resolution_ = resolution;
    robot_radius_ = robot_radius;
    home_ = home;
    goal_ = goal;
    if (map_.size() > 0) {
        generatePlan();
    }
    else {
        ROS_WARN("No plan available");
    }
    return plan_;
}

std::vector<Position*> GlobalPlanner::generatePlan(){
    ROS_INFO("GlobalPlanner::generatePlan-----------------------------------------------");
    ROS_INFO("Generating new plan");
    // Define points to work with
    Position *start = home_;
    Position *end = goal_;
    Position *current;
    Position *child;

    ROS_INFO("start %d,%d, f=%f, g=%f, h=%f"
        , start->getPositionGridX()
        , start->getPositionGridY()
        , start->getFScore()
        , start->getGScore()
        , start->getHScore()
        );
    ROS_INFO("end %d,%d, f=%f, g=%f, h=%f"
        , end->getPositionGridX()
        , end->getPositionGridY()
        , end->getFScore()
        , end->getGScore()
        , end->getHScore()
        );

    // Define the open and the close list
    std::list<Position*> openList;
    std::list<Position*> closedList;
    std::list<Position*>::iterator i;

    unsigned int n = 0;
    int x, y;
    int neighbourX, neighbourY;

    // Add the start point to the openList
    openList.push_back(start);
    start->setOpened(true);
                    
    do
    {

        for (i = openList.begin(); i != openList.end(); ++ i){
            if (i == openList.begin() || (*i)->getFScore() <= current->getFScore()){
                current = (*i);
            }
        }
        ROS_INFO("current %d,%d, f=%f, g=%f, h=%f"
                , current->getPositionGridX()
                , current->getPositionGridY()
                , current->getFScore()
                , current->getGScore()
                , current->getHScore()
                );

        // Stop if we reached the end
        if (current == end){
            break;
        }

        // Remove the current point from the openList
        openList.remove(current);
        current->setOpened(false);

        // Add the current point to the closedList
        closedList.push_back(current);
        current->setClosed(true);

        // Get all current's adjacent walkable points
        for (x = -1; x < 2; x ++){
            // ROS_INFO("For 1");
            for (y = -1; y < 2; y ++){
                // ROS_INFO("For 2");
                // If it's current point then pass
                if (x == 0 && y == 0){
                    continue;
                }   

                neighbourX = (current->getPositionGridX() + x);
                neighbourY = (current->getPositionGridY() + y);

                try{
                    child = getPositionAt(neighbourX, neighbourY);
                }catch(const std::exception& e){
                    child = NULL;
                }
                    
                if(child){

                    // If it's closed or not walkable then pass
                    if (child->getClosed() || !isWalkableAt(child->getPositionGridX(), child->getPositionGridY())){
                        continue;
                    }

                    // // If we are at a corner
                    // if (x != 0 && y != 0){
                    //     // ROS_INFO("x!=0 && y!=0");
                    //     // if the next horizontal point is not walkable or in the closed list then pass
                    //     if (!(getPositionAt(current->getPositionGridX(), current->getPositionGridY() + y)->getWalkable()) || getPositionAt(current->getPositionGridX(), current->getPositionGridY() + y)->getClosed())
                    //     {
                    //         // ROS_INFO("Next horizontal is not walkable");
                    //         continue;
                    //     }
                    //     // if the next vertical point is not walkable or in the closed list then pass
                    //     if (!(getPositionAt(current->getPositionGridX() + x, current->getPositionGridY())->getWalkable()) || getPositionAt(current->getPositionGridX() + x, current->getPositionGridY())->getClosed())
                    //     {
                    //         // ROS_INFO("Next vertical is not walkable");
                    //         continue;
                    //     }
                    // }

                    // If it's already in the openList
                    if (child->getOpened()){
                        // If it has a wroste g score than the one that pass through the current point
                        // then its path is improved when it's parent is the current point
                        if (child->getGScore() > child->getGScore(current)){
                            // Change its parent and g score
                            child->setParent(current);
                            child->computeScores(end);
                        }
                    }
                    else{
                        // Add it to the openList with current point as parent
                        openList.push_back(child);
                        child->setOpened(true);

                        // Compute it's g, h and f score
                        child->setParent(current);
                        child->computeScores(end);
                    }
                }  
            }
        }
        n ++;
    } while ( !(current->getPositionGridX() == end->getPositionGridX() && 
             current->getPositionGridY() == end->getPositionGridY()) );

    // Reset
    for (i = openList.begin(); i != openList.end(); ++ i){
        (*i)->setOpened(false);
    }
    for (i = closedList.begin(); i != closedList.end(); ++ i){
        (*i)->setClosed(false);
    }
    plan_.resize(openList.size(), new Position());

    n = 0;
    // Resolve the path starting from the end point
    while (current->hasParent() && current != start){
        plan_[n] = new Position();
        plan_[n] = current->getPosition();
        current = current->getParent();
        // ROS_INFO("Path-point ----------------i=%d j=%d x=%lf y=%lf------------------------------",
        //     plan_[n]->getPositionGridX(), plan_[n]->getPositionGridY(),
        //     plan_[n]->getPositionX(), plan_[n]->getPositionY());
        n++;

    }
    plan_.resize(n);
    return plan_;
}

Position* GlobalPlanner::getPositionAt(int x, int y){
    return map_[x][y];
}

bool GlobalPlanner::isWalkableAt(int x, int y){
    int i, j;
    bool isWalkable = true;
    Position *tempChild;
    if(robot_radius_ > grid_resolution_){
        for(i=-robot_radius_/(2*grid_resolution_); i<=robot_radius_/(2*grid_resolution_); i++){
            for(j=-robot_radius_/(2*grid_resolution_); j<=robot_radius_/(2*grid_resolution_); j++){
                try{
                    tempChild = getPositionAt(x+i, y+j);
                }catch(const std::exception& e){
                    tempChild = NULL;
                }
                isWalkable = isWalkable && tempChild && tempChild->getWalkable();
            }
        }
    }
    else{
        try{
            tempChild = getPositionAt(x, y);
        }catch(const std::exception& e){
            tempChild = NULL;
        }
        isWalkable = isWalkable && tempChild && tempChild->getWalkable();
    }
    return isWalkable;
}

GlobalPlanner::~GlobalPlanner() {
    
}