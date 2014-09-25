#ifndef PATROLPATH_H_
#define PATROLPATH_H_

#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>

#include <utility>
#include <vector>
#include <string>
#include <cmath>
using namespace std;

// We define location as x,y
typedef pair<double, double> Location;

// A class for calculating and managing the patrol path of the robot
class PatrolPath
{
public:
    PatrolPath(const vector<Location> &locations);
    pair<Location, double> getNextLocation();

private:
    const string SERVICE_NAME;
    const string WORLD_FRAME;
    double goalTolerance;

    ros::NodeHandle nh;
    ros::ServiceClient serviceClient;

    // Stuff for the algorithm
    int numberOfLocations;
    double **distanceMatrix;
    vector<Location> locations;
    vector<int> path;
    int nextLocation;
    double nextDistance;
    int patrolDirection;

    void fillDistanceMatrix();
    double calcDistance(const Location &start, const Location &goal);
    void initStartLocation();
    
    // 1-NN Algorithm
    double calcShortestPath();
};

#endif
