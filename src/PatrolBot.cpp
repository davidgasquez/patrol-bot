#include "PatrolPath.h"
#include <stdlib.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <fstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Parse the locations string into a vector of locations
vector<Location> parseLocations(string locations_x, string locations_y)
{

    // Create positions at the fisrt comma
    int pos_x = locations_x.find(',');
    int pos_y = locations_y.find(',');

    // If there are no locations in the launch file
    if (pos_x == string::npos) // npos is the same as "-1"
    {
        ROS_FATAL("There are no locations in the launch file");
    }

    // Split the locations strings with delimiter ',' and parse the locations
    vector<Location> locations;

    while (pos_x != string::npos)
    {
        // Transform the location before comma to double - atof
        double x = atof(locations_x.substr(0, pos_x).c_str());
        double y = atof(locations_y.substr(0, pos_y).c_str());
        locations.push_back(Location(x, y));

        // Erase the first location from the strings
        locations_x.erase(0, pos_x + 1);
        locations_y.erase(0, pos_y + 1);
        pos_x = locations_x.find(',');
        pos_y = locations_y.find(',');
    }

    // Add last location
    double x = atof(locations_x.c_str());
    double y = atof(locations_y.c_str());
    locations.push_back(Location(x, y));

    return locations;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "patrol_bot");

    ros::NodeHandle nh;

    // Get the locations parameters
    string locations_x, locations_y;
    nh.getParam("locations_x", locations_x);
    nh.getParam("locations_y", locations_y);

    // Parsing the locations of the patrol into a vector of locations
    ROS_INFO("Parsing the locations list of the patrol");
    vector<Location> locations = parseLocations(locations_x, locations_y);

    // Initiate a patrol path
    PatrolPath patrolPath(locations);

    // Create the action client
    MoveBaseClient ac("move_base", true);

    // Wait 60 seconds for the action server to become available
    ROS_INFO("Waiting for the move_base action server");
    ac.waitForServer(ros::Duration(60));
    ROS_INFO("Connected to move base server");

    // Get the pause time at every location parameter
    double pauseTime;
    nh.getParam("pause_time", pauseTime);

    // Variables Initialization for log
    int visitedLocationsCount = 0;
    double totalDistance = 0.0;

    //Initialize the timer
    ros::Time startingTime = ros::Time::now();

    // Get the log file name
    string logFilename;
    nh.getParam("log_file", logFilename);

    // Open the log file for writing
    ofstream log;
    log.open(logFilename.c_str());

    // Keep patrolling until user exits the program
    while (ros::ok())
    {
        pair<Location, double> nextLocation = patrolPath.getNextLocation();

        // Fill the goal message
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = nextLocation.first.first;
        goal.target_pose.pose.position.y = nextLocation.first.second;
        goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Sending next goal: %.3f, %.3f", nextLocation.first.first, nextLocation.first.second);
        ac.sendGoal(goal);

        // Wait for the action to return
        ac.waitForResult();

        // Write the log file
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            // Add the distance travelled
            totalDistance += nextLocation.second;
            log << "[Success] ";
        }
        // Making sure it's really a failure and not the end of the program
        else if (ros::ok())
        {
            log << "[Failure] ";
        }

        if (ros::ok())
        {
            log << "Total locations visited: " << ++visitedLocationsCount << ". Total distance: " << totalDistance  << ". Elapsed time: " << (ros::Time::now() - startingTime).sec << " seconds\n";

            // Pause in the position
            ROS_INFO("Pausing for %.3f seconds", pauseTime);
            ros::Duration(pauseTime).sleep();
        }
    }

    // Close log file
    log.close();

    return 0;
}
