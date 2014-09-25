#include "PatrolPath.h"
#include <limits>

//  Initialize the PatrolPath with move_base_node, better than our potential planner
PatrolPath::PatrolPath(const std::vector<Location> &locations)
    : SERVICE_NAME("move_base_node/make_plan"), WORLD_FRAME("map")
{
    // Get the goal tolerance as a parameter. Easier to test the patrol bot
    nh.getParam("goal_tolerance", goalTolerance);

    // Add the locations to our private locations
    this->locations = locations;
    numberOfLocations = locations.size();

    // Waiting for service to become available
    while (!ros::service::waitForService(SERVICE_NAME, ros::Duration(3.0)))
    {
        ROS_INFO("Waiting for service move_base_node/make_plan to become available");
    }

    // Initiate the service client
    serviceClient = nh.serviceClient<nav_msgs::GetPlan>(SERVICE_NAME, true);
    if (!serviceClient)
    {
        ROS_FATAL("Could not initialize get plan service from %s", serviceClient.getService().c_str());
    }

    // Fill the distance matrix
    ROS_INFO("Calculating the distance between each two locations");
    fillDistanceMatrix();

    // Calculate the shortest path
    ROS_INFO("Calculating the path with minimal distance");
    double pathDistance = calcShortestPath();
    ROS_INFO("Total path distance: %f", pathDistance);

    // Set the first location of the path to be the closest to the initial pose
    initStartLocation();
    ROS_INFO("Patrol path is ready");
}

// Fill the distance matrix
// distanceMatrix[i][j] := The distance of the path from locations[i] to locations[j]
void PatrolPath::fillDistanceMatrix()
{
    // Allocating the memory
    distanceMatrix = new double*[numberOfLocations];
    for (int i = 0; i < numberOfLocations; i++)
    {
        distanceMatrix[i] = new double[numberOfLocations];
    }

    // Calculating the distance between each two locations
    for (int i = 0; i < numberOfLocations - 1 ; i++) // -1 as we dont need to calculate the distance of the last one
    {
        for (int j = i + 1; j < numberOfLocations; j++) //  (j = i+1) because the distance matrix is simetric
        {
            double distance = calcDistance(locations[i], locations[j]);

            // In case the plan fails, trying to calculate the distance in the opposite way
            if (distance == numeric_limits<double>::infinity())
            {
                distance = calcDistance(locations[j], locations[i]);
            }

            // Equal distance for both ways
            distanceMatrix[i][j] = distance;
            distanceMatrix[j][i] = distance;
        }
    }
}

// Calculate the distance between the two locations
double PatrolPath::calcDistance(const Location &start, const Location &goal)
{
    nav_msgs::GetPlan srv;

    // Filling the request
    srv.request.start.header.frame_id = WORLD_FRAME;
    srv.request.start.pose.position.x = start.first;
    srv.request.start.pose.position.y = start.second;

    srv.request.goal.header.frame_id = WORLD_FRAME;
    srv.request.goal.pose.position.x = goal.first;
    srv.request.goal.pose.position.y = goal.second;

    srv.request.tolerance = goalTolerance;

    double distance = 0.0;

    // Get the plan from the service client
    if (serviceClient.call(srv))
    {
        int numberOfPoses = srv.response.plan.poses.size();

        // Check if there is no path between the locations
        if (numberOfPoses == 0)
        {
            distance = numeric_limits<double>::infinity();
            ROS_WARN("Cannot reach from (%f, %f) to (%f, %f)", start.first, start.second, goal.first, goal.second);
        }
        else
        {
            // Sum the distance between each two points on the path
            for (int i = 0; i < numberOfPoses - 1; i++)
            {
                const geometry_msgs::PoseStamped &p1 = srv.response.plan.poses[i];
                const geometry_msgs::PoseStamped &p2 = srv.response.plan.poses[i + 1];

                double a = p2.pose.position.x - p1.pose.position.x;
                double b = p2.pose.position.y - p1.pose.position.y;

                // Straight-line distance
                distance += sqrt((a * a) + (b * b));
            }
        }
    }
    else
    {
        ROS_ERROR("Failed to call service %s", serviceClient.getService().c_str());
    }

    return distance;
}

// Returns the location and distance of the next location in the patrol path
pair<Location, double> PatrolPath::getNextLocation()
{
    Location location = locations[path[nextLocation]];
    double distance = nextDistance;

    // Check if we stand to the edge of the path, changing direction of movement
    if (nextLocation == numberOfLocations - 1)
    {
        patrolDirection = -1;
    }
    else if (nextLocation == 0)
    {
        patrolDirection = 1;
    }

    // Set the next location and distance for next iteration
    nextDistance = distanceMatrix[nextLocation][nextLocation + patrolDirection];
    nextLocation += patrolDirection;

    // Return the next location and distance
    return pair<Location, double>(location, distance);
}

// Approximating the shortest path by using the nearest neighbor algorithm on every possible
// starting point, and taking the path with the minimum distance
double PatrolPath::calcShortestPath()
{
    double minPathDistance = numeric_limits<double>::infinity();

    // For each starting point
    for (int j = 0; j < numberOfLocations; j++)
    {
        vector<int> currentPath;
        double pathDistance = 0.0;

        // Initiate all the locations not in the path yet
        vector<int> locationsNotVisited;
        for (int i = 0; i < numberOfLocations; i++)
        {
            locationsNotVisited.push_back(i);
        }

        // Setting the first location
        int currentLocationIndex = j;
        currentPath.push_back(currentLocationIndex);
        locationsNotVisited.erase(locationsNotVisited.begin() + j);

        // Keep adding the nearest next location until visited all locations
        while (!locationsNotVisited.empty())
        {

            // Initiate variables to find the nearest neighbor
            double minDistance = numeric_limits<double>::infinity();
            int nextLocationIndex;
            vector<int>::iterator minLocationNotVisited;

            // Find the index of the nearest next location
            for (vector<int>::iterator it = locationsNotVisited.begin(); it != locationsNotVisited.end(); it++)
            {
                if (distanceMatrix[currentLocationIndex][*it] < minDistance)
                {
                    minDistance = distanceMatrix[currentLocationIndex][*it];
                    nextLocationIndex = *it;
                    minLocationNotVisited = it;
                }
            }

            // Add next location to the path and update total path distance
            currentPath.push_back(nextLocationIndex);
            pathDistance += minDistance;
            currentLocationIndex = nextLocationIndex;
            locationsNotVisited.erase(minLocationNotVisited);
        }

        // Update the path with the minimum distance
        if (pathDistance < minPathDistance)
        {
            minPathDistance = pathDistance;
            path = currentPath;
        }
    }

    return minPathDistance;
}

// Set the first location for the patrol path
void PatrolPath::initStartLocation()
{
    // Get the initial pose
    double initial_pose_x, initial_pose_y;
    nh.getParam("amcl/initial_pose_x", initial_pose_x);
    nh.getParam("amcl/initial_pose_y", initial_pose_y);
    Location initialPose(initial_pose_x, initial_pose_y);

    /*
    AMCL is a probabilistic localization system for a robot moving in 2D.
    It implements the adaptive (or KLD-sampling) Monte Carlo localization
    approach (as described by Dieter Fox), which uses a particle filter to
    track the pose of a robot against a known map.
    */

    // Calculate the distance to each location and keep the closest location for the 1-NN
    double minDistance = numeric_limits<double>::infinity();
    int closestLocationIndex = 0;
    for (int i = 0; i < numberOfLocations; i++)
    {
        double distance = calcDistance(initialPose, locations[path[i]]);
        if (distance < minDistance)
        {
            minDistance = distance;
            closestLocationIndex = i;
        }
    }

    // Set variables for the next location
    nextLocation = closestLocationIndex;
    nextDistance = minDistance;
    patrolDirection = 1;
}
