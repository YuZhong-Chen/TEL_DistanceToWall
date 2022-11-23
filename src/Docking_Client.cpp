#include "distance_to_wall/Docking.h"
#include "ros/ros.h"

static double Distance;

int main(int argc, char **argv) {
    ros::init(argc, argv, "Docking_client");

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<distance_to_wall::Docking>("/DockingService");

    if (!nh.getParam("/Docking_Client/Distance", Distance)) {
        Distance = 0;
    }

    distance_to_wall::Docking srv;

    srv.request.Distance = Distance;

    if (client.call(srv)) {
        ROS_INFO("Start Docking. Target : %.1f.", srv.request.Distance);
    } else {
        ROS_ERROR("Failed to call service Docking_Server.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}