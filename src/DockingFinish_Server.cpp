#include "distance_to_wall/DockingFinish.h"
#include "ros/ros.h"

static bool Callback(distance_to_wall::DockingFinish::Request& req, distance_to_wall::DockingFinish::Response& res);

int main(int argc, char** argv) {
    ros::init(argc, argv, "DockingFinish_server");

    ros::NodeHandle nh;

    ros::ServiceServer Server = nh.advertiseService("/DockingFinish", Callback);

    ros::spin();

    return 0;
}

static bool Callback(distance_to_wall::DockingFinish::Request& req, distance_to_wall::DockingFinish::Response& res) {
    if (req.isFinished) {
        ROS_INFO("DockingFinish Server: Success.");
    } else {
        ROS_ERROR("DockingFinish Server: Fail.");
    }
    return true;
}