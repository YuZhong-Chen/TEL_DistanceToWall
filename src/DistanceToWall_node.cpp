#include "distance_to_wall/DockingClass.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "DistanceToWall");

    // ------------------------------------------------------------
    // Setup the Ros Node.
    // ------------------------------------------------------------
    ros::NodeHandle nh_Public;
    ros::NodeHandle nh_Private("~");

    // ------------------------------------------------------------
    // Init Docking.
    // ------------------------------------------------------------
    Docking.Init(&nh_Public, &nh_Private);

    // ------------------------------------------------------------
    // Ros loop.
    // ------------------------------------------------------------

    ros::Rate CheckFrequency(10);
    while (nh_Public.ok()) {
        Docking.UpdateStatus();

        ros::spinOnce();
        CheckFrequency.sleep();
    }

    return 0;
}