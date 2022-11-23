#include "distance_to_wall/DockingClass.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"

static bool DebugMode = false;

static double Param_Left_a = 1.0;
static double Param_Left_b = 0.0;
static double Param_Right_a = 1.0;
static double Param_Right_b = 0.0;

static double Left_Standard = 100.0;
static double Right_Standard = 100.0;

static double CalibrationRange = 10.0;

static double Param_CarVelocity = 0.0;
static double CurCarVelocity = 0.0;

int main(int argc, char** argv) {
    ros::init(argc, argv, "DistanceToWall");

    // ------------------------------------------------------------
    // Setup the Ros Node.
    // ------------------------------------------------------------
    ros::NodeHandle nh("~");

    // ------------------------------------------------------------
    // Get Parameter from roslaunch.
    // ------------------------------------------------------------
    nh.param<bool>("DebugMode", DebugMode, false);

    nh.param<double>("Left_a", Param_Left_a, 1.0);
    nh.param<double>("Left_b", Param_Left_b, 0.0);

    nh.param<double>("Right_a", Param_Right_a, 1.0);
    nh.param<double>("Right_b", Param_Right_b, 0.0);

    nh.param<double>("Left_Standard", Left_Standard, 100.0);
    nh.param<double>("Right_Standard", Right_Standard, 100.0);

    nh.param<double>("CalibrationRange", CalibrationRange, 10.0);

    nh.param<double>("CarVelocity", Param_CarVelocity, 0.05);

    // ------------------------------------------------------------
    // Init Docking.
    // ------------------------------------------------------------
    Docking.Init(&nh, Param_Left_a, Param_Left_b, Param_Right_a, Param_Right_b, Param_CarVelocity);

    // ------------------------------------------------------------
    // Ros spin.
    // ------------------------------------------------------------
    ros::spin();

    return 0;
}