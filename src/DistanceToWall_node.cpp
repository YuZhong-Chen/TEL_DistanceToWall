#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int16MultiArray.h"

std_msgs::Float64MultiArray Data;

double Left_a = 1.0;
double Left_b = 0.0;
double Right_a = 1.0;
double Right_b = 0.0;

// 0 -> left
// 1 -> right
void CallBack_VL53(const std_msgs::Int16MultiArray::ConstPtr& msg) {
    Data.data[0] = (double)msg->data[0] * Left_a + Left_b;
    Data.data[1] = (double)msg->data[1] * Right_a + Right_b;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "DistanceToWall");

    // ------------------------------------------------------------
    // Setup the Ros Node.
    // ------------------------------------------------------------
    ros::NodeHandle nh("~");

    ros::Publisher DistanceToWall_pub = nh.advertise<std_msgs::Float64MultiArray>("DistanceToWall", 1000);
    ros::Subscriber VL53_sub = nh.subscribe("/CarDistanceFront", 1000, CallBack_VL53);

    // ------------------------------------------------------------
    // Get Parameter from roslaunch.
    // ------------------------------------------------------------

    if (!nh.getParam("Left_a", Left_a)) {
        Left_a = 1.0;
    }
    if (!nh.getParam("Left_b", Left_b)) {
        Left_b = 0.0;
    }
    if (!nh.getParam("Right_a", Right_a)) {
        Right_a = 1.0;
    }
    if (!nh.getParam("Right_b", Right_b)) {
        Right_b = 0.0;
    }

    int Param_Pub_Frequency;
    if (!nh.getParam("Frequency", Param_Pub_Frequency)) {
        Param_Pub_Frequency = 10;
    }
    ros::Rate Pub_Frequency(Param_Pub_Frequency);

    // ------------------------------------------------------------
    // Ros Loop
    // ------------------------------------------------------------

    Data.data.push_back(0.0);
    Data.data.push_back(0.0);

    while (nh.ok()) {
        DistanceToWall_pub.publish(Data);

        ROS_INFO("%lf %lf", Data.data[0], Data.data[1]);

        ros::spinOnce();
        Pub_Frequency.sleep();
    }

    return 0;
}