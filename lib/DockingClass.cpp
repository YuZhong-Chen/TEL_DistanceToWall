#include "distance_to_wall/DockingClass.h"

DOCKING Docking;

DOCKING::DOCKING() {
}

void DOCKING::Init(ros::NodeHandle* nh, double Param_Left_a, double Param_Left_b, double Param_Right_a, double Param_Right_b, double Param_CarVelocity) {
    VL53_sub = nh->subscribe("/CarDistanceFront", 1, CallBack_VL53);
    CarVel_pub = nh->advertise<geometry_msgs::Twist>("push_vel", 1);
    Server = nh->advertiseService("/DockingService", DOCKING::CallBack_Server);

    VL53_Data.data.push_back(0.0);
    VL53_Data.data.push_back(0.0);

    this->Param_Left_a = Param_Left_a;
    this->Param_Left_b = Param_Left_b;
    this->Param_Right_a = Param_Right_a;
    this->Param_Right_b = Param_Right_b;
    this->CarVelocity = Param_CarVelocity;
}

// 0 -> left
// 1 -> right
void DOCKING::UpdateData_VL53(const std_msgs::Int16MultiArray::ConstPtr& msg) {
    VL53_Data.data[0] = (double)msg->data[0] * Param_Left_a + Param_Left_b;
    VL53_Data.data[1] = (double)msg->data[1] * Param_Right_a + Param_Right_b;

    // ROS_INFO("%.1lf %.1lf", VL53_Data.data[0], VL53_Data.data[1]);
}

void DOCKING::CallBack_VL53(const std_msgs::Int16MultiArray::ConstPtr& msg) {
    Docking.UpdateData_VL53(msg);
}

bool DOCKING::CallBack_Server(distance_to_wall::Docking::Request& req, distance_to_wall::Docking::Response& res) {
    Docking.StartDocking(req.Distance);
    return true;
}

void DOCKING::StartDocking(double TargetDistance) {
    ROS_INFO("test : %d", TargetDistance);

    geometry_msgs::Twist msg;

    msg.linear.x = CarVelocity;
    msg.linear.y = 0.0;
    msg.angular.z = 0.0;

    CarVel_pub.publish(msg);

    while (true) {
        // TODO
    }

    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.angular.z = 0.0;

    CarVel_pub.publish(msg);

    // Remember to call service.
}