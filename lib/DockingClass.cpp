#include "distance_to_wall/DockingClass.h"

#include <cmath>

DOCKING Docking;

DOCKING::DOCKING() {
}

void DOCKING::Init(ros::NodeHandle* nh_Public, ros::NodeHandle* nh_Private) {
    // ------------------------------------------------------------
    // Get Parameter from roslaunch.
    // ------------------------------------------------------------
    nh_Private->param<bool>("DebugMode", DebugMode, false);

    nh_Private->param<double>("Left_a", Left_a, 1.0);
    nh_Private->param<double>("Left_b", Left_b, 0.0);
    nh_Private->param<double>("Right_a", Right_a, 1.0);
    nh_Private->param<double>("Right_b", Right_b, 0.0);

    nh_Private->param<double>("TargetDistanceError", TargetDistanceError, 1.0);

    nh_Private->param<double>("Left_Standard", Left_Standard, 100.0);
    nh_Private->param<double>("Right_Standard", Right_Standard, 100.0);
    nh_Private->param<double>("CalibrationRange", CalibrationRange, 10.0);

    nh_Private->param<double>("CarVelocity", CarVelocity, 0.05);

    nh_Private->param<double>("VL53_Data_TimeOut", VL53_Data_TimeOut, 2.0);

    // ------------------------------------------------------------
    // Setup Publisher, Subscriber and Service.
    // ------------------------------------------------------------
    VL53_sub = nh_Public->subscribe("CarDistanceFront", 1, CallBack_VL53);
    CarVel_pub = nh_Public->advertise<geometry_msgs::Twist>("push_vel", 1);
    DockingStart_Server = nh_Public->advertiseService("DockingStart", DOCKING::CallBack_Server);
    DockingFinish_Client = nh_Public->serviceClient<distance_to_wall::DockingFinish>("DockingFinish");

    // ------------------------------------------------------------
    // Setup others.
    // ------------------------------------------------------------
    VL53_Data.data.push_back(0.0);
    VL53_Data.data.push_back(0.0);
    VL53_Data_LastTime = ros::Time::now().toSec();

    isDocking = false;
}

// 0 -> left
// 1 -> right
void DOCKING::UpdateData_VL53(const std_msgs::Int16MultiArray::ConstPtr& msg) {
    VL53_Data_LastTime = ros::Time::now().toSec();

    VL53_Data.data[0] = (double)msg->data[0] * Left_a + Left_b;
    VL53_Data.data[1] = (double)msg->data[1] * Right_a + Right_b;

    VL53_Data_Average = (VL53_Data.data[0] + VL53_Data.data[1]) / 2.0;

    if (DebugMode) {
        ROS_INFO("Left : %.1lf Right : %.1lf Avg : %.1lf", VL53_Data.data[0], VL53_Data.data[1], VL53_Data_Average);
    }
}

void DOCKING::CallBack_VL53(const std_msgs::Int16MultiArray::ConstPtr& msg) {
    Docking.UpdateData_VL53(msg);
}

bool DOCKING::CallBack_Server(distance_to_wall::DockingStart::Request& req, distance_to_wall::DockingStart::Response& res) {
    Docking.StartDocking(req.Distance);
    return true;
}

void DOCKING::StartDocking(double TargetDistance) {
    isDocking = true;

    CarVelocity_msg.linear.y = 0.0;
    CarVelocity_msg.angular.z = 0.0;

    this->TargetDistance = TargetDistance;

    VL53_DataTimeOut_count = 0;
}

void DOCKING::UpdateStatus() {
    if (isDocking) {
        if (fabs(ros::Time::now().toSec() - VL53_Data_LastTime) > VL53_Data_TimeOut) {
            CarVelocity_msg.linear.x = 0.0;
            CarVel_pub.publish(CarVelocity_msg);

            if (++VL53_DataTimeOut_count >= 10) {
                FinishDocking(false);
            }
        } else if (fabs(TargetDistance - VL53_Data_Average) > TargetDistanceError) {
            if (VL53_Data_Average < TargetDistance) {
                CarVelocity_msg.linear.x = (-1) * CarVelocity;
                CarVel_pub.publish(CarVelocity_msg);
            } else {
                CarVelocity_msg.linear.x = CarVelocity;
                CarVel_pub.publish(CarVelocity_msg);
            }

            VL53_DataTimeOut_count = 0;
        } else {
            CarVelocity_msg.linear.x = 0.0;
            for (int i = 0; i < 3; i++) {
                CarVel_pub.publish(CarVelocity_msg);
            }
            FinishDocking(true);
        }
    }
}

void DOCKING::FinishDocking(bool isSuccess) {
    isDocking = false;

    distance_to_wall::DockingFinish srv;

    srv.request.isFinished = isSuccess;

    if (!isSuccess) {
        ROS_ERROR("DockingClass: Fail to Docking.");
    }

    for (int i = 0; i < 5; i++) {
        if (DockingFinish_Client.call(srv)) {
            break;
        } else {
            ROS_ERROR("DockingClass: Finish's client fail to call server.");
            ros::Duration(0.5).sleep();
        }
    }
}