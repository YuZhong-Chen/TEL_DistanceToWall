#include "distance_to_wall/DockingClass.h"

#include <cmath>

DOCKING Docking;

DOCKING::DOCKING() {
}

void DOCKING::Init(ros::NodeHandle* nh) {
    // ------------------------------------------------------------
    // Get Parameter from roslaunch.
    // ------------------------------------------------------------
    nh->param<bool>("DebugMode", DebugMode, false);

    nh->param<double>("Left_a", Left_a, 1.0);
    nh->param<double>("Left_b", Left_b, 0.0);
    nh->param<double>("Right_a", Right_a, 1.0);
    nh->param<double>("Right_b", Right_b, 0.0);

    nh->param<double>("TargetDistanceError", TargetDistanceError, 1.0);

    nh->param<double>("Left_Standard", Left_Standard, 100.0);
    nh->param<double>("Right_Standard", Right_Standard, 100.0);
    nh->param<double>("CalibrationRange", CalibrationRange, 10.0);

    nh->param<double>("CarVelocity", CarVelocity, 0.05);

    nh->param<double>("VL53_Data_TimeOut", VL53_Data_TimeOut, 2.0);

    // ------------------------------------------------------------
    // Setup Publisher, Subscriber and Service.
    // ------------------------------------------------------------
    VL53_sub = nh->subscribe("/CarDistanceFront", 1, CallBack_VL53);
    CarVel_pub = nh->advertise<geometry_msgs::Twist>("/push_vel", 1);
    DockingStart_Server = nh->advertiseService("/DockingStart", DOCKING::CallBack_Server);
    DockingFinish_Client = nh->serviceClient<distance_to_wall::DockingFinish>("/DockingFinish");

    // ------------------------------------------------------------
    // Setup others.
    // ------------------------------------------------------------
    VL53_Data.data.push_back(0.0);
    VL53_Data.data.push_back(0.0);
    VL53_Data_LastTime = ros::Time::now().toSec();
}

// 0 -> left
// 1 -> right
void DOCKING::UpdateData_VL53(const std_msgs::Int16MultiArray::ConstPtr& msg) {
    VL53_Data_LastTime = ros::Time::now().toSec();

    VL53_Data.data[0] = (double)msg->data[0] * Left_a + Left_b;
    VL53_Data.data[1] = (double)msg->data[1] * Right_a + Right_b;

    VL53_Data_Average = (VL53_Data.data[0] + VL53_Data.data[1]) / 2.0;
    ROS_INFO("%.1lf %.1lf %.1lf", VL53_Data.data[0], VL53_Data.data[1], VL53_Data_Average);
}

void DOCKING::CallBack_VL53(const std_msgs::Int16MultiArray::ConstPtr& msg) {
    Docking.UpdateData_VL53(msg);
}

bool DOCKING::CallBack_Server(distance_to_wall::DockingStart::Request& req, distance_to_wall::DockingStart::Response& res) {
    Docking.StartDocking(req.Distance);
    return true;
}

void DOCKING::StartDocking(double TargetDistance) {
    bool isSuccess = false;

    geometry_msgs::Twist msg;
    msg.linear.y = 0.0;
    msg.angular.z = 0.0;

    int DataTimeOut_count = 0;

    ros::Rate CheckFrequency(10);
    while (true) {
        if (fabs(ros::Time::now().toSec() - VL53_Data_LastTime) > VL53_Data_TimeOut) {
            msg.linear.x = 0.0;
            CarVel_pub.publish(msg);

            if (++DataTimeOut_count >= 10) {
                isSuccess = false;
                break;
            }
        } else if (fabs(TargetDistance - VL53_Data_Average) > TargetDistanceError) {
            if (VL53_Data_Average < TargetDistance) {
                msg.linear.x = (-1) * CarVelocity;
                CarVel_pub.publish(msg);
            } else {
                msg.linear.x = CarVelocity;
                CarVel_pub.publish(msg);
            }

            DataTimeOut_count = 0;
        } else {
            isSuccess = true;
            msg.linear.x = 0.0;
            for (int i = 0; i < 3; i++) {
                CarVel_pub.publish(msg);
            }
            break;
        }

        ros::spinOnce();
        CheckFrequency.sleep();
    }
    FinishDocking(isSuccess);
}

void DOCKING::FinishDocking(bool isSuccess) {
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