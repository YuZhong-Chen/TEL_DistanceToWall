#ifndef _DOCKING_H_
#define _DOCKING_H_

#include "distance_to_wall/Docking.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"

class DOCKING {
   public:
    DOCKING();

    void Init(ros::NodeHandle* nh, double Param_Left_a, double Param_Left_b, double Param_Right_a, double Param_Right_b, double Param_CarVelocity);

    static void CallBack_VL53(const std_msgs::Int16MultiArray::ConstPtr& msg);
    static bool CallBack_Server(distance_to_wall::Docking::Request& req, distance_to_wall::Docking::Response& res);

    void StartDocking(double TargetDistance);

   private:
    ros::Subscriber VL53_sub;
    ros::Publisher CarVel_pub;
    ros::ServiceServer Server;

    std_msgs::Float64MultiArray VL53_Data;
    void UpdateData_VL53(const std_msgs::Int16MultiArray::ConstPtr& msg);

    double Param_Left_a;
    double Param_Left_b;
    double Param_Right_a;
    double Param_Right_b;

    double CarVelocity;
    double CurrentCarVelocity;
};

extern DOCKING Docking;

#endif