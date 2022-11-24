#ifndef _DOCKING_H_
#define _DOCKING_H_

#include "distance_to_wall/DockingFinish.h"
#include "distance_to_wall/DockingStart.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"

class DOCKING {
   public:
    DOCKING();

    void Init(ros::NodeHandle* nh);

    static void CallBack_VL53(const std_msgs::Int16MultiArray::ConstPtr& msg);
    static bool CallBack_Server(distance_to_wall::DockingStart::Request& req, distance_to_wall::DockingStart::Response& res);

    void StartDocking(double TargetDistance);
    void FinishDocking(bool isSuccess);

   private:
    ros::Subscriber VL53_sub;
    ros::Publisher CarVel_pub;
    ros::ServiceServer DockingStart_Server;
    ros::ServiceClient DockingFinish_Client;

    bool DebugMode;

    // TODO : Remember to add TimeOut for Arduino. If it doesn't work, return false.
    std_msgs::Float64MultiArray VL53_Data;
    void UpdateData_VL53(const std_msgs::Int16MultiArray::ConstPtr& msg);

    double VL53_Data_LastTime;
    bool isDataAvailable_VL53;
    double VL53_Data_TimeOut;

    float VL53_Data_Average;

    double Left_a;
    double Left_b;
    double Right_a;
    double Right_b;

    // For Calibrate Omega. Not used yet.
    double Left_Standard;
    double Right_Standard;
    double CalibrationRange;

    double TargetDistanceError;

    double CarVelocity;
    double CurrentCarVelocity;
};

extern DOCKING Docking;

#endif