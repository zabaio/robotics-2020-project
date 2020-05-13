#ifndef CONTROLLER_JOINT_H_
#define CONTROLLER_JOINT_H_

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */
 
#define NAME_OF_THIS_NODE "controller_joint"

#include "PIDcontrol.h"

 
class controller_joint
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Publisher jointTorque_publisher;
    ros::Subscriber plannedTrajectory_subscriber, jointStates_subscriber;
    
    /* ROS topic callbacks */
    void jointStates_MessageCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void plannedTrajectory_MessageCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg);
    
    /* Parameters from ROS parameter server */
    int num_joint;
    double start_delay;
    std::vector<double> Kp_pos, Kp_vel, Ti_vel, uMin_pos, uMax_pos, uMin_vel, uMax_vel;

    /* Joint controller periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    ros::Duration time_from_start;

    std::vector<PIDcontrol*> PI_vel, P_pos;
    std::vector<double> joint_pos, joint_vel, joint_torque, plan_pos, plan_vel, plan_acc;
    
  public:
    double RunPeriod;
    
    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);
};

#endif /* PLANNER_JOINT_H_ */
