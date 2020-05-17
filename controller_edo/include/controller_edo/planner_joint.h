#ifndef PLANNER_JOINT_H_
#define PLANNER_JOINT_H_

#include "ros/ros.h"
#include "joint_planning_polynomial.h"
#include "joint_planning_trapezoidal.h"

#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */
 
#define NAME_OF_THIS_NODE "planner_joint"

 
class planner_joint
{
  private: 
    ros::NodeHandle Handle;

		enum TrajectoryType { TRAPEZOIDAL, POLYNOMIAL };
    
    /* ROS topics */
    ros::Publisher jointTrajectory_publisher;

    /* Parameters from ROS parameter server */
    double tf;
    int num_joint;
    double start_delay;

    std::vector<double> qi, qf;
    std::vector<double> max_acc, max_vel;

    TrajectoryType trajectoryType;

    /* Joint planner periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    ros::Duration time_from_start;

    joint_planning_polynomial* joint_planner_polynomial;
    joint_planning_trapezoidal* joint_planner_trapezoidal;
    
  public:
    double RunPeriod;
    
    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);
};

#endif /* PLANNER_JOINT_H_ */
