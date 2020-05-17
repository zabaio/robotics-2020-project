#ifndef PLANNER_CARTESIAN_H_
#define PLANNER_CARTESIAN_H_

#include "ros/ros.h"
#include "cartesian_planning.h"

#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */
 
#define NAME_OF_THIS_NODE "planner_cartesian"

 
class planner_cartesian
{
  private: 
    ros::NodeHandle Handle;
    
    /* ROS topics */
    ros::Publisher jointTrajectory_publisher, cartesianTrajectory_publisher;

    /* Parameters from ROS parameter server */
    bool elbow_high;
    int num_coordinate;
    double start_delay;
    double max_acc, max_vel;

    std::vector<double> pi, pf;
    std::vector<double> link_length;

    /* Cartesian planner periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    ros::Duration time_from_start;

    cartesian_planning* cartesian_planner;

    /* Node private functions */
		void inverse_kinematics(double p[], double q[], bool elbow_high);
    void inverse_diffkinematics(double pv[], double q[], double qv[]);

  public:
    double RunPeriod;
    
    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);
};

#endif /* PLANNER_CARTESIAN_H_ */
