#include "controller_2link_planar/planner_joint.h"

#include <trajectory_msgs/JointTrajectoryPoint.h>

void planner_joint::Prepare(void)
{
 RunPeriod = RUN_PERIOD_DEFAULT;

 /* Retrieve parameters from ROS parameter server */
 std::string FullParamName;

 // run_period
 FullParamName = ros::this_node::getName()+"/run_period";

 if (false == Handle.getParam(FullParamName, RunPeriod))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // start_delay
 FullParamName = ros::this_node::getName()+"/start_delay";

 if (false == Handle.getParam(FullParamName, start_delay))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // max_ang_acc
 FullParamName = ros::this_node::getName()+"/max_ang_acc";

 if (false == Handle.getParam(FullParamName, max_acc))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // max_ang_vel
 FullParamName = ros::this_node::getName()+"/max_ang_vel";

 if (false == Handle.getParam(FullParamName, max_vel))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // qi
 FullParamName = ros::this_node::getName()+"/qi";

 if (false == Handle.getParam(FullParamName, qi))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
 else
  num_joint = (unsigned int)qi.size();

 // qf
 FullParamName = ros::this_node::getName()+"/qf";

 if (false == Handle.getParam(FullParamName, qf))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // tf
 FullParamName = ros::this_node::getName()+"/tf";

 if (false == Handle.getParam(FullParamName, tf))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // trajectoryType
 FullParamName = ros::this_node::getName()+"/trajectoryType";

 int type;
 if (false == Handle.getParam(FullParamName, type))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
 else
 {
   switch (type)
   {
     case 1:
      trajectoryType = TRAPEZOIDAL;

      joint_planner_polynomial = NULL;
      joint_planner_trapezoidal = NULL;
      joint_planner_trapezoidal = new joint_planning_trapezoidal();
      
      joint_planner_trapezoidal->init(qi.data(), qf.data(), max_vel.data(), max_acc.data(), num_joint);
      break;

     case 2:
      trajectoryType = POLYNOMIAL;

      joint_planner_trapezoidal = NULL;
      joint_planner_polynomial = NULL;
      joint_planner_polynomial = new joint_planning_polynomial();
      
      joint_planner_polynomial->init(qi.data(), qf.data(), tf, num_joint);
      break;

     default:
      trajectoryType = TRAPEZOIDAL;

      joint_planner_polynomial = NULL;
      joint_planner_trapezoidal = NULL;
      joint_planner_trapezoidal = new joint_planning_trapezoidal();
      
      joint_planner_trapezoidal->init(qi.data(), qf.data(), max_vel.data(), max_acc.data(), num_joint);
      break;
   }
 }

 /* ROS topics */
 jointTrajectory_publisher = Handle.advertise<trajectory_msgs::JointTrajectoryPoint>("/joint_trajectory", 1);

 /* Initialize node state */
 time_from_start = ros::Duration(0.0);
}

void planner_joint::RunPeriodically(float Period)
{
 ros::Rate LoopRate(1.0/Period);

 ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

 while (ros::ok())
 {
  PeriodicTask();

  ros::spinOnce();

  LoopRate.sleep();
 }
}

void planner_joint::Shutdown(void)
{
 if (joint_planner_polynomial)
 {
   delete joint_planner_polynomial;
 }

 ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void planner_joint::PeriodicTask(void)
{
  /* Time update */
  if (ros::Time::now().toSec() <= start_delay) {
    time_from_start = ros::Duration(0.0);
  } else {
    time_from_start += ros::Duration(RunPeriod);
  }

  /* Compute a new joint trajectory point */
  double q[num_joint], v[num_joint], a[num_joint];
  switch (trajectoryType){
    case POLYNOMIAL:
      joint_planner_polynomial->plan(time_from_start.toSec());
      
      joint_planner_polynomial->getJointPosition(q);
      joint_planner_polynomial->getJointVelocity(v);
      joint_planner_polynomial->getJointAcceleration(a);
      break;

    case TRAPEZOIDAL:
      joint_planner_trapezoidal->plan(time_from_start.toSec());

      joint_planner_trapezoidal->getJointPosition(q);
      joint_planner_trapezoidal->getJointVelocity(v);
      joint_planner_trapezoidal->getJointAcceleration(a);
      break;
  }

  /* Publish a joint trajectory point */
  trajectory_msgs::JointTrajectoryPoint jointTrajectory;
  jointTrajectory.time_from_start = time_from_start;

  jointTrajectory.positions.clear();
  jointTrajectory.velocities.clear();
  jointTrajectory.accelerations.clear();
  for (int i=0; i<num_joint; i++)
  {
    jointTrajectory.positions.push_back(q[i]);
    jointTrajectory.velocities.push_back(v[i]);
    jointTrajectory.accelerations.push_back(a[i]);
  }

  jointTrajectory_publisher.publish(jointTrajectory);
}
