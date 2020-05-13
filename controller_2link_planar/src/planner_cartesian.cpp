#include "controller_2link_planar/planner_cartesian.h"

#include <trajectory_msgs/JointTrajectoryPoint.h>

void planner_cartesian::Prepare(void)
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

 // max_acc
 FullParamName = ros::this_node::getName()+"/max_acc";

 if (false == Handle.getParam(FullParamName, max_acc))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // max_vel
 FullParamName = ros::this_node::getName()+"/max_vel";

 if (false == Handle.getParam(FullParamName, max_vel))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // pi
 FullParamName = ros::this_node::getName()+"/pi";

 if (false == Handle.getParam(FullParamName, pi))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
 else
  num_coordinate = (unsigned int)pi.size();

 // pf
 FullParamName = ros::this_node::getName()+"/pf";

 if (false == Handle.getParam(FullParamName, pf))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // elbow_high
 FullParamName = ros::this_node::getName()+"/elbow_high";

 if (false == Handle.getParam(FullParamName, elbow_high))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // link_length
 FullParamName = ros::this_node::getName()+"/link_length";

 if (false == Handle.getParam(FullParamName, link_length))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 /* ROS topics */
 jointTrajectory_publisher = Handle.advertise<trajectory_msgs::JointTrajectoryPoint>("/joint_trajectory", 1);
 cartesianTrajectory_publisher = Handle.advertise<trajectory_msgs::JointTrajectoryPoint>("/cartesian_trajectory", 1);

 /* Initialize node state */
 time_from_start = ros::Duration(0.0);

 cartesian_planner = NULL;
 cartesian_planner = new cartesian_planning();
      
 cartesian_planner->init_line(pi.data(), pf.data(), max_vel, max_acc, num_coordinate);
}

void planner_cartesian::RunPeriodically(float Period)
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

void planner_cartesian::Shutdown(void)
{
 if (cartesian_planner)
 {
   delete cartesian_planner;
 }

 ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void planner_cartesian::PeriodicTask(void)
{
  /* Time update */
  if (ros::Time::now().toSec() <= start_delay) {
    time_from_start = ros::Duration(0.0);
  } else {
    time_from_start += ros::Duration(RunPeriod);
  }

  /* Compute a new cartesian trajectory point */
  double p[num_coordinate], pv[num_coordinate], pa[num_coordinate];  
  cartesian_planner->plan(time_from_start.toSec());
  cartesian_planner->getCartesianPosition(p);
  cartesian_planner->getCartesianVelocity(pv);
  cartesian_planner->getCartesianAcceleration(pa);

  /* Convert a cartesian into a joint trajectory */
  double q[num_coordinate], qv[num_coordinate];
  inverse_kinematics(p, q, elbow_high);
  inverse_diffkinematics(pv, q, qv);

  /* Publish a joint trajectory point */
  trajectory_msgs::JointTrajectoryPoint jointTrajectory;
  jointTrajectory.time_from_start = time_from_start;

  jointTrajectory.positions.clear();
  jointTrajectory.velocities.clear();
  jointTrajectory.accelerations.clear();
  for (int i=0; i<num_coordinate; i++)
  {
    jointTrajectory.positions.push_back(q[i]);
    jointTrajectory.velocities.push_back(qv[i]);
  }

  jointTrajectory_publisher.publish(jointTrajectory);

  /* Publish a cartesian trajectory point */
  trajectory_msgs::JointTrajectoryPoint cartesianTrajectory;
  cartesianTrajectory.time_from_start = time_from_start;

  cartesianTrajectory.positions.clear();
  cartesianTrajectory.velocities.clear();
  cartesianTrajectory.accelerations.clear();
  for (int i=0; i<num_coordinate; i++)
  {
    cartesianTrajectory.positions.push_back(p[i]);
    cartesianTrajectory.velocities.push_back(pv[i]);
    cartesianTrajectory.accelerations.push_back(pa[i]);
  }

  cartesianTrajectory_publisher.publish(cartesianTrajectory);
}

void planner_cartesian::inverse_kinematics(double p[], double q[], bool elbow_high){
    double c2 = (pow(p[0], 2.0) + pow(p[1], 2.0) - pow(link_length.at(0), 2.0) - pow(link_length.at(1), 2.0)) /
                (2.0 * link_length.at(0) * link_length.at(1));

    double s2;
    if (elbow_high){
      s2 = pow(1.0 - pow(c2, 2.0), 0.5);
    }
    else{
      s2 = -pow(1.0 - pow(c2, 2.0), 0.5);
    }

    double c1 = ((link_length.at(0) + link_length.at(1) * c2) * p[0] + link_length.at(1) * s2 * p[1]) / (pow(p[0], 2.0) + pow(p[1], 2.0));
    double s1 = ((link_length.at(0) + link_length.at(1) * c2) * p[1] - link_length.at(1) * s2 * p[0]) / (pow(p[0], 2.0) + pow(p[1], 2.0));

    q[0] = atan2(s1, c1);
    q[1] = atan2(s2, c2);
}

void planner_cartesian::inverse_diffkinematics(double pv[], double q[], double qv[]){
  double s1 = sin(q[0]);
  double c1 = cos(q[0]);
  double s12 = sin(q[0]+q[1]); 
  double c12 = cos(q[0]+q[1]);

  double detJ = link_length.at(0) * link_length.at(1) * s1;
  if (fabs(detJ)<=1e-3){
    detJ = 1e-3;
  }

  qv[0] = ((link_length.at(1) * c12) * pv[0] + (link_length.at(1) * s12) * pv[1]) / detJ;
  qv[1] = -((link_length.at(0) * c1 + link_length.at(1) * c12)* pv[0] + (link_length.at(0) * s1 + link_length.at(1) * s12) * pv[1]) / detJ;
}
