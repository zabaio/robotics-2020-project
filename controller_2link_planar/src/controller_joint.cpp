#include "controller_2link_planar/controller_joint.h"

#include "std_msgs/Float64MultiArray.h"


void controller_joint::Prepare(void)
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

 // Kp_pos
 FullParamName = ros::this_node::getName()+"/Kp_pos";

 if (false == Handle.getParam(FullParamName, Kp_pos))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
 else
  num_joint = (unsigned int)Kp_pos.size();

 // uMin_pos
 FullParamName = ros::this_node::getName()+"/uMin_pos";

 if (false == Handle.getParam(FullParamName, uMin_pos))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // uMax_pos
 FullParamName = ros::this_node::getName()+"/uMax_pos";

 if (false == Handle.getParam(FullParamName, uMax_pos))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // Kp_vel
 FullParamName = ros::this_node::getName()+"/Kp_vel";

 if (false == Handle.getParam(FullParamName, Kp_vel))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // Ti_vel
 FullParamName = ros::this_node::getName()+"/Ti_vel";

 if (false == Handle.getParam(FullParamName, Ti_vel))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // uMin
 FullParamName = ros::this_node::getName()+"/uMin_vel";

 if (false == Handle.getParam(FullParamName, uMin_vel))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // uMax
 FullParamName = ros::this_node::getName()+"/uMax_vel";

 if (false == Handle.getParam(FullParamName, uMax_vel))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 /* ROS topics */
 jointTorque_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/joint_torque", 1);

 plannedTrajectory_subscriber = Handle.subscribe("/joint_trajectory", 1, &controller_joint::plannedTrajectory_MessageCallback, this);
 jointStates_subscriber = Handle.subscribe("/joint_states", 1, &controller_joint::jointStates_MessageCallback, this);

 /* Initialize node state */
 for (int i=0; i<num_joint; i++){
   // Create controllers
   P_pos.push_back(new PIDcontrol(Kp_pos.at(i), RunPeriod, uMin_pos.at(i), uMax_pos.at(i)));
   PI_vel.push_back(new PIDcontrol(Kp_vel.at(i), Ti_vel.at(i), RunPeriod, uMin_vel.at(i), uMax_vel.at(i)));

   // Initialize reference and measurement inputs
   joint_pos.push_back(0.0);
   joint_vel.push_back(0.0);
   joint_torque.push_back(0.0);
   plan_pos.push_back(0.0);
   plan_vel.push_back(0.0);
   plan_acc.push_back(0.0);
 }
}

void controller_joint::plannedTrajectory_MessageCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg)
{
  for (int i=0; i<num_joint; i++){
    plan_pos.at(i) = msg->positions.at(i);
    plan_vel.at(i) = msg->velocities.at(i);
    if (msg->accelerations.size()>0){
      plan_acc.at(i) = msg->accelerations.at(i);
    }
  }
}

void controller_joint::jointStates_MessageCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  for (int i=0; i<num_joint; i++){
    joint_pos.at(i) = msg->position.at(i);
    joint_vel.at(i) = msg->velocity.at(i);
  }
}

void controller_joint::RunPeriodically(float Period)
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

void controller_joint::Shutdown(void)
{
 ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void controller_joint::PeriodicTask(void)
{
  /* Time update */
  if (ros::Time::now().toSec() <= start_delay) {
    time_from_start = ros::Duration(0.0);
  } else {
    time_from_start += ros::Duration(RunPeriod);

    /* Execute P and PI controllers */
    for (int i = 0; i < num_joint; i++) {
      double v_des;

      // Position control
      P_pos.at(i)->execute(joint_pos.at(i), plan_pos.at(i), v_des);

      // Velocity control
      PI_vel.at(i)->execute(joint_vel.at(i), v_des, joint_torque.at(i));
    }
  }

  /* Publish a joint trajectory point */
  std_msgs::Float64MultiArray jointTorque;

  jointTorque.data.clear();
  jointTorque.data.push_back(time_from_start.toSec()); // The first element of the vector is the actual time
  for (int i=0; i<num_joint; i++)
  {
    jointTorque.data.push_back(joint_torque.at(i));
  }

  jointTorque_publisher.publish(jointTorque);
}
