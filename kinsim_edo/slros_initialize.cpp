#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "kinsim_edo";

// For Block kinsim_edo/Subscribe
SimulinkSubscriber<trajectory_msgs::JointTrajectoryPoint, SL_Bus_kinsim_edo_trajectory_msgs_JointTrajectoryPoint> Sub_kinsim_edo_16;

// For Block kinsim_edo/Publish
SimulinkPublisher<sensor_msgs::JointState, SL_Bus_kinsim_edo_sensor_msgs_JointState> Pub_kinsim_edo_22;

// For Block kinsim_edo/Publish1
SimulinkPublisher<rosgraph_msgs::Clock, SL_Bus_kinsim_edo_rosgraph_msgs_Clock> Pub_kinsim_edo_50;

// For Block kinsim_edo/Publish2
SimulinkPublisher<sensor_msgs::JointState, SL_Bus_kinsim_edo_sensor_msgs_JointState> Pub_kinsim_edo_79;

// For Block kinsim_edo/Subsystem/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_edo_61;

// For Block kinsim_edo/Subsystem/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_edo_65;

// For Block kinsim_edo/Subsystem/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_edo_81;

// For Block kinsim_edo/Subsystem/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_edo_82;

// For Block kinsim_edo/Subsystem/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_edo_83;

// For Block kinsim_edo/Subsystem/Get Parameter5
SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_edo_84;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

