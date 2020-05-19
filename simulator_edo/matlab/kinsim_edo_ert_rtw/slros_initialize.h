#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block kinsim_edo/Subscribe
extern SimulinkSubscriber<trajectory_msgs::JointTrajectoryPoint, SL_Bus_kinsim_edo_trajectory_msgs_JointTrajectoryPoint> Sub_kinsim_edo_16;

// For Block kinsim_edo/Publish
extern SimulinkPublisher<sensor_msgs::JointState, SL_Bus_kinsim_edo_sensor_msgs_JointState> Pub_kinsim_edo_22;

// For Block kinsim_edo/Publish1
extern SimulinkPublisher<rosgraph_msgs::Clock, SL_Bus_kinsim_edo_rosgraph_msgs_Clock> Pub_kinsim_edo_50;

// For Block kinsim_edo/Publish2
extern SimulinkPublisher<sensor_msgs::JointState, SL_Bus_kinsim_edo_sensor_msgs_JointState> Pub_kinsim_edo_79;

// For Block kinsim_edo/Subsystem/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_edo_61;

// For Block kinsim_edo/Subsystem/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_edo_65;

// For Block kinsim_edo/Subsystem/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_kinsim_edo_84;

void slros_node_init(int argc, char** argv);

#endif
