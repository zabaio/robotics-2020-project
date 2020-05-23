#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "dynsim_edo";

// For Block dynsim_edo/Subscribe
SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_dynsim_edo_std_msgs_Float64MultiArray> Sub_dynsim_edo_16;

// For Block dynsim_edo/Publish
SimulinkPublisher<sensor_msgs::JointState, SL_Bus_dynsim_edo_sensor_msgs_JointState> Pub_dynsim_edo_22;

// For Block dynsim_edo/Publish1
SimulinkPublisher<rosgraph_msgs::Clock, SL_Bus_dynsim_edo_rosgraph_msgs_Clock> Pub_dynsim_edo_50;

// For Block dynsim_edo/Publish2
SimulinkPublisher<sensor_msgs::JointState, SL_Bus_dynsim_edo_sensor_msgs_JointState> Pub_dynsim_edo_124;

// For Block dynsim_edo/edo robot dynamic model/Subsystem/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_112;

// For Block dynsim_edo/edo robot dynamic model/Subsystem/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_113;

// For Block dynsim_edo/edo robot dynamic model/Subsystem/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_117;

// For Block dynsim_edo/edo robot dynamic model/Subsystem/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_118;

// For Block dynsim_edo/edo robot dynamic model/Subsystem/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_125;

// For Block dynsim_edo/edo robot dynamic model/Subsystem/Get Parameter8
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_133;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

