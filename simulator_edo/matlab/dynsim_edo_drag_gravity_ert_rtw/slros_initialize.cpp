#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "dynsim_edo_drag_gravity";

// For Block dynsim_edo_drag_gravity/Subscribe
SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_dynsim_edo_drag_gravity_std_msgs_Float64MultiArray> Sub_dynsim_edo_drag_gravity_16;

// For Block dynsim_edo_drag_gravity/Publish
SimulinkPublisher<sensor_msgs::JointState, SL_Bus_dynsim_edo_drag_gravity_sensor_msgs_JointState> Pub_dynsim_edo_drag_gravity_22;

// For Block dynsim_edo_drag_gravity/Publish1
SimulinkPublisher<rosgraph_msgs::Clock, SL_Bus_dynsim_edo_drag_gravity_rosgraph_msgs_Clock> Pub_dynsim_edo_drag_gravity_50;

// For Block dynsim_edo_drag_gravity/Publish2
SimulinkPublisher<sensor_msgs::JointState, SL_Bus_dynsim_edo_drag_gravity_sensor_msgs_JointState> Pub_dynsim_edo_drag_gravity_124;

// For Block dynsim_edo_drag_gravity/edo robot dynamic model/Subsystem/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_drag_gravity_112;

// For Block dynsim_edo_drag_gravity/edo robot dynamic model/Subsystem/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_drag_gravity_113;

// For Block dynsim_edo_drag_gravity/edo robot dynamic model/Subsystem/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_drag_gravity_117;

// For Block dynsim_edo_drag_gravity/edo robot dynamic model/Subsystem/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_drag_gravity_118;

// For Block dynsim_edo_drag_gravity/edo robot dynamic model/Subsystem/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_drag_gravity_125;

// For Block dynsim_edo_drag_gravity/edo robot dynamic model/Subsystem/Get Parameter8
SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_drag_gravity_133;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

