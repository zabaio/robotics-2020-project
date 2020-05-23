#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block dynsim_edo_drag_gravity/Subscribe
extern SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_dynsim_edo_drag_gravity_std_msgs_Float64MultiArray> Sub_dynsim_edo_drag_gravity_16;

// For Block dynsim_edo_drag_gravity/Publish
extern SimulinkPublisher<sensor_msgs::JointState, SL_Bus_dynsim_edo_drag_gravity_sensor_msgs_JointState> Pub_dynsim_edo_drag_gravity_22;

// For Block dynsim_edo_drag_gravity/Publish1
extern SimulinkPublisher<rosgraph_msgs::Clock, SL_Bus_dynsim_edo_drag_gravity_rosgraph_msgs_Clock> Pub_dynsim_edo_drag_gravity_50;

// For Block dynsim_edo_drag_gravity/Publish2
extern SimulinkPublisher<sensor_msgs::JointState, SL_Bus_dynsim_edo_drag_gravity_sensor_msgs_JointState> Pub_dynsim_edo_drag_gravity_124;

// For Block dynsim_edo_drag_gravity/edo robot dynamic model/Subsystem/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_drag_gravity_112;

// For Block dynsim_edo_drag_gravity/edo robot dynamic model/Subsystem/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_drag_gravity_113;

// For Block dynsim_edo_drag_gravity/edo robot dynamic model/Subsystem/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_drag_gravity_117;

// For Block dynsim_edo_drag_gravity/edo robot dynamic model/Subsystem/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_drag_gravity_118;

// For Block dynsim_edo_drag_gravity/edo robot dynamic model/Subsystem/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_drag_gravity_125;

// For Block dynsim_edo_drag_gravity/edo robot dynamic model/Subsystem/Get Parameter8
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_drag_gravity_133;

void slros_node_init(int argc, char** argv);

#endif
