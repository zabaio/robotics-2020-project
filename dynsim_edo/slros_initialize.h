#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block dynsim_edo/Subscribe
extern SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_dynsim_edo_std_msgs_Float64MultiArray> Sub_dynsim_edo_16;

// For Block dynsim_edo/Publish
extern SimulinkPublisher<sensor_msgs::JointState, SL_Bus_dynsim_edo_sensor_msgs_JointState> Pub_dynsim_edo_22;

// For Block dynsim_edo/Publish1
extern SimulinkPublisher<rosgraph_msgs::Clock, SL_Bus_dynsim_edo_rosgraph_msgs_Clock> Pub_dynsim_edo_50;

// For Block dynsim_edo/Publish2
extern SimulinkPublisher<sensor_msgs::JointState, SL_Bus_dynsim_edo_sensor_msgs_JointState> Pub_dynsim_edo_124;

// For Block dynsim_edo/edo robot dynamic model/Subsystem/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_112;

// For Block dynsim_edo/edo robot dynamic model/Subsystem/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_113;

// For Block dynsim_edo/edo robot dynamic model/Subsystem/Get Parameter10
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_135;

// For Block dynsim_edo/edo robot dynamic model/Subsystem/Get Parameter11
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_136;

// For Block dynsim_edo/edo robot dynamic model/Subsystem/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_117;

// For Block dynsim_edo/edo robot dynamic model/Subsystem/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_118;

// For Block dynsim_edo/edo robot dynamic model/Subsystem/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_125;

// For Block dynsim_edo/edo robot dynamic model/Subsystem/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_126;

// For Block dynsim_edo/edo robot dynamic model/Subsystem/Get Parameter6
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_127;

// For Block dynsim_edo/edo robot dynamic model/Subsystem/Get Parameter7
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_128;

// For Block dynsim_edo/edo robot dynamic model/Subsystem/Get Parameter8
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_133;

// For Block dynsim_edo/edo robot dynamic model/Subsystem/Get Parameter9
extern SimulinkParameterGetter<real64_T, double> ParamGet_dynsim_edo_134;

void slros_node_init(int argc, char** argv);

#endif
