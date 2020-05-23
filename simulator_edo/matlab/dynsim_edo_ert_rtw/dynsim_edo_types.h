//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: dynsim_edo_types.h
//
// Code generated for Simulink model 'dynsim_edo'.
//
// Model version                  : 1.122
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Sat May 23 15:26:22 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_dynsim_edo_types_h_
#define RTW_HEADER_dynsim_edo_types_h_
#include "rtwtypes.h"

// Model Code Variants
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_edo_std_msgs_String_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_edo_std_msgs_String_

// MsgType=std_msgs/String
typedef struct {
  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=Data_SL_Info:TruncateAction=warn 
  uint8_T Data[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Data
  SL_Bus_ROSVariableLengthArrayInfo Data_SL_Info;
} SL_Bus_dynsim_edo_std_msgs_String;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_edo_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_edo_ros_time_Time_

// MsgType=ros_time/Time
typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_dynsim_edo_ros_time_Time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_edo_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_edo_std_msgs_Header_

// MsgType=std_msgs/Header
typedef struct {
  uint32_T Seq;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=FrameId_SL_Info:TruncateAction=warn 
  uint8_T FrameId[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=FrameId
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;

  // MsgType=ros_time/Time
  SL_Bus_dynsim_edo_ros_time_Time Stamp;
} SL_Bus_dynsim_edo_std_msgs_Header;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_edo_sensor_msgs_JointState_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_edo_sensor_msgs_JointState_

// MsgType=sensor_msgs/JointState
typedef struct {
  // MsgType=std_msgs/String:PrimitiveROSType=string[]:IsVarLen=1:VarLenCategory=data:VarLenElem=Name_SL_Info:TruncateAction=warn 
  SL_Bus_dynsim_edo_std_msgs_String Name[16];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Name
  SL_Bus_ROSVariableLengthArrayInfo Name_SL_Info;

  // IsVarLen=1:VarLenCategory=data:VarLenElem=Position_SL_Info:TruncateAction=warn 
  real_T Position[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Position
  SL_Bus_ROSVariableLengthArrayInfo Position_SL_Info;

  // IsVarLen=1:VarLenCategory=data:VarLenElem=Velocity_SL_Info:TruncateAction=warn 
  real_T Velocity[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Velocity
  SL_Bus_ROSVariableLengthArrayInfo Velocity_SL_Info;

  // IsVarLen=1:VarLenCategory=data:VarLenElem=Effort_SL_Info:TruncateAction=warn 
  real_T Effort[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Effort
  SL_Bus_ROSVariableLengthArrayInfo Effort_SL_Info;

  // MsgType=std_msgs/Header
  SL_Bus_dynsim_edo_std_msgs_Header Header;
} SL_Bus_dynsim_edo_sensor_msgs_JointState;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_edo_rosgraph_msgs_Clock_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_edo_rosgraph_msgs_Clock_

// MsgType=rosgraph_msgs/Clock
typedef struct {
  // MsgType=ros_time/Time
  SL_Bus_dynsim_edo_ros_time_Time Clock_;
} SL_Bus_dynsim_edo_rosgraph_msgs_Clock;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_edo_std_msgs_MultiArrayDimension_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_edo_std_msgs_MultiArrayDimension_

// MsgType=std_msgs/MultiArrayDimension
typedef struct {
  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=Label_SL_Info:TruncateAction=warn 
  uint8_T Label[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Label
  SL_Bus_ROSVariableLengthArrayInfo Label_SL_Info;
  uint32_T Size;
  uint32_T Stride;
} SL_Bus_dynsim_edo_std_msgs_MultiArrayDimension;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_edo_std_msgs_MultiArrayLayout_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_edo_std_msgs_MultiArrayLayout_

// MsgType=std_msgs/MultiArrayLayout
typedef struct {
  uint32_T DataOffset;

  // MsgType=std_msgs/MultiArrayDimension:IsVarLen=1:VarLenCategory=data:VarLenElem=Dim_SL_Info:TruncateAction=warn 
  SL_Bus_dynsim_edo_std_msgs_MultiArrayDimension Dim[16];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Dim
  SL_Bus_ROSVariableLengthArrayInfo Dim_SL_Info;
} SL_Bus_dynsim_edo_std_msgs_MultiArrayLayout;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_edo_std_msgs_Float64MultiArray_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynsim_edo_std_msgs_Float64MultiArray_

// MsgType=std_msgs/Float64MultiArray
typedef struct {
  // IsVarLen=1:VarLenCategory=data:VarLenElem=Data_SL_Info:TruncateAction=warn
  real_T Data[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Data
  SL_Bus_ROSVariableLengthArrayInfo Data_SL_Info;

  // MsgType=std_msgs/MultiArrayLayout
  SL_Bus_dynsim_edo_std_msgs_MultiArrayLayout Layout;
} SL_Bus_dynsim_edo_std_msgs_Float64MultiArray;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_n2RFWdI5YobuyCi5g1HNsC_
#define DEFINED_TYPEDEF_FOR_struct_n2RFWdI5YobuyCi5g1HNsC_

typedef struct {
  real_T NameLength;
  uint8_T Name[9];
  real_T ParentIndex;
  real_T NumChildren;
  real_T ChildrenIndices[6];
  real_T Mass;
  real_T CenterOfMass[3];
  real_T Inertia[9];
  real_T SpatialInertia[36];
} struct_n2RFWdI5YobuyCi5g1HNsC;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_o0IUHl9fQTmfwOrlxOCIeH_
#define DEFINED_TYPEDEF_FOR_struct_o0IUHl9fQTmfwOrlxOCIeH_

typedef struct {
  uint8_T Type;
  real_T NameLength;
  uint8_T Name[7];
  real_T VelocityNumber;
  real_T PositionNumber;
  real_T MotionSubspace[36];
  real_T JointAxis[3];
  real_T PositionLimits[14];
  real_T HomePosition[7];
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
} struct_o0IUHl9fQTmfwOrlxOCIeH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_TwWxrax8fot9L3lU84pxFG_
#define DEFINED_TYPEDEF_FOR_struct_TwWxrax8fot9L3lU84pxFG_

typedef struct {
  real_T NumBodies;
  real_T Gravity[3];
  real_T NumNonFixedBodies;
  real_T PositionNumber;
  real_T VelocityNumber;
  real_T PositionDoFMap[12];
  real_T VelocityDoFMap[12];
  real_T MaxNameLength;
  real_T MaxJointPositionNumber;
  uint8_T DataFormat;
  real_T JointPositionLimits[6];
  struct_n2RFWdI5YobuyCi5g1HNsC Bodies[7];
  struct_o0IUHl9fQTmfwOrlxOCIeH Joints[7];
} struct_TwWxrax8fot9L3lU84pxFG;

#endif

#ifndef struct_tag_rkSooZHJZnr3Dpfu1LNqfH
#define struct_tag_rkSooZHJZnr3Dpfu1LNqfH

struct tag_rkSooZHJZnr3Dpfu1LNqfH
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                 //struct_tag_rkSooZHJZnr3Dpfu1LNqfH

#ifndef typedef_ros_slros_internal_block_Publ_T
#define typedef_ros_slros_internal_block_Publ_T

typedef struct tag_rkSooZHJZnr3Dpfu1LNqfH ros_slros_internal_block_Publ_T;

#endif                                 //typedef_ros_slros_internal_block_Publ_T

#ifndef struct_tag_9SewJ4y3IXNs5GrZti8qkG
#define struct_tag_9SewJ4y3IXNs5GrZti8qkG

struct tag_9SewJ4y3IXNs5GrZti8qkG
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                 //struct_tag_9SewJ4y3IXNs5GrZti8qkG

#ifndef typedef_ros_slros_internal_block_Subs_T
#define typedef_ros_slros_internal_block_Subs_T

typedef struct tag_9SewJ4y3IXNs5GrZti8qkG ros_slros_internal_block_Subs_T;

#endif                                 //typedef_ros_slros_internal_block_Subs_T

#ifndef struct_tag_KSdGoEc2IyOHz4CLi4rcCD
#define struct_tag_KSdGoEc2IyOHz4CLi4rcCD

struct tag_KSdGoEc2IyOHz4CLi4rcCD
{
  int32_T __dummy;
};

#endif                                 //struct_tag_KSdGoEc2IyOHz4CLi4rcCD

#ifndef typedef_e_robotics_slcore_internal_bl_T
#define typedef_e_robotics_slcore_internal_bl_T

typedef struct tag_KSdGoEc2IyOHz4CLi4rcCD e_robotics_slcore_internal_bl_T;

#endif                                 //typedef_e_robotics_slcore_internal_bl_T

#ifndef struct_tag_PzhaB0v2Sx4ikuHWZx5WUB
#define struct_tag_PzhaB0v2Sx4ikuHWZx5WUB

struct tag_PzhaB0v2Sx4ikuHWZx5WUB
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  e_robotics_slcore_internal_bl_T SampleTimeHandler;
};

#endif                                 //struct_tag_PzhaB0v2Sx4ikuHWZx5WUB

#ifndef typedef_ros_slros_internal_block_GetP_T
#define typedef_ros_slros_internal_block_GetP_T

typedef struct tag_PzhaB0v2Sx4ikuHWZx5WUB ros_slros_internal_block_GetP_T;

#endif                                 //typedef_ros_slros_internal_block_GetP_T

#ifndef struct_emxArray_char_T
#define struct_emxArray_char_T

struct emxArray_char_T
{
  char_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_char_T

#ifndef typedef_emxArray_char_T_dynsim_edo_T
#define typedef_emxArray_char_T_dynsim_edo_T

typedef struct emxArray_char_T emxArray_char_T_dynsim_edo_T;

#endif                                 //typedef_emxArray_char_T_dynsim_edo_T

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_real_T

#ifndef typedef_emxArray_real_T_dynsim_edo_T
#define typedef_emxArray_real_T_dynsim_edo_T

typedef struct emxArray_real_T emxArray_real_T_dynsim_edo_T;

#endif                                 //typedef_emxArray_real_T_dynsim_edo_T

// Custom Type definition for MATLAB Function: '<Root>/Assign to JointState msg' 
#ifndef struct_tag_rGJxeAeFOtyQ9icBnCewuB
#define struct_tag_rGJxeAeFOtyQ9icBnCewuB

struct tag_rGJxeAeFOtyQ9icBnCewuB
{
  char_T f1[7];
};

#endif                                 //struct_tag_rGJxeAeFOtyQ9icBnCewuB

#ifndef typedef_cell_wrap_0_dynsim_edo_T
#define typedef_cell_wrap_0_dynsim_edo_T

typedef struct tag_rGJxeAeFOtyQ9icBnCewuB cell_wrap_0_dynsim_edo_T;

#endif                                 //typedef_cell_wrap_0_dynsim_edo_T

#ifndef struct_tag_scvWJBGWVLTwiVc22ZFYQDD
#define struct_tag_scvWJBGWVLTwiVc22ZFYQDD

struct tag_scvWJBGWVLTwiVc22ZFYQDD
{
  real_T NameLength;
  uint8_T Name[9];
  real_T ParentIndex;
  real_T NumChildren;
  real_T ChildrenIndices[6];
  real_T Mass;
  real_T CenterOfMass[3];
  real_T Inertia[9];
  real_T SpatialInertia[36];
};

#endif                                 //struct_tag_scvWJBGWVLTwiVc22ZFYQDD

#ifndef typedef_scvWJBGWVLTwiVc22ZFYQDD_dynsi_T
#define typedef_scvWJBGWVLTwiVc22ZFYQDD_dynsi_T

typedef struct tag_scvWJBGWVLTwiVc22ZFYQDD scvWJBGWVLTwiVc22ZFYQDD_dynsi_T;

#endif                                 //typedef_scvWJBGWVLTwiVc22ZFYQDD_dynsi_T

#ifndef struct_tag_sGoOVDSFGVYEhLxENxeJvlE
#define struct_tag_sGoOVDSFGVYEhLxENxeJvlE

struct tag_sGoOVDSFGVYEhLxENxeJvlE
{
  uint8_T Type;
  real_T NameLength;
  uint8_T Name[7];
  real_T VelocityNumber;
  real_T PositionNumber;
  real_T MotionSubspace[36];
  real_T JointAxis[3];
  real_T PositionLimits[14];
  real_T HomePosition[7];
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
};

#endif                                 //struct_tag_sGoOVDSFGVYEhLxENxeJvlE

#ifndef typedef_sGoOVDSFGVYEhLxENxeJvlE_dynsi_T
#define typedef_sGoOVDSFGVYEhLxENxeJvlE_dynsi_T

typedef struct tag_sGoOVDSFGVYEhLxENxeJvlE sGoOVDSFGVYEhLxENxeJvlE_dynsi_T;

#endif                                 //typedef_sGoOVDSFGVYEhLxENxeJvlE_dynsi_T

#ifndef struct_tag_8EP4ctv7s0SDnh3V6WCLOG
#define struct_tag_8EP4ctv7s0SDnh3V6WCLOG

struct tag_8EP4ctv7s0SDnh3V6WCLOG
{
  real_T f1[16];
};

#endif                                 //struct_tag_8EP4ctv7s0SDnh3V6WCLOG

#ifndef typedef_f_cell_wrap_dynsim_edo_T
#define typedef_f_cell_wrap_dynsim_edo_T

typedef struct tag_8EP4ctv7s0SDnh3V6WCLOG f_cell_wrap_dynsim_edo_T;

#endif                                 //typedef_f_cell_wrap_dynsim_edo_T

#ifndef struct_tag_rHHvNgfpb6xtLaCol8pZFD
#define struct_tag_rHHvNgfpb6xtLaCol8pZFD

struct tag_rHHvNgfpb6xtLaCol8pZFD
{
  emxArray_char_T_dynsim_edo_T *Type;
  real_T PositionNumber;
  emxArray_real_T_dynsim_edo_T *MotionSubspace;
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
  real_T JointAxisInternal[3];
};

#endif                                 //struct_tag_rHHvNgfpb6xtLaCol8pZFD

#ifndef typedef_c_rigidBodyJoint_dynsim_edo_T
#define typedef_c_rigidBodyJoint_dynsim_edo_T

typedef struct tag_rHHvNgfpb6xtLaCol8pZFD c_rigidBodyJoint_dynsim_edo_T;

#endif                                 //typedef_c_rigidBodyJoint_dynsim_edo_T

#ifndef struct_tag_4d0yO3DNkhaNLE6JP8DrI
#define struct_tag_4d0yO3DNkhaNLE6JP8DrI

struct tag_4d0yO3DNkhaNLE6JP8DrI
{
  real_T Index;
  char_T NameInternal[6];
  c_rigidBodyJoint_dynsim_edo_T JointInternal;
  real_T ParentIndex;
};

#endif                                 //struct_tag_4d0yO3DNkhaNLE6JP8DrI

#ifndef typedef_l_robotics_manip_internal_Rig_T
#define typedef_l_robotics_manip_internal_Rig_T

typedef struct tag_4d0yO3DNkhaNLE6JP8DrI l_robotics_manip_internal_Rig_T;

#endif                                 //typedef_l_robotics_manip_internal_Rig_T

#ifndef struct_tag_9ECOrffzte7r0XovPIluy
#define struct_tag_9ECOrffzte7r0XovPIluy

struct tag_9ECOrffzte7r0XovPIluy
{
  emxArray_char_T_dynsim_edo_T *NameInternal;
  c_rigidBodyJoint_dynsim_edo_T JointInternal;
};

#endif                                 //struct_tag_9ECOrffzte7r0XovPIluy

#ifndef typedef_m_robotics_manip_internal_Rig_T
#define typedef_m_robotics_manip_internal_Rig_T

typedef struct tag_9ECOrffzte7r0XovPIluy m_robotics_manip_internal_Rig_T;

#endif                                 //typedef_m_robotics_manip_internal_Rig_T

#ifndef struct_tag_dasIWGj2VYFZ6wJYYj1swC
#define struct_tag_dasIWGj2VYFZ6wJYYj1swC

struct tag_dasIWGj2VYFZ6wJYYj1swC
{
  real_T NumBodies;
  m_robotics_manip_internal_Rig_T Base;
  l_robotics_manip_internal_Rig_T *Bodies[6];
  real_T PositionNumber;
  real_T VelocityNumber;
  real_T PositionDoFMap[12];
};

#endif                                 //struct_tag_dasIWGj2VYFZ6wJYYj1swC

#ifndef typedef_n_robotics_manip_internal_Rig_T
#define typedef_n_robotics_manip_internal_Rig_T

typedef struct tag_dasIWGj2VYFZ6wJYYj1swC n_robotics_manip_internal_Rig_T;

#endif                                 //typedef_n_robotics_manip_internal_Rig_T

#ifndef struct_tag_BlYT8UN04ehVwmWKnQ4DgD
#define struct_tag_BlYT8UN04ehVwmWKnQ4DgD

struct tag_BlYT8UN04ehVwmWKnQ4DgD
{
  int32_T isInitialized;
  n_robotics_manip_internal_Rig_T TreeInternal;
};

#endif                                 //struct_tag_BlYT8UN04ehVwmWKnQ4DgD

#ifndef typedef_robotics_slmanip_internal_blo_T
#define typedef_robotics_slmanip_internal_blo_T

typedef struct tag_BlYT8UN04ehVwmWKnQ4DgD robotics_slmanip_internal_blo_T;

#endif                                 //typedef_robotics_slmanip_internal_blo_T

#ifndef struct_tag_2xAJzS7rvIyFzfuC7FOGlD
#define struct_tag_2xAJzS7rvIyFzfuC7FOGlD

struct tag_2xAJzS7rvIyFzfuC7FOGlD
{
  emxArray_char_T_dynsim_edo_T *Type;
  real_T PositionNumber;
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
  real_T JointAxisInternal[3];
};

#endif                                 //struct_tag_2xAJzS7rvIyFzfuC7FOGlD

#ifndef typedef_c_rigidBodyJoint_dynsim_edo_h_T
#define typedef_c_rigidBodyJoint_dynsim_edo_h_T

typedef struct tag_2xAJzS7rvIyFzfuC7FOGlD c_rigidBodyJoint_dynsim_edo_h_T;

#endif                                 //typedef_c_rigidBodyJoint_dynsim_edo_h_T

#ifndef struct_tag_AV1LBgFrotghMr9djKBRN
#define struct_tag_AV1LBgFrotghMr9djKBRN

struct tag_AV1LBgFrotghMr9djKBRN
{
  char_T NameInternal[6];
  c_rigidBodyJoint_dynsim_edo_h_T JointInternal;
  real_T ParentIndex;
};

#endif                                 //struct_tag_AV1LBgFrotghMr9djKBRN

#ifndef typedef_l_robotics_manip_internal_R_h_T
#define typedef_l_robotics_manip_internal_R_h_T

typedef struct tag_AV1LBgFrotghMr9djKBRN l_robotics_manip_internal_R_h_T;

#endif                                 //typedef_l_robotics_manip_internal_R_h_T

#ifndef struct_tag_EpnOXkcFkOFmcaZiu2CuCD
#define struct_tag_EpnOXkcFkOFmcaZiu2CuCD

struct tag_EpnOXkcFkOFmcaZiu2CuCD
{
  emxArray_char_T_dynsim_edo_T *NameInternal;
  c_rigidBodyJoint_dynsim_edo_h_T JointInternal;
};

#endif                                 //struct_tag_EpnOXkcFkOFmcaZiu2CuCD

#ifndef typedef_m_robotics_manip_internal_R_h_T
#define typedef_m_robotics_manip_internal_R_h_T

typedef struct tag_EpnOXkcFkOFmcaZiu2CuCD m_robotics_manip_internal_R_h_T;

#endif                                 //typedef_m_robotics_manip_internal_R_h_T

#ifndef struct_tag_vjc6KIMqa87986f8hhDLFH
#define struct_tag_vjc6KIMqa87986f8hhDLFH

struct tag_vjc6KIMqa87986f8hhDLFH
{
  real_T NumBodies;
  m_robotics_manip_internal_R_h_T Base;
  l_robotics_manip_internal_R_h_T *Bodies[6];
  real_T PositionNumber;
};

#endif                                 //struct_tag_vjc6KIMqa87986f8hhDLFH

#ifndef typedef_n_robotics_manip_internal_R_h_T
#define typedef_n_robotics_manip_internal_R_h_T

typedef struct tag_vjc6KIMqa87986f8hhDLFH n_robotics_manip_internal_R_h_T;

#endif                                 //typedef_n_robotics_manip_internal_R_h_T

#ifndef struct_tag_cGBQnFP5Mh4Emo8puqX8QF
#define struct_tag_cGBQnFP5Mh4Emo8puqX8QF

struct tag_cGBQnFP5Mh4Emo8puqX8QF
{
  int32_T isInitialized;
  n_robotics_manip_internal_R_h_T TreeInternal;
};

#endif                                 //struct_tag_cGBQnFP5Mh4Emo8puqX8QF

#ifndef typedef_robotics_slmanip_internal_b_h_T
#define typedef_robotics_slmanip_internal_b_h_T

typedef struct tag_cGBQnFP5Mh4Emo8puqX8QF robotics_slmanip_internal_b_h_T;

#endif                                 //typedef_robotics_slmanip_internal_b_h_T

#ifndef struct_tag_pGgszObO16I6TGXaEMnuXB
#define struct_tag_pGgszObO16I6TGXaEMnuXB

struct tag_pGgszObO16I6TGXaEMnuXB
{
  real_T f1[36];
};

#endif                                 //struct_tag_pGgszObO16I6TGXaEMnuXB

#ifndef typedef_f_cell_wrap_dynsim_edo_h_T
#define typedef_f_cell_wrap_dynsim_edo_h_T

typedef struct tag_pGgszObO16I6TGXaEMnuXB f_cell_wrap_dynsim_edo_h_T;

#endif                                 //typedef_f_cell_wrap_dynsim_edo_h_T

#ifndef struct_tag_QsLFVUgAtzQUBY1V99co0D
#define struct_tag_QsLFVUgAtzQUBY1V99co0D

struct tag_QsLFVUgAtzQUBY1V99co0D
{
  emxArray_char_T_dynsim_edo_T *Type;
  emxArray_real_T_dynsim_edo_T *MotionSubspace;
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
  real_T JointAxisInternal[3];
};

#endif                                 //struct_tag_QsLFVUgAtzQUBY1V99co0D

#ifndef typedef_c_rigidBodyJoint_dynsim_ed_ha_T
#define typedef_c_rigidBodyJoint_dynsim_ed_ha_T

typedef struct tag_QsLFVUgAtzQUBY1V99co0D c_rigidBodyJoint_dynsim_ed_ha_T;

#endif                                 //typedef_c_rigidBodyJoint_dynsim_ed_ha_T

#ifndef struct_tag_IYTryndM9hCl2aQvRVOEpC
#define struct_tag_IYTryndM9hCl2aQvRVOEpC

struct tag_IYTryndM9hCl2aQvRVOEpC
{
  real_T Index;
  c_rigidBodyJoint_dynsim_ed_ha_T JointInternal;
  real_T ParentIndex;
  real_T SpatialInertia[36];
};

#endif                                 //struct_tag_IYTryndM9hCl2aQvRVOEpC

#ifndef typedef_l_robotics_manip_internal__ha_T
#define typedef_l_robotics_manip_internal__ha_T

typedef struct tag_IYTryndM9hCl2aQvRVOEpC l_robotics_manip_internal__ha_T;

#endif                                 //typedef_l_robotics_manip_internal__ha_T

#ifndef struct_tag_Y04n94zUMq8rhLX3OxwtfD
#define struct_tag_Y04n94zUMq8rhLX3OxwtfD

struct tag_Y04n94zUMq8rhLX3OxwtfD
{
  c_rigidBodyJoint_dynsim_ed_ha_T JointInternal;
};

#endif                                 //struct_tag_Y04n94zUMq8rhLX3OxwtfD

#ifndef typedef_m_robotics_manip_internal__ha_T
#define typedef_m_robotics_manip_internal__ha_T

typedef struct tag_Y04n94zUMq8rhLX3OxwtfD m_robotics_manip_internal__ha_T;

#endif                                 //typedef_m_robotics_manip_internal__ha_T

#ifndef struct_tag_mYgZv8X39cdHbKQNQhG0oD
#define struct_tag_mYgZv8X39cdHbKQNQhG0oD

struct tag_mYgZv8X39cdHbKQNQhG0oD
{
  real_T NumBodies;
  m_robotics_manip_internal__ha_T Base;
  real_T Gravity[3];
  l_robotics_manip_internal__ha_T *Bodies[6];
  real_T VelocityNumber;
  real_T PositionDoFMap[12];
  real_T VelocityDoFMap[12];
};

#endif                                 //struct_tag_mYgZv8X39cdHbKQNQhG0oD

#ifndef typedef_n_robotics_manip_internal__ha_T
#define typedef_n_robotics_manip_internal__ha_T

typedef struct tag_mYgZv8X39cdHbKQNQhG0oD n_robotics_manip_internal__ha_T;

#endif                                 //typedef_n_robotics_manip_internal__ha_T

#ifndef struct_tag_uU4fM87gRHLNKfBsLIBxeE
#define struct_tag_uU4fM87gRHLNKfBsLIBxeE

struct tag_uU4fM87gRHLNKfBsLIBxeE
{
  int32_T isInitialized;
  n_robotics_manip_internal__ha_T TreeInternal;
};

#endif                                 //struct_tag_uU4fM87gRHLNKfBsLIBxeE

#ifndef typedef_robotics_slmanip_internal__ha_T
#define typedef_robotics_slmanip_internal__ha_T

typedef struct tag_uU4fM87gRHLNKfBsLIBxeE robotics_slmanip_internal__ha_T;

#endif                                 //typedef_robotics_slmanip_internal__ha_T

#ifndef struct_emxArray_tag_8EP4ctv7s0SDnh3V6W
#define struct_emxArray_tag_8EP4ctv7s0SDnh3V6W

struct emxArray_tag_8EP4ctv7s0SDnh3V6W
{
  f_cell_wrap_dynsim_edo_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_tag_8EP4ctv7s0SDnh3V6W

#ifndef typedef_emxArray_f_cell_wrap_dynsim_e_T
#define typedef_emxArray_f_cell_wrap_dynsim_e_T

typedef struct emxArray_tag_8EP4ctv7s0SDnh3V6W emxArray_f_cell_wrap_dynsim_e_T;

#endif                                 //typedef_emxArray_f_cell_wrap_dynsim_e_T

#ifndef struct_emxArray_tag_pGgszObO16I6TGXaEM
#define struct_emxArray_tag_pGgszObO16I6TGXaEM

struct emxArray_tag_pGgszObO16I6TGXaEM
{
  f_cell_wrap_dynsim_edo_h_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_tag_pGgszObO16I6TGXaEM

#ifndef typedef_emxArray_f_cell_wrap_dynsim_h_T
#define typedef_emxArray_f_cell_wrap_dynsim_h_T

typedef struct emxArray_tag_pGgszObO16I6TGXaEM emxArray_f_cell_wrap_dynsim_h_T;

#endif                                 //typedef_emxArray_f_cell_wrap_dynsim_h_T

// Parameters (default storage)
typedef struct P_dynsim_edo_T_ P_dynsim_edo_T;

// Forward declaration for rtModel
typedef struct tag_RTM_dynsim_edo_T RT_MODEL_dynsim_edo_T;

#endif                                 // RTW_HEADER_dynsim_edo_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
