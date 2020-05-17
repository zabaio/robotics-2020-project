//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: dynsim_edo.h
//
// Code generated for Simulink model 'dynsim_edo'.
//
// Model version                  : 1.118
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Sun May 17 22:57:07 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_dynsim_edo_h_
#define RTW_HEADER_dynsim_edo_h_
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#include "dynsim_edo_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
# define rtmGetOdeY(rtm)               ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
# define rtmSetOdeY(rtm, val)          ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

// Block signals (default storage)
typedef struct {
  SL_Bus_dynsim_edo_sensor_msgs_JointState msg_g;// '<Root>/Assign to CartesianState msg' 
  SL_Bus_dynsim_edo_std_msgs_Float64MultiArray In1;// '<S12>/In1'
  SL_Bus_dynsim_edo_std_msgs_Float64MultiArray b_varargout_2;
  SL_Bus_dynsim_edo_std_msgs_MultiArrayDimension b_varargout_2_Layout_Dim[16];
  real_T b_varargout_2_Data[128];
  real_T b_I[36];
  real_T R[36];
  real_T X[36];
  real_T T[16];
  real_T Tinv[16];
  real_T TJ[16];
  real_T obj[16];
  real_T T_m[16];
  real_T dv[16];
  real_T TJ_c[16];
  real_T obj_k[16];
  real_T R_c[9];
  real_T R_b[9];
  real_T dv1[9];
  real_T R_p[9];
  real_T tempR[9];
  real_T R_cv[9];
  real_T tempR_f[9];
  real_T dv2[9];
  real_T dv3[9];
  real_T R_g[9];
  real_T R_g1[9];
  real_T Velocity[6];                  // '<S11>/Velocity'
  real_T MATLABSystem[6];              // '<S13>/MATLAB System'
  real_T a0[6];
  real_T q_data[6];
  real_T X_m[6];
  real_T b_I_n[6];
  real_T q_data_p[6];
  int8_T msubspace_data[36];
  real_T result_data[4];
  char_T cv[25];
  int32_T nonFixedIndices_data[6];
  int32_T ii_data[6];
  char_T cv1[24];
  real_T v[3];
  real_T v_l[3];
  char_T cv2[23];
  char_T cv3[18];
  SL_Bus_dynsim_edo_rosgraph_msgs_Clock msg_l;// '<Root>/Assign to Time msg'
  char_T cv4[14];
  char_T b[9];
  char_T b_j[9];
  char_T b_d[8];
  char_T b_g[8];
  char_T b_l[8];
  real_T value;
  real_T value_d;
  real_T value_dy;
  real_T vNum;
  real_T k;
  real_T j;
  real_T d;
  real_T d1;
  real_T nb;
  real_T a_idx_1;
  real_T a_idx_0;
  real_T b_idx_0;
  real_T b_idx_1;
  real_T cth;
  real_T sth;
  real_T tempR_tmp;
  real_T tempR_tmp_l;
  real_T tempR_tmp_o;
  real_T tempR_tmp_b;
  real_T tempR_tmp_n;
  real_T nb_b;
  real_T vNum_l;
  real_T pid;
  real_T s;
  real_T p_idx_1;
  real_T b_idx_0_h;
  real_T b_idx_1_b;
  real_T b_da;
  real_T axang_idx_0;
  real_T axang_idx_1;
  real_T axang_idx_2;
  cell_wrap_0_dynsim_edo_T b_e;
  cell_wrap_0_dynsim_edo_T c;
  cell_wrap_0_dynsim_edo_T d_b;
  cell_wrap_0_dynsim_edo_T e;
  cell_wrap_0_dynsim_edo_T f;
  cell_wrap_0_dynsim_edo_T g;
  boolean_T mask[6];
  char_T b_jz[5];
  char_T b_f[5];
  char_T b_a[5];
  char_T b_ju[5];
  int32_T n;
  int32_T iend;
  int32_T j_j;
  int32_T i;
  int32_T i_o;
  int32_T vNum_idx_0_tmp;
  int32_T MATLABSystem_tmp;
  int32_T b_k;
  int32_T p;
  int32_T m;
  int32_T inner;
  int32_T aoffset;
  int32_T i_n;
  int32_T q_size;
  int32_T unnamed_idx_1;
  int32_T loop_ub_tmp;
  int32_T q_size_tmp;
  int32_T kstr;
  int32_T b_kstr;
  int32_T obj_tmp;
  int32_T obj_tmp_tmp;
  int32_T kstr_i;
  int32_T b_kstr_o;
  int32_T b_i;
  int32_T f_n;
  int32_T cb;
  int32_T idx;
  int32_T n_m;
  int32_T nm1d2;
  int32_T m_c;
  int32_T coffset;
  int32_T boffset;
  int32_T aoffset_m;
  int32_T loop_ub;
  int32_T q_size_m;
  int32_T pid_tmp;
  int32_T X_tmp;
  int32_T coffset_tmp;
  int32_T kstr_j;
  int32_T b_kstr_h;
  int32_T obj_tmp_c;
  int32_T obj_tmp_tmp_c;
  int32_T i1;
  int32_T i2;
  int32_T X_tmp_p;
  int32_T X_tmp_p5;
  int32_T i3;
  int32_T Tinv_tmp;
  int32_T newNumel;
  int32_T i_a;
  int32_T newNumel_e;
  int32_T i_ax;
  int32_T newNumel_a;
  int32_T i_i;
  int32_T i_l;
  int32_T i_oj;
  int32_T i_o2;
  uint32_T b_varargout_2_Data_SL_Info_Curr;
  uint32_T b_varargout_2_Data_SL_Info_Rece;
  uint32_T b_varargout_2_Layout_DataOffset;
  uint32_T b_varargout_2_Layout_Dim_SL_Inf;
  uint32_T b_varargout_2_Layout_Dim_SL_I_f;
  boolean_T b_varargout_1;
  boolean_T b_bool;
  boolean_T b_bool_g;
  boolean_T b_bool_c;
  boolean_T b_bool_o;
  boolean_T b_bool_l;
} B_dynsim_edo_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  l_robotics_manip_internal_Rig_T gobj_1;// '<S13>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_2;// '<S13>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_3;// '<S13>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_4;// '<S13>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_5;// '<S13>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_6;// '<S13>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_7;// '<S13>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_8;// '<S13>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_9;// '<S13>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_10;// '<S13>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_11;// '<S13>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_12;// '<S13>/MATLAB System'
  robotics_slmanip_internal_blo_T obj; // '<S13>/MATLAB System'
  ros_slros_internal_block_GetP_T obj_p;// '<S14>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_n;// '<S14>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_nf;// '<S14>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_e;// '<S14>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_n4;// '<S14>/Get Parameter6'
  ros_slros_internal_block_GetP_T obj_f;// '<S14>/Get Parameter7'
  ros_slros_internal_block_GetP_T obj_h;// '<S14>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_b;// '<S14>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_nb;// '<S14>/Get Parameter8'
  ros_slros_internal_block_GetP_T obj_pd;// '<S14>/Get Parameter9'
  ros_slros_internal_block_GetP_T obj_eu;// '<S14>/Get Parameter10'
  ros_slros_internal_block_GetP_T obj_ez;// '<S14>/Get Parameter11'
  ros_slros_internal_block_Publ_T obj_a;// '<S9>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_f2;// '<S8>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_nr;// '<S7>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_pp;// '<S10>/SourceBlock'
  int_T Position_IWORK;                // '<S11>/Position'
  int_T Velocity_IWORK;                // '<S11>/Velocity'
} DW_dynsim_edo_T;

// Continuous states (default storage)
typedef struct {
  real_T Position_CSTATE[6];           // '<S11>/Position'
  real_T Velocity_CSTATE[6];           // '<S11>/Velocity'
} X_dynsim_edo_T;

// State derivatives (default storage)
typedef struct {
  real_T Position_CSTATE[6];           // '<S11>/Position'
  real_T Velocity_CSTATE[6];           // '<S11>/Velocity'
} XDot_dynsim_edo_T;

// State disabled
typedef struct {
  boolean_T Position_CSTATE[6];        // '<S11>/Position'
  boolean_T Velocity_CSTATE[6];        // '<S11>/Velocity'
} XDis_dynsim_edo_T;

#ifndef ODE3_INTG
#define ODE3_INTG

// ODE3 Integration Data
typedef struct {
  real_T *y;                           // output
  real_T *f[3];                        // derivatives
} ODE3_IntgData;

#endif

// Parameters (default storage)
struct P_dynsim_edo_T_ {
  SL_Bus_dynsim_edo_sensor_msgs_JointState Constant_Value;// Computed Parameter: Constant_Value
                                                             //  Referenced by: '<S4>/Constant'

  SL_Bus_dynsim_edo_std_msgs_Float64MultiArray Out1_Y0;// Computed Parameter: Out1_Y0
                                                          //  Referenced by: '<S12>/Out1'

  SL_Bus_dynsim_edo_std_msgs_Float64MultiArray Constant_Value_h;// Computed Parameter: Constant_Value_h
                                                                   //  Referenced by: '<S10>/Constant'

  SL_Bus_dynsim_edo_rosgraph_msgs_Clock Constant_Value_o;// Computed Parameter: Constant_Value_o
                                                            //  Referenced by: '<S5>/Constant'

  real_T Constant_Value_f[36];         // Expression: zeros(6,6)
                                          //  Referenced by: '<S11>/Constant'

  real_T Constant_Value_oa[2];         // Expression: [L1,L2]
                                          //  Referenced by: '<Root>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_dynsim_edo_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_dynsim_edo_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[12];
  real_T odeF[3][12];
  ODE3_IntgData intgData;

  //
  //  Sizes:
  //  The following substructure contains sizes information
  //  for many of the model attributes such as inputs, outputs,
  //  dwork, sample times, etc.

  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    boolean_T firstInitCondFlag;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_dynsim_edo_T dynsim_edo_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_dynsim_edo_T dynsim_edo_B;

#ifdef __cplusplus

}
#endif

// Continuous states (default storage)
extern X_dynsim_edo_T dynsim_edo_X;

// Block states (default storage)
extern DW_dynsim_edo_T dynsim_edo_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void dynsim_edo_initialize(void);
  extern void dynsim_edo_step(void);
  extern void dynsim_edo_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_dynsim_edo_T *const dynsim_edo_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S13>/Reshape' : Reshape block reduction


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'dynsim_edo'
//  '<S1>'   : 'dynsim_edo/Assign to CartesianState msg'
//  '<S2>'   : 'dynsim_edo/Assign to JointState msg'
//  '<S3>'   : 'dynsim_edo/Assign to Time msg'
//  '<S4>'   : 'dynsim_edo/JointState'
//  '<S5>'   : 'dynsim_edo/JointState1'
//  '<S6>'   : 'dynsim_edo/MATLAB Function'
//  '<S7>'   : 'dynsim_edo/Publish'
//  '<S8>'   : 'dynsim_edo/Publish1'
//  '<S9>'   : 'dynsim_edo/Publish2'
//  '<S10>'  : 'dynsim_edo/Subscribe'
//  '<S11>'  : 'dynsim_edo/edo robot dynamic model'
//  '<S12>'  : 'dynsim_edo/Subscribe/Enabled Subsystem'
//  '<S13>'  : 'dynsim_edo/edo robot dynamic model/Forward Dynamics'
//  '<S14>'  : 'dynsim_edo/edo robot dynamic model/Subsystem'

#endif                                 // RTW_HEADER_dynsim_edo_h_

//
// File trailer for generated code.
//
// [EOF]
//
