//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: kinsim_edo.h
//
// Code generated for Simulink model 'kinsim_edo'.
//
// Model version                  : 1.135
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Wed May 20 12:50:27 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_kinsim_edo_h_
#define RTW_HEADER_kinsim_edo_h_
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <string.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#include "kinsim_edo_types.h"
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
  SL_Bus_kinsim_edo_sensor_msgs_JointState msg_d;// '<Root>/Assign to CartesianState msg' 
  SL_Bus_kinsim_edo_trajectory_msgs_JointTrajectoryPoint In1;// '<S14>/In1'
  SL_Bus_kinsim_edo_trajectory_msgs_JointTrajectoryPoint b_varargout_2;
  real_T b_varargout_2_Positions[128];
  real_T b_varargout_2_Velocities[128];
  real_T b_varargout_2_Accelerations[128];
  real_T b_varargout_2_Effort[128];
  real_T X[36];
  real_T T1[16];
  real_T MATLABSystem[16];             // '<S5>/MATLAB System'
  real_T R[16];
  real_T T2inv[16];
  real_T T2[16];
  real_T T1_c[16];
  real_T T[16];
  real_T Tdh[16];
  real_T c_f1[16];
  real_T a[16];
  real_T b[16];
  real_T a_k[16];
  real_T c_f1_c[16];
  real_T a_b[16];
  real_T b_p[16];
  real_T a_c[16];
  f_cell_wrap_kinsim_edo_T expl_temp;
  f_cell_wrap_kinsim_edo_T expl_temp_m;
  real_T R_f[9];
  real_T R_g[9];
  real_T R_g1[9];
  real_T R_m[9];
  real_T R_n[9];
  real_T tempR[9];
  real_T R_p[9];
  real_T tempR_l[9];
  int8_T msubspace_data[36];
  real_T cartVel[4];                   // '<Root>/MATLAB Function'
  real_T result_data[4];
  real_T result_data_j[4];
  real_T R_d[3];
  real_T v[3];
  real_T v_g[3];
  char_T cv[23];
  char_T cv1[18];
  int8_T T2_l[16];
  SL_Bus_kinsim_edo_rosgraph_msgs_Clock msg_l;// '<Root>/Assign to Time msg'
  char_T cv2[14];
  int32_T e_data[3];
  int32_T e_data_d[3];
  char_T b_d[9];
  char_T b_l[9];
  char_T b_o[9];
  char_T b_b[8];
  char_T b_n[8];
  char_T b_bs[8];
  char_T b_ln[8];
  real_T value;
  real_T value_h;
  real_T bid1;
  real_T endeffectorIndex;
  real_T s;
  real_T idx_idx_1;
  real_T n;
  real_T k;
  real_T sth;
  real_T tempR_tmp;
  real_T tempR_tmp_b;
  real_T tempR_tmp_d;
  real_T tempR_tmp_e;
  real_T n_b;
  real_T k_j;
  real_T sth_f;
  real_T tempR_tmp_a;
  real_T tempR_tmp_j;
  real_T tempR_tmp_jz;
  real_T tempR_tmp_o;
  cell_wrap_0_kinsim_edo_a_T b_ny;
  cell_wrap_0_kinsim_edo_a_T c;
  cell_wrap_0_kinsim_edo_a_T d;
  cell_wrap_0_kinsim_edo_T b_i;
  cell_wrap_0_kinsim_edo_T c_o;
  cell_wrap_0_kinsim_edo_T d_n;
  int8_T chainmask[6];
  char_T bname[6];
  char_T b_m[6];
  char_T a_cz[6];
  char_T bname_m[6];
  char_T b_m3[5];
  char_T b_j[5];
  char_T b_h[5];
  int32_T ret;
  int32_T loop_ub;
  int32_T rtb_MATLABSystem_tmp;
  int32_T rtb_MATLABSystem_tmp_tmp;
  int32_T c_c;
  int32_T b_i_c;
  int32_T kstr;
  int32_T n_p;
  int32_T ret_p;
  int32_T loop_ub_a;
  int32_T coffset_tmp;
  int32_T d_e;
  int32_T e;
  int32_T ntilecols;
  int32_T b_jtilecol;
  int32_T b_kstr;
  int32_T loop_ub_ax;
  int32_T kstr_a;
  int32_T b_kstr_i;
  int32_T d_l;
  int32_T e_o;
  int32_T ntilecols_o;
  int32_T b_jtilecol_i;
  int32_T b_kstr_f;
  int32_T loop_ub_i;
  int32_T kstr_f;
  int32_T b_kstr_g;
  int32_T newNumel;
  int32_T i;
  int32_T newNumel_c;
  int32_T i_o;
  int32_T i_l;
  uint32_T b_varargout_2_Positions_SL_Info;
  uint32_T b_varargout_2_Positions_SL_In_m;
  uint32_T b_varargout_2_Velocities_SL_Inf;
  uint32_T b_varargout_2_Velocities_SL_I_f;
  uint32_T b_varargout_2_Accelerations_SL_;
  uint32_T b_varargout_2_Accelerations_S_e;
  uint32_T b_varargout_2_Effort_SL_Info_Cu;
  uint32_T b_varargout_2_Effort_SL_Info_Re;
  boolean_T b_bool;
  boolean_T b_bool_l;
  boolean_T b_bool_h;
  boolean_T b_bool_m;
  boolean_T b_bool_mc;
  boolean_T b_bool_h3;
} B_kinsim_edo_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  robotics_slmanip_internal_blo_T obj; // '<S4>/MATLAB System'
  robotics_slmanip_internal_b_p_T obj_i;// '<S5>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_1;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_2;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_3;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_4;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_5;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_6;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_7;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_8;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_9;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_10;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_11;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_12;// '<S4>/MATLAB System'
  l_robotics_manip_internal_R_p_T gobj_1_a;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_p_T gobj_2_b;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_p_T gobj_3_g;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_p_T gobj_4_m;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_p_T gobj_5_k;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_p_T gobj_6_p;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_p_T gobj_7_d;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_p_T gobj_8_c;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_p_T gobj_9_f;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_p_T gobj_10_g;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_p_T gobj_11_e;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_p_T gobj_12_h;// '<S5>/MATLAB System'
  ros_slros_internal_block_GetP_T obj_e;// '<S13>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_n;// '<S13>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_o;// '<S13>/Get Parameter5'
  ros_slros_internal_block_Publ_T obj_m;// '<S11>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_f;// '<S10>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_nr;// '<S9>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_p;// '<S12>/SourceBlock'
  int_T Integrator_IWORK;              // '<Root>/Integrator'
} DW_kinsim_edo_T;

// Continuous states (default storage)
typedef struct {
  real_T Integrator_CSTATE[3];         // '<Root>/Integrator'
} X_kinsim_edo_T;

// State derivatives (default storage)
typedef struct {
  real_T Integrator_CSTATE[3];         // '<Root>/Integrator'
} XDot_kinsim_edo_T;

// State disabled
typedef struct {
  boolean_T Integrator_CSTATE[3];      // '<Root>/Integrator'
} XDis_kinsim_edo_T;

#ifndef ODE3_INTG
#define ODE3_INTG

// ODE3 Integration Data
typedef struct {
  real_T *y;                           // output
  real_T *f[3];                        // derivatives
} ODE3_IntgData;

#endif

// Parameters (default storage)
struct P_kinsim_edo_T_ {
  SL_Bus_kinsim_edo_sensor_msgs_JointState Constant_Value;// Computed Parameter: Constant_Value
                                                             //  Referenced by: '<S6>/Constant'

  SL_Bus_kinsim_edo_trajectory_msgs_JointTrajectoryPoint Out1_Y0;// Computed Parameter: Out1_Y0
                                                                    //  Referenced by: '<S14>/Out1'

  SL_Bus_kinsim_edo_trajectory_msgs_JointTrajectoryPoint Constant_Value_h;// Computed Parameter: Constant_Value_h
                                                                      //  Referenced by: '<S12>/Constant'

  SL_Bus_kinsim_edo_rosgraph_msgs_Clock Constant_Value_o;// Computed Parameter: Constant_Value_o
                                                            //  Referenced by: '<S7>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_kinsim_edo_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_kinsim_edo_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[3];
  real_T odeF[3][3];
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

  extern P_kinsim_edo_T kinsim_edo_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_kinsim_edo_T kinsim_edo_B;

#ifdef __cplusplus

}
#endif

// Continuous states (default storage)
extern X_kinsim_edo_T kinsim_edo_X;

// Block states (default storage)
extern DW_kinsim_edo_T kinsim_edo_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void kinsim_edo_initialize(void);
  extern void kinsim_edo_step(void);
  extern void kinsim_edo_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_kinsim_edo_T *const kinsim_edo_M;

#ifdef __cplusplus

}
#endif

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
//  '<Root>' : 'kinsim_edo'
//  '<S1>'   : 'kinsim_edo/Assign to CartesianState msg'
//  '<S2>'   : 'kinsim_edo/Assign to JointState msg'
//  '<S3>'   : 'kinsim_edo/Assign to Time msg'
//  '<S4>'   : 'kinsim_edo/Get Jacobian'
//  '<S5>'   : 'kinsim_edo/Get Transform'
//  '<S6>'   : 'kinsim_edo/JointState'
//  '<S7>'   : 'kinsim_edo/JointState1'
//  '<S8>'   : 'kinsim_edo/MATLAB Function'
//  '<S9>'   : 'kinsim_edo/Publish'
//  '<S10>'  : 'kinsim_edo/Publish1'
//  '<S11>'  : 'kinsim_edo/Publish2'
//  '<S12>'  : 'kinsim_edo/Subscribe'
//  '<S13>'  : 'kinsim_edo/Subsystem'
//  '<S14>'  : 'kinsim_edo/Subscribe/Enabled Subsystem'

#endif                                 // RTW_HEADER_kinsim_edo_h_

//
// File trailer for generated code.
//
// [EOF]
//
