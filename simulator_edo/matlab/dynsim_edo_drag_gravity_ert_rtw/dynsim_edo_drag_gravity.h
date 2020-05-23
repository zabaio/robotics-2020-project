//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: dynsim_edo_drag_gravity.h
//
// Code generated for Simulink model 'dynsim_edo_drag_gravity'.
//
// Model version                  : 1.123
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Sat May 23 12:35:45 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_dynsim_edo_drag_gravity_h_
#define RTW_HEADER_dynsim_edo_drag_gravity_h_
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <string.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#include "dynsim_edo_drag_gravity_types.h"
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
  SL_Bus_dynsim_edo_drag_gravity_sensor_msgs_JointState msg_g;// '<Root>/Assign to CartesianState msg' 
  SL_Bus_dynsim_edo_drag_gravity_std_msgs_Float64MultiArray In1;// '<S14>/In1'
  SL_Bus_dynsim_edo_drag_gravity_std_msgs_Float64MultiArray b_varargout_2;
  SL_Bus_dynsim_edo_drag_gravity_std_msgs_MultiArrayDimension
    b_varargout_2_Layout_Dim[16];
  real_T b_varargout_2_Data[128];
  real_T X[36];
  real_T b_I[36];
  real_T R[36];
  real_T X_m[36];
  real_T T1[16];
  real_T MATLABSystem_k[16];           // '<S5>/MATLAB System'
  real_T R_c[16];
  real_T T2inv[16];
  real_T T2[16];
  real_T T1_b[16];
  real_T T[16];
  real_T Tdh[16];
  real_T c_f1[16];
  real_T a[16];
  real_T b[16];
  real_T a_p[16];
  real_T T_c[16];
  real_T Tinv[16];
  real_T TJ[16];
  real_T obj[16];
  real_T T_f[16];
  real_T dv[16];
  real_T TJ_g[16];
  real_T obj_g[16];
  real_T c_f1_m[16];
  real_T a_n[16];
  real_T b_p[16];
  real_T a_l[16];
  f_cell_wrap_dynsim_edo_drag_g_T expl_temp;
  f_cell_wrap_dynsim_edo_drag_g_T expl_temp_c;
  real_T R_j[9];
  real_T R_d[9];
  real_T R_g[9];
  real_T R_l[9];
  real_T R_dh[9];
  real_T tempR[9];
  real_T R_dy[9];
  real_T R_lx[9];
  real_T dv1[9];
  real_T R_o[9];
  real_T tempR_b[9];
  real_T R_n[9];
  real_T tempR_bs[9];
  real_T R_ln[9];
  real_T tempR_h[9];
  real_T dv2[9];
  real_T dv3[9];
  real_T R_b[9];
  real_T R_da[9];
  real_T a0[6];
  real_T y[6];
  real_T X_e[6];
  real_T b_I_b[6];
  int8_T msubspace_data[36];
  real_T cartVel[4];                   // '<Root>/MATLAB Function'
  real_T result_data[4];
  real_T result_data_j[4];
  real_T result_data_f[4];
  char_T cv[25];
  int32_T nonFixedIndices_data[6];
  int32_T ii_data[6];
  char_T cv1[24];
  real_T Velocity[3];                  // '<S13>/Velocity'
  real_T MATLABSystem[3];              // '<S15>/MATLAB System'
  real_T R_a[3];
  real_T v[3];
  real_T q_data[3];
  real_T v_j[3];
  real_T q_data_j[3];
  real_T v_o[3];
  real_T v_n[3];
  char_T cv2[23];
  char_T cv3[18];
  int8_T T2_i[16];
  SL_Bus_dynsim_edo_drag_gravity_rosgraph_msgs_Clock msg_l;// '<Root>/Assign to Time msg' 
  char_T cv4[14];
  int32_T e_data[3];
  int32_T e_data_o[3];
  char_T b_n[9];
  char_T b_m[9];
  char_T b_c[9];
  char_T b_md[9];
  char_T b_m3[8];
  char_T b_j[8];
  char_T b_h[8];
  char_T b_c0[8];
  char_T b_ct[8];
  char_T b_px[8];
  char_T b_p5[8];
  real_T bid1;
  real_T k;
  real_T j;
  real_T endeffectorIndex;
  real_T s;
  real_T idx_idx_1;
  real_T n;
  real_T k_a;
  real_T sth;
  real_T tempR_tmp;
  real_T tempR_tmp_e;
  real_T tempR_tmp_a;
  real_T tempR_tmp_as;
  real_T nb;
  real_T a_idx_1;
  real_T a_idx_0;
  real_T b_idx_0;
  real_T b_idx_1;
  real_T cth;
  real_T sth_i;
  real_T tempR_tmp_l;
  real_T tempR_tmp_o;
  real_T tempR_tmp_o2;
  real_T tempR_tmp_i;
  real_T tempR_tmp_f;
  real_T nb_i;
  real_T vNum;
  real_T pid;
  real_T s_f;
  real_T p_idx_1;
  real_T b_idx_0_g;
  real_T b_idx_1_c;
  real_T b_o;
  real_T axang_idx_0;
  real_T axang_idx_1;
  real_T axang_idx_2;
  real_T n_l;
  real_T k_m;
  real_T sth_m;
  real_T tempR_tmp_c;
  real_T tempR_tmp_fm;
  real_T tempR_tmp_p;
  real_T tempR_tmp_e1;
  cell_wrap_0_dynsim_edo_drag_g_T b_o4;
  cell_wrap_0_dynsim_edo_drag_g_T c;
  cell_wrap_0_dynsim_edo_drag_g_T d;
  int8_T chainmask[6];
  boolean_T mask[6];
  char_T bname[6];
  char_T b_hh[6];
  char_T a_l5[6];
  char_T bname_h[6];
  char_T b_me[5];
  char_T b_mc[5];
  char_T b_h3[5];
  char_T b_cs[5];
  char_T b_k[5];
  char_T b_pc[5];
  char_T b_pxv[5];
  int32_T ret;
  int32_T n_p;
  int32_T iend;
  int32_T loop_ub;
  int32_T u1;
  int32_T rtb_MATLABSystem_tmp;
  int32_T i;
  int32_T c_a;
  int32_T b_i;
  int32_T kstr;
  int32_T n_j;
  int32_T ret_e;
  int32_T loop_ub_o;
  int32_T coffset_tmp;
  int32_T d_b;
  int32_T e;
  int32_T ntilecols;
  int32_T b_jtilecol;
  int32_T b_kstr;
  int32_T loop_ub_a;
  int32_T kstr_g;
  int32_T b_kstr_e;
  int32_T b_k_f;
  int32_T p;
  int32_T b_k_h;
  int32_T m;
  int32_T inner;
  int32_T aoffset;
  int32_T q_size;
  int32_T unnamed_idx_1;
  int32_T loop_ub_tmp;
  int32_T q_size_tmp;
  int32_T kstr_e;
  int32_T b_kstr_c;
  int32_T obj_tmp;
  int32_T obj_tmp_tmp;
  int32_T kstr_a;
  int32_T b_kstr_d;
  int32_T b_i_a;
  int32_T f;
  int32_T cb;
  int32_T idx;
  int32_T n_pb;
  int32_T nm1d2;
  int32_T m_m;
  int32_T coffset;
  int32_T boffset;
  int32_T aoffset_o;
  int32_T loop_ub_n;
  int32_T q_size_l;
  int32_T pid_tmp;
  int32_T X_tmp;
  int32_T coffset_tmp_p;
  int32_T kstr_p;
  int32_T b_kstr_f;
  int32_T obj_tmp_i;
  int32_T obj_tmp_tmp_o;
  int32_T d_k;
  int32_T e_i;
  int32_T ntilecols_o;
  int32_T b_jtilecol_m;
  int32_T b_kstr_cu;
  int32_T loop_ub_f;
  int32_T kstr_h;
  int32_T b_kstr_m;
  int32_T i1;
  int32_T i2;
  int32_T X_tmp_a;
  int32_T X_tmp_k;
  int32_T i3;
  int32_T Tinv_tmp;
  int32_T newNumel;
  int32_T i_p;
  int32_T newNumel_b;
  int32_T i_c;
  int32_T newNumel_n;
  int32_T i_i;
  int32_T i_m;
  int32_T i_j;
  int32_T i_e;
  uint32_T b_varargout_2_Data_SL_Info_Curr;
  uint32_T b_varargout_2_Data_SL_Info_Rece;
  uint32_T b_varargout_2_Layout_DataOffset;
  uint32_T b_varargout_2_Layout_Dim_SL_Inf;
  uint32_T b_varargout_2_Layout_Dim_SL_I_f;
  boolean_T b_bool;
  boolean_T b_bool_a;
  boolean_T b_bool_g;
  boolean_T b_bool_n;
  boolean_T b_bool_d;
  boolean_T b_bool_na;
  boolean_T b_bool_c;
  boolean_T b_bool_f;
  boolean_T b_bool_p;
  boolean_T b_bool_p2;
  boolean_T b_bool_nj;
} B_dynsim_edo_drag_gravity_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  l_robotics_manip_internal__ld_T gobj_1;// '<S15>/MATLAB System'
  l_robotics_manip_internal__ld_T gobj_2;// '<S15>/MATLAB System'
  l_robotics_manip_internal__ld_T gobj_3;// '<S15>/MATLAB System'
  l_robotics_manip_internal__ld_T gobj_4;// '<S15>/MATLAB System'
  l_robotics_manip_internal__ld_T gobj_5;// '<S15>/MATLAB System'
  l_robotics_manip_internal__ld_T gobj_6;// '<S15>/MATLAB System'
  l_robotics_manip_internal__ld_T gobj_7;// '<S15>/MATLAB System'
  l_robotics_manip_internal__ld_T gobj_8;// '<S15>/MATLAB System'
  l_robotics_manip_internal__ld_T gobj_9;// '<S15>/MATLAB System'
  l_robotics_manip_internal__ld_T gobj_10;// '<S15>/MATLAB System'
  l_robotics_manip_internal__ld_T gobj_11;// '<S15>/MATLAB System'
  l_robotics_manip_internal__ld_T gobj_12;// '<S15>/MATLAB System'
  robotics_slmanip_internal__ld_T obj; // '<S15>/MATLAB System'
  robotics_slmanip_internal_blo_T obj_a;// '<S4>/MATLAB System'
  robotics_slmanip_internal_b_l_T obj_o;// '<S5>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_1_p;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_2_a;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_3_g;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_4_b;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_5_l;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_6_n;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_7_p;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_8_l;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_9_d;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_10_h;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_11_a;// '<S4>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_12_m;// '<S4>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_1_pg;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_2_j;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_3_m;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_4_c;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_5_o;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_6_o;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_7_h;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_8_o;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_9_b;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_10_k;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_11_d;// '<S5>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_12_c;// '<S5>/MATLAB System'
  ros_slros_internal_block_GetP_T obj_p;// '<S16>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_n;// '<S16>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_nf;// '<S16>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_h;// '<S16>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_b;// '<S16>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_nb;// '<S16>/Get Parameter8'
  ros_slros_internal_block_Publ_T obj_ar;// '<S11>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_f;// '<S10>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_nr;// '<S9>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_pp;// '<S12>/SourceBlock'
  int_T Position_IWORK;                // '<S13>/Position'
  int_T Velocity_IWORK;                // '<S13>/Velocity'
} DW_dynsim_edo_drag_gravity_T;

// Continuous states (default storage)
typedef struct {
  real_T Position_CSTATE[3];           // '<S13>/Position'
  real_T Velocity_CSTATE[3];           // '<S13>/Velocity'
} X_dynsim_edo_drag_gravity_T;

// State derivatives (default storage)
typedef struct {
  real_T Position_CSTATE[3];           // '<S13>/Position'
  real_T Velocity_CSTATE[3];           // '<S13>/Velocity'
} XDot_dynsim_edo_drag_gravity_T;

// State disabled
typedef struct {
  boolean_T Position_CSTATE[3];        // '<S13>/Position'
  boolean_T Velocity_CSTATE[3];        // '<S13>/Velocity'
} XDis_dynsim_edo_drag_gravity_T;

#ifndef ODE3_INTG
#define ODE3_INTG

// ODE3 Integration Data
typedef struct {
  real_T *y;                           // output
  real_T *f[3];                        // derivatives
} ODE3_IntgData;

#endif

// Parameters (default storage)
struct P_dynsim_edo_drag_gravity_T_ {
  SL_Bus_dynsim_edo_drag_gravity_sensor_msgs_JointState Constant_Value;// Computed Parameter: Constant_Value
                                                                      //  Referenced by: '<S6>/Constant'

  SL_Bus_dynsim_edo_drag_gravity_std_msgs_Float64MultiArray Out1_Y0;// Computed Parameter: Out1_Y0
                                                                      //  Referenced by: '<S14>/Out1'

  SL_Bus_dynsim_edo_drag_gravity_std_msgs_Float64MultiArray Constant_Value_h;// Computed Parameter: Constant_Value_h
                                                                      //  Referenced by: '<S12>/Constant'

  SL_Bus_dynsim_edo_drag_gravity_rosgraph_msgs_Clock Constant_Value_o;// Computed Parameter: Constant_Value_o
                                                                      //  Referenced by: '<S7>/Constant'

  real_T Constant_Value_f[36];         // Expression: zeros(6,6)
                                          //  Referenced by: '<S13>/Constant'

  real_T Gain_Gain;                    // Expression: 0.02
                                          //  Referenced by: '<S13>/Gain'

};

// Real-time Model Data Structure
struct tag_RTM_dynsim_edo_drag_gravi_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_dynsim_edo_drag_gravity_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[6];
  real_T odeF[3][6];
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

  extern P_dynsim_edo_drag_gravity_T dynsim_edo_drag_gravity_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_dynsim_edo_drag_gravity_T dynsim_edo_drag_gravity_B;

#ifdef __cplusplus

}
#endif

// Continuous states (default storage)
extern X_dynsim_edo_drag_gravity_T dynsim_edo_drag_gravity_X;

// Block states (default storage)
extern DW_dynsim_edo_drag_gravity_T dynsim_edo_drag_gravity_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void dynsim_edo_drag_gravity_initialize(void);
  extern void dynsim_edo_drag_gravity_step(void);
  extern void dynsim_edo_drag_gravity_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_dynsim_edo_drag_grav_T *const dynsim_edo_drag_gravity_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S15>/Reshape' : Reshape block reduction


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
//  '<Root>' : 'dynsim_edo_drag_gravity'
//  '<S1>'   : 'dynsim_edo_drag_gravity/Assign to CartesianState msg'
//  '<S2>'   : 'dynsim_edo_drag_gravity/Assign to JointState msg'
//  '<S3>'   : 'dynsim_edo_drag_gravity/Assign to Time msg'
//  '<S4>'   : 'dynsim_edo_drag_gravity/Get Jacobian'
//  '<S5>'   : 'dynsim_edo_drag_gravity/Get Transform'
//  '<S6>'   : 'dynsim_edo_drag_gravity/JointState'
//  '<S7>'   : 'dynsim_edo_drag_gravity/JointState1'
//  '<S8>'   : 'dynsim_edo_drag_gravity/MATLAB Function'
//  '<S9>'   : 'dynsim_edo_drag_gravity/Publish'
//  '<S10>'  : 'dynsim_edo_drag_gravity/Publish1'
//  '<S11>'  : 'dynsim_edo_drag_gravity/Publish2'
//  '<S12>'  : 'dynsim_edo_drag_gravity/Subscribe'
//  '<S13>'  : 'dynsim_edo_drag_gravity/edo robot dynamic model'
//  '<S14>'  : 'dynsim_edo_drag_gravity/Subscribe/Enabled Subsystem'
//  '<S15>'  : 'dynsim_edo_drag_gravity/edo robot dynamic model/Forward Dynamics'
//  '<S16>'  : 'dynsim_edo_drag_gravity/edo robot dynamic model/Subsystem'

#endif                                 // RTW_HEADER_dynsim_edo_drag_gravity_h_

//
// File trailer for generated code.
//
// [EOF]
//
