//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: kinsim_edo.cpp
//
// Code generated for Simulink model 'kinsim_edo'.
//
// Model version                  : 1.117
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Sun May 17 11:21:23 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "kinsim_edo.h"
#include "kinsim_edo_private.h"

// Block signals (default storage)
B_kinsim_edo_T kinsim_edo_B;

// Continuous states
X_kinsim_edo_T kinsim_edo_X;

// Block states (default storage)
DW_kinsim_edo_T kinsim_edo_DW;

// Real-time model
RT_MODEL_kinsim_edo_T kinsim_edo_M_ = RT_MODEL_kinsim_edo_T();
RT_MODEL_kinsim_edo_T *const kinsim_edo_M = &kinsim_edo_M_;

// Forward declaration for local functions
static void kinsim_edo_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Positions[128], uint32_T *varargout_2_Positions_SL_Info_C,
  uint32_T *varargout_2_Positions_SL_Info_R, real_T varargout_2_Velocities[128],
  uint32_T *varargout_2_Velocities_SL_Info_, uint32_T
  *varargout_2_Velocities_SL_Inf_0, real_T varargout_2_Accelerations[128],
  uint32_T *varargout_2_Accelerations_SL_In, uint32_T
  *varargout_2_Accelerations_SL__0, real_T varargout_2_Effort[128], uint32_T
  *varargout_2_Effort_SL_Info_Curr, uint32_T *varargout_2_Effort_SL_Info_Rece,
  real_T *varargout_2_TimeFromStart_Sec, real_T *varargout_2_TimeFromStart_Nsec);
static void matlabCodegenHandle_matlab_pcdk(ros_slros_internal_block_GetP_T *obj);
static void matlabCodegenHandle_matlabC_pcd(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj);

//
// This function updates continuous states using the ODE3 fixed-step
// solver algorithm
//
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  // Solver Matrices
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = static_cast<ODE3_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 6;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) memcpy(y, x,
                static_cast<uint_T>(nXc)*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  kinsim_edo_derivatives();

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  kinsim_edo_step();
  kinsim_edo_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  kinsim_edo_step();
  kinsim_edo_derivatives();

  // tnew = t + hA(3);
  // ynew = y + f*hB(:,3);
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

static void kinsim_edo_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Positions[128], uint32_T *varargout_2_Positions_SL_Info_C,
  uint32_T *varargout_2_Positions_SL_Info_R, real_T varargout_2_Velocities[128],
  uint32_T *varargout_2_Velocities_SL_Info_, uint32_T
  *varargout_2_Velocities_SL_Inf_0, real_T varargout_2_Accelerations[128],
  uint32_T *varargout_2_Accelerations_SL_In, uint32_T
  *varargout_2_Accelerations_SL__0, real_T varargout_2_Effort[128], uint32_T
  *varargout_2_Effort_SL_Info_Curr, uint32_T *varargout_2_Effort_SL_Info_Rece,
  real_T *varargout_2_TimeFromStart_Sec, real_T *varargout_2_TimeFromStart_Nsec)
{
  *varargout_1 = Sub_kinsim_edo_16.getLatestMessage(&kinsim_edo_B.b_varargout_2);
  *varargout_2_Positions_SL_Info_C =
    kinsim_edo_B.b_varargout_2.Positions_SL_Info.CurrentLength;
  *varargout_2_Positions_SL_Info_R =
    kinsim_edo_B.b_varargout_2.Positions_SL_Info.ReceivedLength;
  *varargout_2_Velocities_SL_Info_ =
    kinsim_edo_B.b_varargout_2.Velocities_SL_Info.CurrentLength;
  *varargout_2_Velocities_SL_Inf_0 =
    kinsim_edo_B.b_varargout_2.Velocities_SL_Info.ReceivedLength;
  *varargout_2_Accelerations_SL_In =
    kinsim_edo_B.b_varargout_2.Accelerations_SL_Info.CurrentLength;
  *varargout_2_Accelerations_SL__0 =
    kinsim_edo_B.b_varargout_2.Accelerations_SL_Info.ReceivedLength;
  memcpy(&varargout_2_Positions[0], &kinsim_edo_B.b_varargout_2.Positions[0],
         sizeof(real_T) << 7U);
  memcpy(&varargout_2_Velocities[0], &kinsim_edo_B.b_varargout_2.Velocities[0],
         sizeof(real_T) << 7U);
  memcpy(&varargout_2_Accelerations[0],
         &kinsim_edo_B.b_varargout_2.Accelerations[0], sizeof(real_T) << 7U);
  memcpy(&varargout_2_Effort[0], &kinsim_edo_B.b_varargout_2.Effort[0], sizeof
         (real_T) << 7U);
  *varargout_2_Effort_SL_Info_Curr =
    kinsim_edo_B.b_varargout_2.Effort_SL_Info.CurrentLength;
  *varargout_2_Effort_SL_Info_Rece =
    kinsim_edo_B.b_varargout_2.Effort_SL_Info.ReceivedLength;
  *varargout_2_TimeFromStart_Sec = kinsim_edo_B.b_varargout_2.TimeFromStart.Sec;
  *varargout_2_TimeFromStart_Nsec =
    kinsim_edo_B.b_varargout_2.TimeFromStart.Nsec;
}

static void matlabCodegenHandle_matlab_pcdk(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabC_pcd(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

// Model step function
void kinsim_edo_step(void)
{
  cell_wrap_0_kinsim_edo_T e;
  cell_wrap_0_kinsim_edo_T f;
  cell_wrap_0_kinsim_edo_T g;
  uint32_T b_varargout_2_Positions_SL_Info;
  uint32_T b_varargout_2_Positions_SL_In_0;
  uint32_T b_varargout_2_Velocities_SL_Inf;
  uint32_T b_varargout_2_Velocities_SL_I_0;
  uint32_T b_varargout_2_Accelerations_SL_;
  uint32_T b_varargout_2_Accelerations_S_0;
  uint32_T b_varargout_2_Effort_SL_Info_Cu;
  uint32_T b_varargout_2_Effort_SL_Info_Re;
  boolean_T b_varargout_1;
  int32_T i;
  static const char_T b[9] = { 'a', 'r', 'm', '_', 'j', 'o', 'i', 'n', 't' };

  static const char_T c[13] = { 'f', 'o', 'r', 'e', 'a', 'r', 'm', '_', 'j', 'o',
    'i', 'n', 't' };

  static const char_T h[7] = { 'j', 'o', 'i', 'n', 't', '_', '1' };

  static const char_T i_0[7] = { 'j', 'o', 'i', 'n', 't', '_', '2' };

  static const char_T j[7] = { 'j', 'o', 'i', 'n', 't', '_', '3' };

  static const char_T k[7] = { 'j', 'o', 'i', 'n', 't', '_', '4' };

  static const char_T l[7] = { 'j', 'o', 'i', 'n', 't', '_', '5' };

  static const char_T m[7] = { 'j', 'o', 'i', 'n', 't', '_', '6' };

  if (rtmIsMajorTimeStep(kinsim_edo_M)) {
    // set solver stop time
    rtsiSetSolverStopTime(&kinsim_edo_M->solverInfo,
                          ((kinsim_edo_M->Timing.clockTick0+1)*
      kinsim_edo_M->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep(kinsim_edo_M)) {
    kinsim_edo_M->Timing.t[0] = rtsiGetT(&kinsim_edo_M->solverInfo);
  }

  // MATLABSystem: '<S11>/Get Parameter'
  ParamGet_kinsim_edo_61.get_parameter(&kinsim_edo_B.value);

  // MATLABSystem: '<S11>/Get Parameter1'
  ParamGet_kinsim_edo_65.get_parameter(&kinsim_edo_B.value_m);

  // MATLABSystem: '<S11>/Get Parameter5'
  ParamGet_kinsim_edo_84.get_parameter(&kinsim_edo_B.value_c);

  // MATLABSystem: '<S11>/Get Parameter4'
  ParamGet_kinsim_edo_83.get_parameter(&kinsim_edo_B.value_k);

  // MATLABSystem: '<S11>/Get Parameter3'
  ParamGet_kinsim_edo_82.get_parameter(&kinsim_edo_B.value_cx);

  // MATLABSystem: '<S11>/Get Parameter2'
  ParamGet_kinsim_edo_81.get_parameter(&kinsim_edo_B.value_b);

  // Integrator: '<Root>/Integrator' incorporates:
  //   MATLABSystem: '<S11>/Get Parameter'
  //   MATLABSystem: '<S11>/Get Parameter1'
  //   MATLABSystem: '<S11>/Get Parameter2'
  //   MATLABSystem: '<S11>/Get Parameter3'
  //   MATLABSystem: '<S11>/Get Parameter4'
  //   MATLABSystem: '<S11>/Get Parameter5'

  if (kinsim_edo_DW.Integrator_IWORK != 0) {
    kinsim_edo_X.Integrator_CSTATE[0] = kinsim_edo_B.value;
    kinsim_edo_X.Integrator_CSTATE[1] = kinsim_edo_B.value_m;
    kinsim_edo_X.Integrator_CSTATE[2] = kinsim_edo_B.value_c;
    kinsim_edo_X.Integrator_CSTATE[3] = kinsim_edo_B.value_k;
    kinsim_edo_X.Integrator_CSTATE[4] = kinsim_edo_B.value_cx;
    kinsim_edo_X.Integrator_CSTATE[5] = kinsim_edo_B.value_b;
  }

  if (rtmIsMajorTimeStep(kinsim_edo_M)) {
    // Outputs for Atomic SubSystem: '<Root>/Subscribe'
    // MATLABSystem: '<S10>/SourceBlock' incorporates:
    //   Inport: '<S12>/In1'

    kinsim_edo_SystemCore_step(&b_varargout_1,
      kinsim_edo_B.b_varargout_2_Positions, &b_varargout_2_Positions_SL_Info,
      &b_varargout_2_Positions_SL_In_0, kinsim_edo_B.b_varargout_2_Velocities,
      &b_varargout_2_Velocities_SL_Inf, &b_varargout_2_Velocities_SL_I_0,
      kinsim_edo_B.b_varargout_2_Accelerations, &b_varargout_2_Accelerations_SL_,
      &b_varargout_2_Accelerations_S_0, kinsim_edo_B.b_varargout_2_Effort,
      &b_varargout_2_Effort_SL_Info_Cu, &b_varargout_2_Effort_SL_Info_Re,
      &kinsim_edo_B.value, &kinsim_edo_B.value_m);

    // Outputs for Enabled SubSystem: '<S10>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S12>/Enable'

    if (b_varargout_1) {
      kinsim_edo_B.In1.Positions_SL_Info.CurrentLength =
        b_varargout_2_Positions_SL_Info;
      kinsim_edo_B.In1.Positions_SL_Info.ReceivedLength =
        b_varargout_2_Positions_SL_In_0;
      kinsim_edo_B.In1.Velocities_SL_Info.CurrentLength =
        b_varargout_2_Velocities_SL_Inf;
      kinsim_edo_B.In1.Velocities_SL_Info.ReceivedLength =
        b_varargout_2_Velocities_SL_I_0;
      kinsim_edo_B.In1.Accelerations_SL_Info.CurrentLength =
        b_varargout_2_Accelerations_SL_;
      kinsim_edo_B.In1.Accelerations_SL_Info.ReceivedLength =
        b_varargout_2_Accelerations_S_0;
      memcpy(&kinsim_edo_B.In1.Positions[0],
             &kinsim_edo_B.b_varargout_2_Positions[0], sizeof(real_T) << 7U);
      memcpy(&kinsim_edo_B.In1.Velocities[0],
             &kinsim_edo_B.b_varargout_2_Velocities[0], sizeof(real_T) << 7U);
      memcpy(&kinsim_edo_B.In1.Accelerations[0],
             &kinsim_edo_B.b_varargout_2_Accelerations[0], sizeof(real_T) << 7U);
      memcpy(&kinsim_edo_B.In1.Effort[0], &kinsim_edo_B.b_varargout_2_Effort[0],
             sizeof(real_T) << 7U);
      kinsim_edo_B.In1.Effort_SL_Info.CurrentLength =
        b_varargout_2_Effort_SL_Info_Cu;
      kinsim_edo_B.In1.Effort_SL_Info.ReceivedLength =
        b_varargout_2_Effort_SL_Info_Re;
      kinsim_edo_B.In1.TimeFromStart.Sec = kinsim_edo_B.value;
      kinsim_edo_B.In1.TimeFromStart.Nsec = kinsim_edo_B.value_m;
    }

    // End of MATLABSystem: '<S10>/SourceBlock'
    // End of Outputs for SubSystem: '<S10>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe'
  }

  // Clock: '<Root>/Clock1' incorporates:
  //   Clock: '<Root>/Clock'

  kinsim_edo_B.value = kinsim_edo_M->Timing.t[0];

  // MATLAB Function: '<Root>/Assign to CartesianState msg' incorporates:
  //   Clock: '<Root>/Clock1'
  //   Constant: '<S4>/Constant'

  kinsim_edo_B.msg_d = kinsim_edo_P.Constant_Value;
  if (kinsim_edo_B.value < 0.0) {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    kinsim_edo_B.value_m = ceil(kinsim_edo_B.value);
  } else {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    kinsim_edo_B.value_m = floor(kinsim_edo_B.value);
  }

  kinsim_edo_B.msg_d.Header.Stamp.Sec = kinsim_edo_B.value_m;
  kinsim_edo_B.value_c = (kinsim_edo_B.value - kinsim_edo_B.value_m) * 1.0E+9;
  if (kinsim_edo_B.value_c < 0.0) {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    kinsim_edo_B.value_c = ceil(kinsim_edo_B.value_c);
  } else {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    kinsim_edo_B.value_c = floor(kinsim_edo_B.value_c);
  }

  kinsim_edo_B.msg_d.Header.Stamp.Nsec = kinsim_edo_B.value_c;
  kinsim_edo_B.msg_d.Name_SL_Info.CurrentLength = 2U;
  kinsim_edo_B.msg_d.Position_SL_Info.CurrentLength = 2U;
  kinsim_edo_B.msg_d.Velocity_SL_Info.CurrentLength = 2U;
  for (i = 0; i < 9; i++) {
    kinsim_edo_B.msg_d.Name[0].Data[i] = static_cast<uint8_T>(b[i]);
  }

  kinsim_edo_B.msg_d.Name[0].Data_SL_Info.CurrentLength = 9U;

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   Integrator: '<Root>/Integrator'

  kinsim_edo_B.value_k = cos(kinsim_edo_X.Integrator_CSTATE[0]);
  kinsim_edo_B.value_cx = kinsim_edo_X.Integrator_CSTATE[0] +
    kinsim_edo_X.Integrator_CSTATE[1];
  kinsim_edo_B.value_b = cos(kinsim_edo_B.value_cx);

  // MATLAB Function: '<Root>/Assign to CartesianState msg' incorporates:
  //   Constant: '<Root>/Constant'
  //   MATLAB Function: '<Root>/MATLAB Function'

  kinsim_edo_B.msg_d.Position[0] = kinsim_edo_P.Constant_Value_m[0] *
    kinsim_edo_B.value_k + kinsim_edo_P.Constant_Value_m[1] *
    kinsim_edo_B.value_b;

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   Constant: '<Root>/Constant'
  //   Integrator: '<Root>/Integrator'

  kinsim_edo_B.d = sin(kinsim_edo_X.Integrator_CSTATE[0]);
  kinsim_edo_B.value_cx = sin(kinsim_edo_B.value_cx);
  kinsim_edo_B.d1 = (kinsim_edo_B.In1.Velocities[0] +
                     kinsim_edo_B.In1.Velocities[1]) *
    kinsim_edo_P.Constant_Value_m[1];

  // MATLAB Function: '<Root>/Assign to CartesianState msg' incorporates:
  //   Constant: '<Root>/Constant'
  //   MATLAB Function: '<Root>/MATLAB Function'

  kinsim_edo_B.msg_d.Velocity[0] = -kinsim_edo_P.Constant_Value_m[0] *
    kinsim_edo_B.In1.Velocities[0] * kinsim_edo_B.d - kinsim_edo_B.d1 *
    kinsim_edo_B.value_cx;
  for (i = 0; i < 13; i++) {
    kinsim_edo_B.msg_d.Name[1].Data[i] = static_cast<uint8_T>(c[i]);
  }

  kinsim_edo_B.msg_d.Name[1].Data_SL_Info.CurrentLength = 13U;
  kinsim_edo_B.msg_d.Position[1] = kinsim_edo_P.Constant_Value_m[0] *
    kinsim_edo_B.d + kinsim_edo_P.Constant_Value_m[1] * kinsim_edo_B.value_cx;
  kinsim_edo_B.msg_d.Velocity[1] = kinsim_edo_B.d1 * kinsim_edo_B.value_b +
    kinsim_edo_P.Constant_Value_m[0] * kinsim_edo_B.In1.Velocities[0] *
    kinsim_edo_B.value_k;

  // Outputs for Atomic SubSystem: '<Root>/Publish2'
  // MATLABSystem: '<S9>/SinkBlock'
  Pub_kinsim_edo_79.publish(&kinsim_edo_B.msg_d);

  // End of Outputs for SubSystem: '<Root>/Publish2'

  // MATLAB Function: '<Root>/Assign to JointState msg' incorporates:
  //   Constant: '<S4>/Constant'
  //   Integrator: '<Root>/Integrator'

  kinsim_edo_B.msg_d = kinsim_edo_P.Constant_Value;
  kinsim_edo_B.msg_d.Header.Stamp.Sec = kinsim_edo_B.value_m;
  kinsim_edo_B.msg_d.Header.Stamp.Nsec = kinsim_edo_B.value_c;
  kinsim_edo_B.msg_d.Name_SL_Info.CurrentLength = 6U;
  kinsim_edo_B.msg_d.Position_SL_Info.CurrentLength = 6U;
  kinsim_edo_B.msg_d.Velocity_SL_Info.CurrentLength = 6U;
  kinsim_edo_B.msg_d.Name[0].Data_SL_Info.CurrentLength = 7U;
  kinsim_edo_B.msg_d.Position[0] = kinsim_edo_X.Integrator_CSTATE[0];
  kinsim_edo_B.msg_d.Velocity[0] = kinsim_edo_B.In1.Velocities[0];
  kinsim_edo_B.msg_d.Name[1].Data_SL_Info.CurrentLength = 7U;
  kinsim_edo_B.msg_d.Position[1] = kinsim_edo_X.Integrator_CSTATE[1];
  kinsim_edo_B.msg_d.Velocity[1] = kinsim_edo_B.In1.Velocities[1];
  kinsim_edo_B.msg_d.Name[2].Data_SL_Info.CurrentLength = 7U;
  kinsim_edo_B.msg_d.Position[2] = kinsim_edo_X.Integrator_CSTATE[2];
  kinsim_edo_B.msg_d.Velocity[2] = kinsim_edo_B.In1.Velocities[2];
  kinsim_edo_B.msg_d.Name[3].Data_SL_Info.CurrentLength = 7U;
  kinsim_edo_B.msg_d.Position[3] = kinsim_edo_X.Integrator_CSTATE[3];
  kinsim_edo_B.msg_d.Velocity[3] = kinsim_edo_B.In1.Velocities[3];
  kinsim_edo_B.msg_d.Name[4].Data_SL_Info.CurrentLength = 7U;
  kinsim_edo_B.msg_d.Position[4] = kinsim_edo_X.Integrator_CSTATE[4];
  kinsim_edo_B.msg_d.Velocity[4] = kinsim_edo_B.In1.Velocities[4];
  for (i = 0; i < 7; i++) {
    kinsim_edo_B.b.f1[i] = h[i];
    kinsim_edo_B.c.f1[i] = i_0[i];
    kinsim_edo_B.d_p.f1[i] = j[i];
    e.f1[i] = k[i];
    f.f1[i] = l[i];
    g.f1[i] = m[i];
    kinsim_edo_B.msg_d.Name[0].Data[i] = static_cast<uint8_T>
      (kinsim_edo_B.b.f1[i]);
    kinsim_edo_B.msg_d.Name[1].Data[i] = static_cast<uint8_T>
      (kinsim_edo_B.c.f1[i]);
    kinsim_edo_B.msg_d.Name[2].Data[i] = static_cast<uint8_T>
      (kinsim_edo_B.d_p.f1[i]);
    kinsim_edo_B.msg_d.Name[3].Data[i] = static_cast<uint8_T>(e.f1[i]);
    kinsim_edo_B.msg_d.Name[4].Data[i] = static_cast<uint8_T>(f.f1[i]);
    kinsim_edo_B.msg_d.Name[5].Data[i] = static_cast<uint8_T>(g.f1[i]);
  }

  kinsim_edo_B.msg_d.Name[5].Data_SL_Info.CurrentLength = 7U;
  kinsim_edo_B.msg_d.Position[5] = kinsim_edo_X.Integrator_CSTATE[5];
  kinsim_edo_B.msg_d.Velocity[5] = kinsim_edo_B.In1.Velocities[5];

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S7>/SinkBlock'
  Pub_kinsim_edo_22.publish(&kinsim_edo_B.msg_d);

  // End of Outputs for SubSystem: '<Root>/Publish'

  // MATLAB Function: '<Root>/Assign to Time msg'
  if (kinsim_edo_B.value < 0.0) {
    kinsim_edo_B.value_m = ceil(kinsim_edo_B.value);
  } else {
    kinsim_edo_B.value_m = floor(kinsim_edo_B.value);
  }

  kinsim_edo_B.msg_l.Clock_.Sec = kinsim_edo_B.value_m;
  kinsim_edo_B.value_c = (kinsim_edo_B.value - kinsim_edo_B.value_m) * 1.0E+9;
  if (kinsim_edo_B.value_c < 0.0) {
    kinsim_edo_B.msg_l.Clock_.Nsec = ceil(kinsim_edo_B.value_c);
  } else {
    kinsim_edo_B.msg_l.Clock_.Nsec = floor(kinsim_edo_B.value_c);
  }

  // End of MATLAB Function: '<Root>/Assign to Time msg'

  // Outputs for Atomic SubSystem: '<Root>/Publish1'
  // MATLABSystem: '<S8>/SinkBlock'
  Pub_kinsim_edo_50.publish(&kinsim_edo_B.msg_l);

  // End of Outputs for SubSystem: '<Root>/Publish1'
  if (rtmIsMajorTimeStep(kinsim_edo_M)) {
    // Update for Integrator: '<Root>/Integrator'
    kinsim_edo_DW.Integrator_IWORK = 0;
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep(kinsim_edo_M)) {
    rt_ertODEUpdateContinuousStates(&kinsim_edo_M->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++kinsim_edo_M->Timing.clockTick0;
    kinsim_edo_M->Timing.t[0] = rtsiGetSolverStopTime(&kinsim_edo_M->solverInfo);

    {
      // Update absolute timer for sample time: [0.001s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.001, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      kinsim_edo_M->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void kinsim_edo_derivatives(void)
{
  int32_T i;
  XDot_kinsim_edo_T *_rtXdot;
  _rtXdot = ((XDot_kinsim_edo_T *) kinsim_edo_M->derivs);

  // Derivatives for Integrator: '<Root>/Integrator'
  for (i = 0; i < 6; i++) {
    _rtXdot->Integrator_CSTATE[i] = kinsim_edo_B.In1.Velocities[i];
  }

  // End of Derivatives for Integrator: '<Root>/Integrator'
}

// Model initialize function
void kinsim_edo_initialize(void)
{
  // Registration code
  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&kinsim_edo_M->solverInfo,
                          &kinsim_edo_M->Timing.simTimeStep);
    rtsiSetTPtr(&kinsim_edo_M->solverInfo, &rtmGetTPtr(kinsim_edo_M));
    rtsiSetStepSizePtr(&kinsim_edo_M->solverInfo,
                       &kinsim_edo_M->Timing.stepSize0);
    rtsiSetdXPtr(&kinsim_edo_M->solverInfo, &kinsim_edo_M->derivs);
    rtsiSetContStatesPtr(&kinsim_edo_M->solverInfo, (real_T **)
                         &kinsim_edo_M->contStates);
    rtsiSetNumContStatesPtr(&kinsim_edo_M->solverInfo,
      &kinsim_edo_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&kinsim_edo_M->solverInfo,
      &kinsim_edo_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&kinsim_edo_M->solverInfo,
      &kinsim_edo_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&kinsim_edo_M->solverInfo,
      &kinsim_edo_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&kinsim_edo_M->solverInfo, (&rtmGetErrorStatus
      (kinsim_edo_M)));
    rtsiSetRTModelPtr(&kinsim_edo_M->solverInfo, kinsim_edo_M);
  }

  rtsiSetSimTimeStep(&kinsim_edo_M->solverInfo, MAJOR_TIME_STEP);
  kinsim_edo_M->intgData.y = kinsim_edo_M->odeY;
  kinsim_edo_M->intgData.f[0] = kinsim_edo_M->odeF[0];
  kinsim_edo_M->intgData.f[1] = kinsim_edo_M->odeF[1];
  kinsim_edo_M->intgData.f[2] = kinsim_edo_M->odeF[2];
  kinsim_edo_M->contStates = ((X_kinsim_edo_T *) &kinsim_edo_X);
  rtsiSetSolverData(&kinsim_edo_M->solverInfo, static_cast<void *>
                    (&kinsim_edo_M->intgData));
  rtsiSetSolverName(&kinsim_edo_M->solverInfo,"ode3");
  rtmSetTPtr(kinsim_edo_M, &kinsim_edo_M->Timing.tArray[0]);
  kinsim_edo_M->Timing.stepSize0 = 0.001;
  rtmSetFirstInitCond(kinsim_edo_M, 1);

  {
    char_T tmp[18];
    char_T tmp_0[14];
    char_T tmp_1[7];
    int32_T i;
    static const char_T tmp_2[17] = { '/', 'j', 'o', 'i', 'n', 't', '_', 't',
      'r', 'a', 'j', 'e', 'c', 't', 'o', 'r', 'y' };

    static const char_T tmp_3[17] = { '/', 'c', 'a', 'r', 't', 'e', 's', 'i',
      'a', 'n', '_', 's', 't', 'a', 't', 'e', 's' };

    static const char_T tmp_4[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 's',
      't', 'a', 't', 'e', 's' };

    static const char_T tmp_5[6] = { '/', 'c', 'l', 'o', 'c', 'k' };

    static const char_T tmp_6[22] = { '/', 'k', 'i', 'n', 's', 'i', 'm', '_',
      'e', 'd', 'o', '/', 'q', '1', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_7[22] = { '/', 'k', 'i', 'n', 's', 'i', 'm', '_',
      'e', 'd', 'o', '/', 'q', '2', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_8[22] = { '/', 'k', 'i', 'n', 's', 'i', 'm', '_',
      'e', 'd', 'o', '/', 'q', '3', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_9[22] = { '/', 'k', 'i', 'n', 's', 'i', 'm', '_',
      'e', 'd', 'o', '/', 'q', '4', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_a[22] = { '/', 'k', 'i', 'n', 's', 'i', 'm', '_',
      'e', 'd', 'o', '/', 'q', '5', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_b[22] = { '/', 'k', 'i', 'n', 's', 'i', 'm', '_',
      'e', 'd', 'o', '/', 'q', '6', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    // InitializeConditions for Integrator: '<Root>/Integrator'
    if (rtmIsFirstInitCond(kinsim_edo_M)) {
      kinsim_edo_X.Integrator_CSTATE[0] = 0.0;
      kinsim_edo_X.Integrator_CSTATE[1] = 0.0;
      kinsim_edo_X.Integrator_CSTATE[2] = 0.0;
      kinsim_edo_X.Integrator_CSTATE[3] = 0.0;
      kinsim_edo_X.Integrator_CSTATE[4] = 0.0;
      kinsim_edo_X.Integrator_CSTATE[5] = 0.0;
    }

    kinsim_edo_DW.Integrator_IWORK = 1;

    // End of InitializeConditions for Integrator: '<Root>/Integrator'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S10>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S12>/Out1'
    kinsim_edo_B.In1 = kinsim_edo_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S10>/Enabled Subsystem'

    // Start for MATLABSystem: '<S10>/SourceBlock'
    kinsim_edo_DW.obj_p.matlabCodegenIsDeleted = false;
    kinsim_edo_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      tmp[i] = tmp_2[i];
    }

    tmp[17] = '\x00';
    Sub_kinsim_edo_16.createSubscriber(tmp, 1);
    kinsim_edo_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish2'
    // Start for MATLABSystem: '<S9>/SinkBlock'
    kinsim_edo_DW.obj_m.matlabCodegenIsDeleted = false;
    kinsim_edo_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      tmp[i] = tmp_3[i];
    }

    tmp[17] = '\x00';
    Pub_kinsim_edo_79.createPublisher(tmp, 1);
    kinsim_edo_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish2'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S7>/SinkBlock'
    kinsim_edo_DW.obj_nr.matlabCodegenIsDeleted = false;
    kinsim_edo_DW.obj_nr.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp_0[i] = tmp_4[i];
    }

    tmp_0[13] = '\x00';
    Pub_kinsim_edo_22.createPublisher(tmp_0, 1);
    kinsim_edo_DW.obj_nr.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S8>/SinkBlock'
    kinsim_edo_DW.obj_f.matlabCodegenIsDeleted = false;
    kinsim_edo_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 6; i++) {
      tmp_1[i] = tmp_5[i];
    }

    tmp_1[6] = '\x00';
    Pub_kinsim_edo_50.createPublisher(tmp_1, 1);
    kinsim_edo_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish1'

    // Start for MATLABSystem: '<S11>/Get Parameter'
    kinsim_edo_DW.obj.matlabCodegenIsDeleted = false;
    kinsim_edo_DW.obj.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      kinsim_edo_B.cv[i] = tmp_6[i];
    }

    kinsim_edo_B.cv[22] = '\x00';
    ParamGet_kinsim_edo_61.initialize(kinsim_edo_B.cv);
    ParamGet_kinsim_edo_61.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_edo_61.set_initial_value(0.78539816339744828);
    kinsim_edo_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter'

    // Start for MATLABSystem: '<S11>/Get Parameter1'
    kinsim_edo_DW.obj_n.matlabCodegenIsDeleted = false;
    kinsim_edo_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      kinsim_edo_B.cv[i] = tmp_7[i];
    }

    kinsim_edo_B.cv[22] = '\x00';
    ParamGet_kinsim_edo_65.initialize(kinsim_edo_B.cv);
    ParamGet_kinsim_edo_65.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_edo_65.set_initial_value(-0.78539816339744828);
    kinsim_edo_DW.obj_n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter1'

    // Start for MATLABSystem: '<S11>/Get Parameter5'
    kinsim_edo_DW.obj_o.matlabCodegenIsDeleted = false;
    kinsim_edo_DW.obj_o.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      kinsim_edo_B.cv[i] = tmp_8[i];
    }

    kinsim_edo_B.cv[22] = '\x00';
    ParamGet_kinsim_edo_84.initialize(kinsim_edo_B.cv);
    ParamGet_kinsim_edo_84.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_edo_84.set_initial_value(0.78539816339744828);
    kinsim_edo_DW.obj_o.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter5'

    // Start for MATLABSystem: '<S11>/Get Parameter4'
    kinsim_edo_DW.obj_b.matlabCodegenIsDeleted = false;
    kinsim_edo_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      kinsim_edo_B.cv[i] = tmp_9[i];
    }

    kinsim_edo_B.cv[22] = '\x00';
    ParamGet_kinsim_edo_83.initialize(kinsim_edo_B.cv);
    ParamGet_kinsim_edo_83.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_edo_83.set_initial_value(0.78539816339744828);
    kinsim_edo_DW.obj_b.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter4'

    // Start for MATLABSystem: '<S11>/Get Parameter3'
    kinsim_edo_DW.obj_d.matlabCodegenIsDeleted = false;
    kinsim_edo_DW.obj_d.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      kinsim_edo_B.cv[i] = tmp_a[i];
    }

    kinsim_edo_B.cv[22] = '\x00';
    ParamGet_kinsim_edo_82.initialize(kinsim_edo_B.cv);
    ParamGet_kinsim_edo_82.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_edo_82.set_initial_value(0.78539816339744828);
    kinsim_edo_DW.obj_d.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter3'

    // Start for MATLABSystem: '<S11>/Get Parameter2'
    kinsim_edo_DW.obj_i.matlabCodegenIsDeleted = false;
    kinsim_edo_DW.obj_i.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      kinsim_edo_B.cv[i] = tmp_b[i];
    }

    kinsim_edo_B.cv[22] = '\x00';
    ParamGet_kinsim_edo_81.initialize(kinsim_edo_B.cv);
    ParamGet_kinsim_edo_81.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_edo_81.set_initial_value(0.78539816339744828);
    kinsim_edo_DW.obj_i.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter2'
  }

  // set "at time zero" to false
  if (rtmIsFirstInitCond(kinsim_edo_M)) {
    rtmSetFirstInitCond(kinsim_edo_M, 0);
  }
}

// Model terminate function
void kinsim_edo_terminate(void)
{
  // Terminate for MATLABSystem: '<S11>/Get Parameter'
  matlabCodegenHandle_matlab_pcdk(&kinsim_edo_DW.obj);

  // Terminate for MATLABSystem: '<S11>/Get Parameter1'
  matlabCodegenHandle_matlab_pcdk(&kinsim_edo_DW.obj_n);

  // Terminate for MATLABSystem: '<S11>/Get Parameter5'
  matlabCodegenHandle_matlab_pcdk(&kinsim_edo_DW.obj_o);

  // Terminate for MATLABSystem: '<S11>/Get Parameter4'
  matlabCodegenHandle_matlab_pcdk(&kinsim_edo_DW.obj_b);

  // Terminate for MATLABSystem: '<S11>/Get Parameter3'
  matlabCodegenHandle_matlab_pcdk(&kinsim_edo_DW.obj_d);

  // Terminate for MATLABSystem: '<S11>/Get Parameter2'
  matlabCodegenHandle_matlab_pcdk(&kinsim_edo_DW.obj_i);

  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S10>/SourceBlock'
  matlabCodegenHandle_matlabC_pcd(&kinsim_edo_DW.obj_p);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Atomic SubSystem: '<Root>/Publish2'
  // Terminate for MATLABSystem: '<S9>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&kinsim_edo_DW.obj_m);

  // End of Terminate for SubSystem: '<Root>/Publish2'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S7>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&kinsim_edo_DW.obj_nr);

  // End of Terminate for SubSystem: '<Root>/Publish'

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S8>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&kinsim_edo_DW.obj_f);

  // End of Terminate for SubSystem: '<Root>/Publish1'
}

//
// File trailer for generated code.
//
// [EOF]
//
