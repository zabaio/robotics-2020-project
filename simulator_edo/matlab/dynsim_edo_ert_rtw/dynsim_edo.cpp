//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: dynsim_edo.cpp
//
// Code generated for Simulink model 'dynsim_edo'.
//
// Model version                  : 1.123
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Sun May 24 20:24:55 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "dynsim_edo.h"
#include "dynsim_edo_private.h"

// Block signals (default storage)
B_dynsim_edo_T dynsim_edo_B;

// Continuous states
X_dynsim_edo_T dynsim_edo_X;

// Block states (default storage)
DW_dynsim_edo_T dynsim_edo_DW;

// Real-time model
RT_MODEL_dynsim_edo_T dynsim_edo_M_ = RT_MODEL_dynsim_edo_T();
RT_MODEL_dynsim_edo_T *const dynsim_edo_M = &dynsim_edo_M_;

// Forward declaration for local functions
static void dynsim_edo_emxInit_real_T(emxArray_real_T_dynsim_edo_T **pEmxArray,
  int32_T numDimensions);
static void dynsim_edo_emxInit_f_cell_wrap(emxArray_f_cell_wrap_dynsim_e_T
  **pEmxArray, int32_T numDimensions);
static void dynsim_edo_emxInit_char_T(emxArray_char_T_dynsim_edo_T **pEmxArray,
  int32_T numDimensions);
static void d_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_dynsim_e_T
  *emxArray, int32_T oldNumel);
static void dynsim_emxEnsureCapacity_char_T(emxArray_char_T_dynsim_edo_T
  *emxArray, int32_T oldNumel);
static void dy_rigidBodyJoint_get_JointAxis(const c_rigidBodyJoint_dynsim_edo_T *
  obj, real_T ax[3]);
static void dynsim_edo_emxFree_char_T(emxArray_char_T_dynsim_edo_T **pEmxArray);
static void RigidBodyTree_forwardKinematics(n_robotics_manip_internal_Rig_T *obj,
  const real_T qvec[3], emxArray_f_cell_wrap_dynsim_e_T *Ttree);
static void dynsim_emxEnsureCapacity_real_T(emxArray_real_T_dynsim_edo_T
  *emxArray, int32_T oldNumel);
static void dynsim_edo_emxFree_real_T(emxArray_real_T_dynsim_edo_T **pEmxArray);
static void dynsim_edo_emxFree_f_cell_wrap(emxArray_f_cell_wrap_dynsim_e_T
  **pEmxArray);
static void RigidBodyTree_geometricJacobian(n_robotics_manip_internal_Rig_T *obj,
  const real_T Q[3], emxArray_real_T_dynsim_edo_T *Jac);
static void rigidBodyJoint_get_JointAxis_h(const c_rigidBodyJoint_dynsim_edo_h_T
  *obj, real_T ax[3]);
static void RigidBodyTree_forwardKinemati_h(n_robotics_manip_internal_R_h_T *obj,
  const real_T qvec[3], emxArray_f_cell_wrap_dynsim_e_T *Ttree);
static void dynsim_edo_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[128], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_dynsim_edo_std_msgs_MultiArrayDimension varargout_2_Layout_Dim[16],
  uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0);
static void dynsim_edo_emxInit_f_cell_wrap1(emxArray_f_cell_wrap_dynsim_h_T
  **pEmxArray, int32_T numDimensions);
static void emxEnsureCapacity_f_cell_wrap1(emxArray_f_cell_wrap_dynsim_h_T
  *emxArray, int32_T oldNumel);
static void rigidBodyJoint_get_JointAxis_ha(const
  c_rigidBodyJoint_dynsim_ed_ha_T *obj, real_T ax[3]);
static void dynsim_edo_cat(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9]);
static void rigidBodyJoint_transformBodyT_h(const
  c_rigidBodyJoint_dynsim_ed_ha_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16]);
static void rigidBodyJoint_transformBodyToP(const
  c_rigidBodyJoint_dynsim_ed_ha_T *obj, real_T T[16]);
static void dynsim_edo_tforminv(const real_T T[16], real_T Tinv[16]);
static void dynsim_edo_tformToSpatialXform(const real_T T[16], real_T X[36]);
static void dynsim_edo_emxFree_f_cell_wrap1(emxArray_f_cell_wrap_dynsim_h_T
  **pEmxArray);
static void RigidBodyTreeDynamics_massMatri(n_robotics_manip_internal__ha_T
  *robot, const real_T q[3], emxArray_real_T_dynsim_edo_T *H,
  emxArray_real_T_dynsim_edo_T *lambda);
static void RigidBodyTreeDynamics_inverseDy(n_robotics_manip_internal__ha_T
  *robot, const real_T q[3], const real_T qdot[3], const real_T fext[36], real_T
  tau[3]);
static void matlabCodegenHandle_matlab_hau4(ros_slros_internal_block_GetP_T *obj);
static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynsim_edo_T
  *pStruct);
static void emxFreeStruct_m_robotics_manip_(m_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxFreeStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_dynsim_edo_h_T
  *pStruct);
static void emxFreeStruct_m_robotics_mani_h(m_robotics_manip_internal_R_h_T
  *pStruct);
static void emxFreeStruct_n_robotics_mani_h(n_robotics_manip_internal_R_h_T
  *pStruct);
static void emxFreeStruct_robotics_slmani_h(robotics_slmanip_internal_b_h_T
  *pStruct);
static void emxFreeStruct_l_robotics_mani_h(l_robotics_manip_internal_R_h_T
  *pStruct);
static void matlabCodegenHandle_matlabC_hau(ros_slros_internal_block_Subs_T *obj);
static void emxFreeStruct_c_rigidBodyJoint2(c_rigidBodyJoint_dynsim_ed_ha_T
  *pStruct);
static void emxFreeStruct_m_robotics_man_ha(m_robotics_manip_internal__ha_T
  *pStruct);
static void emxFreeStruct_n_robotics_man_ha(n_robotics_manip_internal__ha_T
  *pStruct);
static void emxFreeStruct_robotics_slman_ha(robotics_slmanip_internal__ha_T
  *pStruct);
static void emxFreeStruct_l_robotics_man_ha(l_robotics_manip_internal__ha_T
  *pStruct);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj);
static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynsim_edo_T
  *pStruct);
static void emxInitStruct_m_robotics_manip_(m_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxInitStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct);
static l_robotics_manip_internal_Rig_T *dynsim_edo_RigidBody_RigidBody
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *dynsim_ed_RigidBody_RigidBody_h
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *dynsim_e_RigidBody_RigidBody_ha
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *dynsim__RigidBody_RigidBody_hau
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *dynsim_RigidBody_RigidBody_hau4
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *dynsi_RigidBody_RigidBody_hau4i
  (l_robotics_manip_internal_Rig_T *obj);
static m_robotics_manip_internal_Rig_T *dyns_RigidBody_RigidBody_hau4ih
  (m_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *dyn_RigidBodyTree_RigidBodyTree
  (n_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Rig_T *iobj_0,
   l_robotics_manip_internal_Rig_T *iobj_1, l_robotics_manip_internal_Rig_T
   *iobj_2, l_robotics_manip_internal_Rig_T *iobj_3,
   l_robotics_manip_internal_Rig_T *iobj_4, l_robotics_manip_internal_Rig_T
   *iobj_5);
static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_dynsim_edo_h_T
  *pStruct);
static void emxInitStruct_m_robotics_mani_h(m_robotics_manip_internal_R_h_T
  *pStruct);
static void emxInitStruct_n_robotics_mani_h(n_robotics_manip_internal_R_h_T
  *pStruct);
static void emxInitStruct_robotics_slmani_h(robotics_slmanip_internal_b_h_T
  *pStruct);
static void emxInitStruct_l_robotics_mani_h(l_robotics_manip_internal_R_h_T
  *pStruct);
static l_robotics_manip_internal_R_h_T *dyn_RigidBody_RigidBody_hau4ihh
  (l_robotics_manip_internal_R_h_T *obj);
static l_robotics_manip_internal_R_h_T *dy_RigidBody_RigidBody_hau4ihh0
  (l_robotics_manip_internal_R_h_T *obj);
static l_robotics_manip_internal_R_h_T *d_RigidBody_RigidBody_hau4ihh0a
  (l_robotics_manip_internal_R_h_T *obj);
static n_robotics_manip_internal_R_h_T *d_RigidBodyTree_RigidBodyTree_h
  (n_robotics_manip_internal_R_h_T *obj, l_robotics_manip_internal_R_h_T *iobj_0,
   l_robotics_manip_internal_R_h_T *iobj_1, l_robotics_manip_internal_R_h_T
   *iobj_2, l_robotics_manip_internal_R_h_T *iobj_3,
   l_robotics_manip_internal_R_h_T *iobj_4, l_robotics_manip_internal_R_h_T
   *iobj_5);
static void emxInitStruct_c_rigidBodyJoint2(c_rigidBodyJoint_dynsim_ed_ha_T
  *pStruct);
static void emxInitStruct_m_robotics_man_ha(m_robotics_manip_internal__ha_T
  *pStruct);
static void emxInitStruct_n_robotics_man_ha(n_robotics_manip_internal__ha_T
  *pStruct);
static void emxInitStruct_robotics_slman_ha(robotics_slmanip_internal__ha_T
  *pStruct);
static void emxInitStruct_l_robotics_man_ha(l_robotics_manip_internal__ha_T
  *pStruct);
static l_robotics_manip_internal__ha_T *RigidBody_RigidBody_hau4ihh0a3
  (l_robotics_manip_internal__ha_T *obj);
static l_robotics_manip_internal__ha_T *RigidBody_RigidBody_hau4ihh0a3c
  (l_robotics_manip_internal__ha_T *obj);
static l_robotics_manip_internal__ha_T *RigidBody_RigidBod_hau4ihh0a3ct
  (l_robotics_manip_internal__ha_T *obj);
static l_robotics_manip_internal__ha_T *RigidBody_RigidBo_hau4ihh0a3ctk
  (l_robotics_manip_internal__ha_T *obj);
static l_robotics_manip_internal__ha_T *RigidBody_RigidB_hau4ihh0a3ctk5
  (l_robotics_manip_internal__ha_T *obj);
static l_robotics_manip_internal__ha_T *RigidBody_Rigid_hau4ihh0a3ctk52
  (l_robotics_manip_internal__ha_T *obj);
static m_robotics_manip_internal__ha_T *d_RigidBody_Rigid_c
  (m_robotics_manip_internal__ha_T *obj);
static n_robotics_manip_internal__ha_T *RigidBodyTree_RigidBodyTree_ha
  (n_robotics_manip_internal__ha_T *obj, l_robotics_manip_internal__ha_T *iobj_0,
   l_robotics_manip_internal__ha_T *iobj_1, l_robotics_manip_internal__ha_T
   *iobj_2, l_robotics_manip_internal__ha_T *iobj_3,
   l_robotics_manip_internal__ha_T *iobj_4, l_robotics_manip_internal__ha_T
   *iobj_5);

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
  dynsim_edo_derivatives();

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  dynsim_edo_step();
  dynsim_edo_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  dynsim_edo_step();
  dynsim_edo_derivatives();

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

static void dynsim_edo_emxInit_real_T(emxArray_real_T_dynsim_edo_T **pEmxArray,
  int32_T numDimensions)
{
  emxArray_real_T_dynsim_edo_T *emxArray;
  *pEmxArray = (emxArray_real_T_dynsim_edo_T *)malloc(sizeof
    (emxArray_real_T_dynsim_edo_T));
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (dynsim_edo_B.i_j = 0; dynsim_edo_B.i_j < numDimensions; dynsim_edo_B.i_j
       ++) {
    emxArray->size[dynsim_edo_B.i_j] = 0;
  }
}

static void dynsim_edo_emxInit_f_cell_wrap(emxArray_f_cell_wrap_dynsim_e_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_cell_wrap_dynsim_e_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_f_cell_wrap_dynsim_e_T *)malloc(sizeof
    (emxArray_f_cell_wrap_dynsim_e_T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_dynsim_edo_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void dynsim_edo_emxInit_char_T(emxArray_char_T_dynsim_edo_T **pEmxArray,
  int32_T numDimensions)
{
  emxArray_char_T_dynsim_edo_T *emxArray;
  *pEmxArray = (emxArray_char_T_dynsim_edo_T *)malloc(sizeof
    (emxArray_char_T_dynsim_edo_T));
  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (dynsim_edo_B.i_m = 0; dynsim_edo_B.i_m < numDimensions; dynsim_edo_B.i_m
       ++) {
    emxArray->size[dynsim_edo_B.i_m] = 0;
  }
}

static void d_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_dynsim_e_T
  *emxArray, int32_T oldNumel)
{
  int32_T newNumel;
  int32_T i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(i), sizeof(f_cell_wrap_dynsim_edo_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_dynsim_edo_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_dynsim_edo_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void dynsim_emxEnsureCapacity_char_T(emxArray_char_T_dynsim_edo_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  dynsim_edo_B.newNumel_b = 1;
  for (dynsim_edo_B.i_c = 0; dynsim_edo_B.i_c < emxArray->numDimensions;
       dynsim_edo_B.i_c++) {
    dynsim_edo_B.newNumel_b *= emxArray->size[dynsim_edo_B.i_c];
  }

  if (dynsim_edo_B.newNumel_b > emxArray->allocatedSize) {
    dynsim_edo_B.i_c = emxArray->allocatedSize;
    if (dynsim_edo_B.i_c < 16) {
      dynsim_edo_B.i_c = 16;
    }

    while (dynsim_edo_B.i_c < dynsim_edo_B.newNumel_b) {
      if (dynsim_edo_B.i_c > 1073741823) {
        dynsim_edo_B.i_c = MAX_int32_T;
      } else {
        dynsim_edo_B.i_c <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(dynsim_edo_B.i_c), sizeof(char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = dynsim_edo_B.i_c;
    emxArray->canFreeData = true;
  }
}

static void dy_rigidBodyJoint_get_JointAxis(const c_rigidBodyJoint_dynsim_edo_T *
  obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (dynsim_edo_B.b_kstr_e = 0; dynsim_edo_B.b_kstr_e < 8;
       dynsim_edo_B.b_kstr_e++) {
    dynsim_edo_B.b_j[dynsim_edo_B.b_kstr_e] = tmp[dynsim_edo_B.b_kstr_e];
  }

  dynsim_edo_B.b_bool_n = false;
  if (obj->Type->size[1] == 8) {
    dynsim_edo_B.b_kstr_e = 1;
    do {
      exitg1 = 0;
      if (dynsim_edo_B.b_kstr_e - 1 < 8) {
        dynsim_edo_B.kstr_g = dynsim_edo_B.b_kstr_e - 1;
        if (obj->Type->data[dynsim_edo_B.kstr_g] !=
            dynsim_edo_B.b_j[dynsim_edo_B.kstr_g]) {
          exitg1 = 1;
        } else {
          dynsim_edo_B.b_kstr_e++;
        }
      } else {
        dynsim_edo_B.b_bool_n = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (dynsim_edo_B.b_bool_n) {
    guard1 = true;
  } else {
    for (dynsim_edo_B.b_kstr_e = 0; dynsim_edo_B.b_kstr_e < 9;
         dynsim_edo_B.b_kstr_e++) {
      dynsim_edo_B.b_n[dynsim_edo_B.b_kstr_e] = tmp_0[dynsim_edo_B.b_kstr_e];
    }

    dynsim_edo_B.b_bool_n = false;
    if (obj->Type->size[1] == 9) {
      dynsim_edo_B.b_kstr_e = 1;
      do {
        exitg1 = 0;
        if (dynsim_edo_B.b_kstr_e - 1 < 9) {
          dynsim_edo_B.kstr_g = dynsim_edo_B.b_kstr_e - 1;
          if (obj->Type->data[dynsim_edo_B.kstr_g] !=
              dynsim_edo_B.b_n[dynsim_edo_B.kstr_g]) {
            exitg1 = 1;
          } else {
            dynsim_edo_B.b_kstr_e++;
          }
        } else {
          dynsim_edo_B.b_bool_n = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_edo_B.b_bool_n) {
      guard1 = true;
    } else {
      ax[0] = (rtNaN);
      ax[1] = (rtNaN);
      ax[2] = (rtNaN);
    }
  }

  if (guard1) {
    ax[0] = obj->JointAxisInternal[0];
    ax[1] = obj->JointAxisInternal[1];
    ax[2] = obj->JointAxisInternal[2];
  }
}

static void dynsim_edo_emxFree_char_T(emxArray_char_T_dynsim_edo_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T_dynsim_edo_T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_char_T_dynsim_edo_T *)NULL;
  }
}

static void RigidBodyTree_forwardKinematics(n_robotics_manip_internal_Rig_T *obj,
  const real_T qvec[3], emxArray_f_cell_wrap_dynsim_e_T *Ttree)
{
  l_robotics_manip_internal_Rig_T *body;
  emxArray_char_T_dynsim_edo_T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  dynsim_edo_B.n = obj->NumBodies;
  for (dynsim_edo_B.b_kstr = 0; dynsim_edo_B.b_kstr < 16; dynsim_edo_B.b_kstr++)
  {
    dynsim_edo_B.c_f1[dynsim_edo_B.b_kstr] = tmp[dynsim_edo_B.b_kstr];
  }

  dynsim_edo_B.b_kstr = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  dynsim_edo_B.e = static_cast<int32_T>(dynsim_edo_B.n);
  Ttree->size[1] = dynsim_edo_B.e;
  d_emxEnsureCapacity_f_cell_wrap(Ttree, dynsim_edo_B.b_kstr);
  if (dynsim_edo_B.e != 0) {
    dynsim_edo_B.ntilecols = dynsim_edo_B.e - 1;
    if (0 <= dynsim_edo_B.ntilecols) {
      memcpy(&dynsim_edo_B.expl_temp.f1[0], &dynsim_edo_B.c_f1[0], sizeof(real_T)
             << 4U);
    }

    for (dynsim_edo_B.b_jtilecol = 0; dynsim_edo_B.b_jtilecol <=
         dynsim_edo_B.ntilecols; dynsim_edo_B.b_jtilecol++) {
      Ttree->data[dynsim_edo_B.b_jtilecol] = dynsim_edo_B.expl_temp;
    }
  }

  dynsim_edo_B.k_a = 1.0;
  dynsim_edo_B.ntilecols = static_cast<int32_T>(dynsim_edo_B.n) - 1;
  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  if (0 <= dynsim_edo_B.ntilecols) {
    for (dynsim_edo_B.b_kstr = 0; dynsim_edo_B.b_kstr < 5; dynsim_edo_B.b_kstr++)
    {
      dynsim_edo_B.b_mc[dynsim_edo_B.b_kstr] = tmp_0[dynsim_edo_B.b_kstr];
    }
  }

  for (dynsim_edo_B.b_jtilecol = 0; dynsim_edo_B.b_jtilecol <=
       dynsim_edo_B.ntilecols; dynsim_edo_B.b_jtilecol++) {
    body = obj->Bodies[dynsim_edo_B.b_jtilecol];
    dynsim_edo_B.n = body->JointInternal.PositionNumber;
    dynsim_edo_B.n += dynsim_edo_B.k_a;
    if (dynsim_edo_B.k_a > dynsim_edo_B.n - 1.0) {
      dynsim_edo_B.e = 0;
      dynsim_edo_B.d_b = 0;
    } else {
      dynsim_edo_B.e = static_cast<int32_T>(dynsim_edo_B.k_a) - 1;
      dynsim_edo_B.d_b = static_cast<int32_T>(dynsim_edo_B.n - 1.0);
    }

    dynsim_edo_B.b_kstr = switch_expression->size[0] * switch_expression->size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    dynsim_emxEnsureCapacity_char_T(switch_expression, dynsim_edo_B.b_kstr);
    dynsim_edo_B.loop_ub_a = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (dynsim_edo_B.b_kstr = 0; dynsim_edo_B.b_kstr <= dynsim_edo_B.loop_ub_a;
         dynsim_edo_B.b_kstr++) {
      switch_expression->data[dynsim_edo_B.b_kstr] = body->
        JointInternal.Type->data[dynsim_edo_B.b_kstr];
    }

    dynsim_edo_B.b_bool_g = false;
    if (switch_expression->size[1] == 5) {
      dynsim_edo_B.b_kstr = 1;
      do {
        exitg1 = 0;
        if (dynsim_edo_B.b_kstr - 1 < 5) {
          dynsim_edo_B.loop_ub_a = dynsim_edo_B.b_kstr - 1;
          if (switch_expression->data[dynsim_edo_B.loop_ub_a] !=
              dynsim_edo_B.b_mc[dynsim_edo_B.loop_ub_a]) {
            exitg1 = 1;
          } else {
            dynsim_edo_B.b_kstr++;
          }
        } else {
          dynsim_edo_B.b_bool_g = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_edo_B.b_bool_g) {
      dynsim_edo_B.b_kstr = 0;
    } else {
      for (dynsim_edo_B.b_kstr = 0; dynsim_edo_B.b_kstr < 8; dynsim_edo_B.b_kstr
           ++) {
        dynsim_edo_B.b_m3[dynsim_edo_B.b_kstr] = tmp_1[dynsim_edo_B.b_kstr];
      }

      dynsim_edo_B.b_bool_g = false;
      if (switch_expression->size[1] == 8) {
        dynsim_edo_B.b_kstr = 1;
        do {
          exitg1 = 0;
          if (dynsim_edo_B.b_kstr - 1 < 8) {
            dynsim_edo_B.loop_ub_a = dynsim_edo_B.b_kstr - 1;
            if (switch_expression->data[dynsim_edo_B.loop_ub_a] !=
                dynsim_edo_B.b_m3[dynsim_edo_B.loop_ub_a]) {
              exitg1 = 1;
            } else {
              dynsim_edo_B.b_kstr++;
            }
          } else {
            dynsim_edo_B.b_bool_g = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (dynsim_edo_B.b_bool_g) {
        dynsim_edo_B.b_kstr = 1;
      } else {
        dynsim_edo_B.b_kstr = -1;
      }
    }

    switch (dynsim_edo_B.b_kstr) {
     case 0:
      memset(&dynsim_edo_B.c_f1[0], 0, sizeof(real_T) << 4U);
      dynsim_edo_B.c_f1[0] = 1.0;
      dynsim_edo_B.c_f1[5] = 1.0;
      dynsim_edo_B.c_f1[10] = 1.0;
      dynsim_edo_B.c_f1[15] = 1.0;
      break;

     case 1:
      dy_rigidBodyJoint_get_JointAxis(&body->JointInternal, dynsim_edo_B.v);
      dynsim_edo_B.d_b -= dynsim_edo_B.e;
      for (dynsim_edo_B.b_kstr = 0; dynsim_edo_B.b_kstr < dynsim_edo_B.d_b;
           dynsim_edo_B.b_kstr++) {
        dynsim_edo_B.e_data[dynsim_edo_B.b_kstr] = dynsim_edo_B.e +
          dynsim_edo_B.b_kstr;
      }

      dynsim_edo_B.result_data[0] = dynsim_edo_B.v[0];
      dynsim_edo_B.result_data[1] = dynsim_edo_B.v[1];
      dynsim_edo_B.result_data[2] = dynsim_edo_B.v[2];
      if (0 <= (dynsim_edo_B.d_b != 0) - 1) {
        dynsim_edo_B.result_data[3] = qvec[dynsim_edo_B.e_data[0]];
      }

      dynsim_edo_B.k_a = 1.0 / sqrt((dynsim_edo_B.result_data[0] *
        dynsim_edo_B.result_data[0] + dynsim_edo_B.result_data[1] *
        dynsim_edo_B.result_data[1]) + dynsim_edo_B.result_data[2] *
        dynsim_edo_B.result_data[2]);
      dynsim_edo_B.v[0] = dynsim_edo_B.result_data[0] * dynsim_edo_B.k_a;
      dynsim_edo_B.v[1] = dynsim_edo_B.result_data[1] * dynsim_edo_B.k_a;
      dynsim_edo_B.v[2] = dynsim_edo_B.result_data[2] * dynsim_edo_B.k_a;
      dynsim_edo_B.k_a = cos(dynsim_edo_B.result_data[3]);
      dynsim_edo_B.sth = sin(dynsim_edo_B.result_data[3]);
      dynsim_edo_B.tempR[0] = dynsim_edo_B.v[0] * dynsim_edo_B.v[0] * (1.0 -
        dynsim_edo_B.k_a) + dynsim_edo_B.k_a;
      dynsim_edo_B.tempR_tmp = dynsim_edo_B.v[1] * dynsim_edo_B.v[0] * (1.0 -
        dynsim_edo_B.k_a);
      dynsim_edo_B.tempR_tmp_e = dynsim_edo_B.v[2] * dynsim_edo_B.sth;
      dynsim_edo_B.tempR[1] = dynsim_edo_B.tempR_tmp - dynsim_edo_B.tempR_tmp_e;
      dynsim_edo_B.tempR_tmp_a = dynsim_edo_B.v[2] * dynsim_edo_B.v[0] * (1.0 -
        dynsim_edo_B.k_a);
      dynsim_edo_B.tempR_tmp_as = dynsim_edo_B.v[1] * dynsim_edo_B.sth;
      dynsim_edo_B.tempR[2] = dynsim_edo_B.tempR_tmp_a +
        dynsim_edo_B.tempR_tmp_as;
      dynsim_edo_B.tempR[3] = dynsim_edo_B.tempR_tmp + dynsim_edo_B.tempR_tmp_e;
      dynsim_edo_B.tempR[4] = dynsim_edo_B.v[1] * dynsim_edo_B.v[1] * (1.0 -
        dynsim_edo_B.k_a) + dynsim_edo_B.k_a;
      dynsim_edo_B.tempR_tmp = dynsim_edo_B.v[2] * dynsim_edo_B.v[1] * (1.0 -
        dynsim_edo_B.k_a);
      dynsim_edo_B.tempR_tmp_e = dynsim_edo_B.v[0] * dynsim_edo_B.sth;
      dynsim_edo_B.tempR[5] = dynsim_edo_B.tempR_tmp - dynsim_edo_B.tempR_tmp_e;
      dynsim_edo_B.tempR[6] = dynsim_edo_B.tempR_tmp_a -
        dynsim_edo_B.tempR_tmp_as;
      dynsim_edo_B.tempR[7] = dynsim_edo_B.tempR_tmp + dynsim_edo_B.tempR_tmp_e;
      dynsim_edo_B.tempR[8] = dynsim_edo_B.v[2] * dynsim_edo_B.v[2] * (1.0 -
        dynsim_edo_B.k_a) + dynsim_edo_B.k_a;
      for (dynsim_edo_B.b_kstr = 0; dynsim_edo_B.b_kstr < 3; dynsim_edo_B.b_kstr
           ++) {
        dynsim_edo_B.e = dynsim_edo_B.b_kstr + 1;
        dynsim_edo_B.R_dh[dynsim_edo_B.e - 1] = dynsim_edo_B.tempR
          [(dynsim_edo_B.e - 1) * 3];
        dynsim_edo_B.e = dynsim_edo_B.b_kstr + 1;
        dynsim_edo_B.R_dh[dynsim_edo_B.e + 2] = dynsim_edo_B.tempR
          [(dynsim_edo_B.e - 1) * 3 + 1];
        dynsim_edo_B.e = dynsim_edo_B.b_kstr + 1;
        dynsim_edo_B.R_dh[dynsim_edo_B.e + 5] = dynsim_edo_B.tempR
          [(dynsim_edo_B.e - 1) * 3 + 2];
      }

      memset(&dynsim_edo_B.c_f1[0], 0, sizeof(real_T) << 4U);
      for (dynsim_edo_B.b_kstr = 0; dynsim_edo_B.b_kstr < 3; dynsim_edo_B.b_kstr
           ++) {
        dynsim_edo_B.d_b = dynsim_edo_B.b_kstr << 2;
        dynsim_edo_B.c_f1[dynsim_edo_B.d_b] = dynsim_edo_B.R_dh[3 *
          dynsim_edo_B.b_kstr];
        dynsim_edo_B.c_f1[dynsim_edo_B.d_b + 1] = dynsim_edo_B.R_dh[3 *
          dynsim_edo_B.b_kstr + 1];
        dynsim_edo_B.c_f1[dynsim_edo_B.d_b + 2] = dynsim_edo_B.R_dh[3 *
          dynsim_edo_B.b_kstr + 2];
      }

      dynsim_edo_B.c_f1[15] = 1.0;
      break;

     default:
      dy_rigidBodyJoint_get_JointAxis(&body->JointInternal, dynsim_edo_B.v);
      memset(&dynsim_edo_B.tempR[0], 0, 9U * sizeof(real_T));
      dynsim_edo_B.tempR[0] = 1.0;
      dynsim_edo_B.tempR[4] = 1.0;
      dynsim_edo_B.tempR[8] = 1.0;
      for (dynsim_edo_B.b_kstr = 0; dynsim_edo_B.b_kstr < 3; dynsim_edo_B.b_kstr
           ++) {
        dynsim_edo_B.d_b = dynsim_edo_B.b_kstr << 2;
        dynsim_edo_B.c_f1[dynsim_edo_B.d_b] = dynsim_edo_B.tempR[3 *
          dynsim_edo_B.b_kstr];
        dynsim_edo_B.c_f1[dynsim_edo_B.d_b + 1] = dynsim_edo_B.tempR[3 *
          dynsim_edo_B.b_kstr + 1];
        dynsim_edo_B.c_f1[dynsim_edo_B.d_b + 2] = dynsim_edo_B.tempR[3 *
          dynsim_edo_B.b_kstr + 2];
        dynsim_edo_B.c_f1[dynsim_edo_B.b_kstr + 12] =
          dynsim_edo_B.v[dynsim_edo_B.b_kstr] * qvec[dynsim_edo_B.e];
      }

      dynsim_edo_B.c_f1[3] = 0.0;
      dynsim_edo_B.c_f1[7] = 0.0;
      dynsim_edo_B.c_f1[11] = 0.0;
      dynsim_edo_B.c_f1[15] = 1.0;
      break;
    }

    for (dynsim_edo_B.b_kstr = 0; dynsim_edo_B.b_kstr < 16; dynsim_edo_B.b_kstr
         ++) {
      dynsim_edo_B.a[dynsim_edo_B.b_kstr] =
        body->JointInternal.JointToParentTransform[dynsim_edo_B.b_kstr];
    }

    for (dynsim_edo_B.b_kstr = 0; dynsim_edo_B.b_kstr < 16; dynsim_edo_B.b_kstr
         ++) {
      dynsim_edo_B.b[dynsim_edo_B.b_kstr] =
        body->JointInternal.ChildToJointTransform[dynsim_edo_B.b_kstr];
    }

    for (dynsim_edo_B.b_kstr = 0; dynsim_edo_B.b_kstr < 4; dynsim_edo_B.b_kstr++)
    {
      for (dynsim_edo_B.e = 0; dynsim_edo_B.e < 4; dynsim_edo_B.e++) {
        dynsim_edo_B.d_b = dynsim_edo_B.e << 2;
        dynsim_edo_B.loop_ub_a = dynsim_edo_B.b_kstr + dynsim_edo_B.d_b;
        dynsim_edo_B.a_p[dynsim_edo_B.loop_ub_a] = 0.0;
        dynsim_edo_B.a_p[dynsim_edo_B.loop_ub_a] +=
          dynsim_edo_B.c_f1[dynsim_edo_B.d_b] *
          dynsim_edo_B.a[dynsim_edo_B.b_kstr];
        dynsim_edo_B.a_p[dynsim_edo_B.loop_ub_a] +=
          dynsim_edo_B.c_f1[dynsim_edo_B.d_b + 1] *
          dynsim_edo_B.a[dynsim_edo_B.b_kstr + 4];
        dynsim_edo_B.a_p[dynsim_edo_B.loop_ub_a] +=
          dynsim_edo_B.c_f1[dynsim_edo_B.d_b + 2] *
          dynsim_edo_B.a[dynsim_edo_B.b_kstr + 8];
        dynsim_edo_B.a_p[dynsim_edo_B.loop_ub_a] +=
          dynsim_edo_B.c_f1[dynsim_edo_B.d_b + 3] *
          dynsim_edo_B.a[dynsim_edo_B.b_kstr + 12];
      }

      for (dynsim_edo_B.e = 0; dynsim_edo_B.e < 4; dynsim_edo_B.e++) {
        dynsim_edo_B.d_b = dynsim_edo_B.e << 2;
        dynsim_edo_B.loop_ub_a = dynsim_edo_B.b_kstr + dynsim_edo_B.d_b;
        Ttree->data[dynsim_edo_B.b_jtilecol].f1[dynsim_edo_B.loop_ub_a] = 0.0;
        Ttree->data[dynsim_edo_B.b_jtilecol].f1[dynsim_edo_B.loop_ub_a] +=
          dynsim_edo_B.b[dynsim_edo_B.d_b] *
          dynsim_edo_B.a_p[dynsim_edo_B.b_kstr];
        Ttree->data[dynsim_edo_B.b_jtilecol].f1[dynsim_edo_B.loop_ub_a] +=
          dynsim_edo_B.b[dynsim_edo_B.d_b + 1] *
          dynsim_edo_B.a_p[dynsim_edo_B.b_kstr + 4];
        Ttree->data[dynsim_edo_B.b_jtilecol].f1[dynsim_edo_B.loop_ub_a] +=
          dynsim_edo_B.b[dynsim_edo_B.d_b + 2] *
          dynsim_edo_B.a_p[dynsim_edo_B.b_kstr + 8];
        Ttree->data[dynsim_edo_B.b_jtilecol].f1[dynsim_edo_B.loop_ub_a] +=
          dynsim_edo_B.b[dynsim_edo_B.d_b + 3] *
          dynsim_edo_B.a_p[dynsim_edo_B.b_kstr + 12];
      }
    }

    dynsim_edo_B.k_a = dynsim_edo_B.n;
    if (body->ParentIndex > 0.0) {
      for (dynsim_edo_B.b_kstr = 0; dynsim_edo_B.b_kstr < 16;
           dynsim_edo_B.b_kstr++) {
        dynsim_edo_B.a[dynsim_edo_B.b_kstr] = Ttree->data[static_cast<int32_T>
          (body->ParentIndex) - 1].f1[dynsim_edo_B.b_kstr];
      }

      for (dynsim_edo_B.b_kstr = 0; dynsim_edo_B.b_kstr < 4; dynsim_edo_B.b_kstr
           ++) {
        for (dynsim_edo_B.e = 0; dynsim_edo_B.e < 4; dynsim_edo_B.e++) {
          dynsim_edo_B.d_b = dynsim_edo_B.e << 2;
          dynsim_edo_B.loop_ub_a = dynsim_edo_B.b_kstr + dynsim_edo_B.d_b;
          dynsim_edo_B.a_p[dynsim_edo_B.loop_ub_a] = 0.0;
          dynsim_edo_B.a_p[dynsim_edo_B.loop_ub_a] += Ttree->
            data[dynsim_edo_B.b_jtilecol].f1[dynsim_edo_B.d_b] *
            dynsim_edo_B.a[dynsim_edo_B.b_kstr];
          dynsim_edo_B.a_p[dynsim_edo_B.loop_ub_a] += Ttree->
            data[dynsim_edo_B.b_jtilecol].f1[dynsim_edo_B.d_b + 1] *
            dynsim_edo_B.a[dynsim_edo_B.b_kstr + 4];
          dynsim_edo_B.a_p[dynsim_edo_B.loop_ub_a] += Ttree->
            data[dynsim_edo_B.b_jtilecol].f1[dynsim_edo_B.d_b + 2] *
            dynsim_edo_B.a[dynsim_edo_B.b_kstr + 8];
          dynsim_edo_B.a_p[dynsim_edo_B.loop_ub_a] += Ttree->
            data[dynsim_edo_B.b_jtilecol].f1[dynsim_edo_B.d_b + 3] *
            dynsim_edo_B.a[dynsim_edo_B.b_kstr + 12];
        }
      }

      for (dynsim_edo_B.b_kstr = 0; dynsim_edo_B.b_kstr < 16;
           dynsim_edo_B.b_kstr++) {
        Ttree->data[dynsim_edo_B.b_jtilecol].f1[dynsim_edo_B.b_kstr] =
          dynsim_edo_B.a_p[dynsim_edo_B.b_kstr];
      }
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
}

static void dynsim_emxEnsureCapacity_real_T(emxArray_real_T_dynsim_edo_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  dynsim_edo_B.newNumel = 1;
  for (dynsim_edo_B.i_p = 0; dynsim_edo_B.i_p < emxArray->numDimensions;
       dynsim_edo_B.i_p++) {
    dynsim_edo_B.newNumel *= emxArray->size[dynsim_edo_B.i_p];
  }

  if (dynsim_edo_B.newNumel > emxArray->allocatedSize) {
    dynsim_edo_B.i_p = emxArray->allocatedSize;
    if (dynsim_edo_B.i_p < 16) {
      dynsim_edo_B.i_p = 16;
    }

    while (dynsim_edo_B.i_p < dynsim_edo_B.newNumel) {
      if (dynsim_edo_B.i_p > 1073741823) {
        dynsim_edo_B.i_p = MAX_int32_T;
      } else {
        dynsim_edo_B.i_p <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(dynsim_edo_B.i_p), sizeof(real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = dynsim_edo_B.i_p;
    emxArray->canFreeData = true;
  }
}

static void dynsim_edo_emxFree_real_T(emxArray_real_T_dynsim_edo_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T_dynsim_edo_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T_dynsim_edo_T *)NULL;
  }
}

static void dynsim_edo_emxFree_f_cell_wrap(emxArray_f_cell_wrap_dynsim_e_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_cell_wrap_dynsim_e_T *)NULL) {
    if (((*pEmxArray)->data != (f_cell_wrap_dynsim_edo_T *)NULL) && (*pEmxArray
        )->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_cell_wrap_dynsim_e_T *)NULL;
  }
}

static void RigidBodyTree_geometricJacobian(n_robotics_manip_internal_Rig_T *obj,
  const real_T Q[3], emxArray_real_T_dynsim_edo_T *Jac)
{
  emxArray_f_cell_wrap_dynsim_e_T *Ttree;
  l_robotics_manip_internal_Rig_T *body;
  emxArray_real_T_dynsim_edo_T *JacSlice;
  emxArray_char_T_dynsim_edo_T *bname;
  emxArray_real_T_dynsim_edo_T *b;
  static const char_T tmp[6] = { 'l', 'i', 'n', 'k', '_', '4' };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  boolean_T exitg2;
  dynsim_edo_emxInit_f_cell_wrap(&Ttree, 2);
  RigidBodyTree_forwardKinematics(obj, Q, Ttree);
  dynsim_edo_B.ret_e = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = static_cast<int32_T>(obj->VelocityNumber);
  dynsim_emxEnsureCapacity_real_T(Jac, dynsim_edo_B.ret_e);
  dynsim_edo_B.loop_ub_o = 6 * static_cast<int32_T>(obj->VelocityNumber) - 1;
  for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e <= dynsim_edo_B.loop_ub_o;
       dynsim_edo_B.ret_e++) {
    Jac->data[dynsim_edo_B.ret_e] = 0.0;
  }

  for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 6; dynsim_edo_B.ret_e++) {
    dynsim_edo_B.chainmask[dynsim_edo_B.ret_e] = 0;
  }

  dynsim_edo_emxInit_char_T(&bname, 2);
  dynsim_edo_B.ret_e = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  dynsim_emxEnsureCapacity_char_T(bname, dynsim_edo_B.ret_e);
  dynsim_edo_B.loop_ub_o = obj->Base.NameInternal->size[0] *
    obj->Base.NameInternal->size[1] - 1;
  for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e <= dynsim_edo_B.loop_ub_o;
       dynsim_edo_B.ret_e++) {
    bname->data[dynsim_edo_B.ret_e] = obj->Base.NameInternal->
      data[dynsim_edo_B.ret_e];
  }

  for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 6; dynsim_edo_B.ret_e++) {
    dynsim_edo_B.a_l5[dynsim_edo_B.ret_e] = tmp[dynsim_edo_B.ret_e];
  }

  dynsim_edo_B.b_bool_a = false;
  if (6 == bname->size[1]) {
    dynsim_edo_B.ret_e = 1;
    do {
      exitg1 = 0;
      if (dynsim_edo_B.ret_e - 1 < 6) {
        dynsim_edo_B.kstr = dynsim_edo_B.ret_e - 1;
        if (dynsim_edo_B.a_l5[dynsim_edo_B.kstr] != bname->
            data[dynsim_edo_B.kstr]) {
          exitg1 = 1;
        } else {
          dynsim_edo_B.ret_e++;
        }
      } else {
        dynsim_edo_B.b_bool_a = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (dynsim_edo_B.b_bool_a) {
    memset(&dynsim_edo_B.T2inv[0], 0, sizeof(real_T) << 4U);
    dynsim_edo_B.T2inv[0] = 1.0;
    dynsim_edo_B.T2inv[5] = 1.0;
    dynsim_edo_B.T2inv[10] = 1.0;
    dynsim_edo_B.T2inv[15] = 1.0;
    memset(&dynsim_edo_B.T2[0], 0, sizeof(real_T) << 4U);
    dynsim_edo_B.T2[0] = 1.0;
    dynsim_edo_B.T2[5] = 1.0;
    dynsim_edo_B.T2[10] = 1.0;
    dynsim_edo_B.T2[15] = 1.0;
  } else {
    dynsim_edo_B.endeffectorIndex = -1.0;
    dynsim_edo_B.ret_e = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = obj->Base.NameInternal->size[1];
    dynsim_emxEnsureCapacity_char_T(bname, dynsim_edo_B.ret_e);
    dynsim_edo_B.loop_ub_o = obj->Base.NameInternal->size[0] *
      obj->Base.NameInternal->size[1] - 1;
    for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e <= dynsim_edo_B.loop_ub_o;
         dynsim_edo_B.ret_e++) {
      bname->data[dynsim_edo_B.ret_e] = obj->Base.NameInternal->
        data[dynsim_edo_B.ret_e];
    }

    for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 6; dynsim_edo_B.ret_e++) {
      dynsim_edo_B.a_l5[dynsim_edo_B.ret_e] = tmp[dynsim_edo_B.ret_e];
    }

    dynsim_edo_B.b_bool_a = false;
    if (bname->size[1] == 6) {
      dynsim_edo_B.ret_e = 1;
      do {
        exitg1 = 0;
        if (dynsim_edo_B.ret_e - 1 < 6) {
          dynsim_edo_B.kstr = dynsim_edo_B.ret_e - 1;
          if (bname->data[dynsim_edo_B.kstr] !=
              dynsim_edo_B.a_l5[dynsim_edo_B.kstr]) {
            exitg1 = 1;
          } else {
            dynsim_edo_B.ret_e++;
          }
        } else {
          dynsim_edo_B.b_bool_a = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_edo_B.b_bool_a) {
      dynsim_edo_B.endeffectorIndex = 0.0;
    } else {
      dynsim_edo_B.idx_idx_1 = obj->NumBodies;
      dynsim_edo_B.b_i = 0;
      exitg2 = false;
      while ((!exitg2) && (dynsim_edo_B.b_i <= static_cast<int32_T>
                           (dynsim_edo_B.idx_idx_1) - 1)) {
        body = obj->Bodies[dynsim_edo_B.b_i];
        for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 6; dynsim_edo_B.ret_e
             ++) {
          dynsim_edo_B.bname_h[dynsim_edo_B.ret_e] = body->
            NameInternal[dynsim_edo_B.ret_e];
        }

        for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 6; dynsim_edo_B.ret_e
             ++) {
          dynsim_edo_B.a_l5[dynsim_edo_B.ret_e] = tmp[dynsim_edo_B.ret_e];
        }

        dynsim_edo_B.ret_e = memcmp(&dynsim_edo_B.bname_h[0],
          &dynsim_edo_B.a_l5[0], 6);
        if (dynsim_edo_B.ret_e == 0) {
          dynsim_edo_B.endeffectorIndex = static_cast<real_T>(dynsim_edo_B.b_i)
            + 1.0;
          exitg2 = true;
        } else {
          dynsim_edo_B.b_i++;
        }
      }
    }

    dynsim_edo_B.b_i = static_cast<int32_T>(dynsim_edo_B.endeffectorIndex) - 1;
    body = obj->Bodies[dynsim_edo_B.b_i];
    for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 16; dynsim_edo_B.ret_e++)
    {
      dynsim_edo_B.T2[dynsim_edo_B.ret_e] = Ttree->data[dynsim_edo_B.b_i]
        .f1[dynsim_edo_B.ret_e];
    }

    for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 3; dynsim_edo_B.ret_e++) {
      dynsim_edo_B.R_g[3 * dynsim_edo_B.ret_e] = Ttree->data[dynsim_edo_B.b_i].
        f1[dynsim_edo_B.ret_e];
      dynsim_edo_B.R_g[3 * dynsim_edo_B.ret_e + 1] = Ttree->
        data[dynsim_edo_B.b_i].f1[dynsim_edo_B.ret_e + 4];
      dynsim_edo_B.R_g[3 * dynsim_edo_B.ret_e + 2] = Ttree->
        data[dynsim_edo_B.b_i].f1[dynsim_edo_B.ret_e + 8];
    }

    for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 9; dynsim_edo_B.ret_e++) {
      dynsim_edo_B.R_l[dynsim_edo_B.ret_e] =
        -dynsim_edo_B.R_g[dynsim_edo_B.ret_e];
    }

    for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 3; dynsim_edo_B.ret_e++) {
      dynsim_edo_B.endeffectorIndex = Ttree->data[dynsim_edo_B.b_i].f1[12] *
        dynsim_edo_B.R_l[dynsim_edo_B.ret_e];
      dynsim_edo_B.loop_ub_o = dynsim_edo_B.ret_e << 2;
      dynsim_edo_B.T2inv[dynsim_edo_B.loop_ub_o] = dynsim_edo_B.R_g[3 *
        dynsim_edo_B.ret_e];
      dynsim_edo_B.endeffectorIndex += dynsim_edo_B.R_l[dynsim_edo_B.ret_e + 3] *
        Ttree->data[dynsim_edo_B.b_i].f1[13];
      dynsim_edo_B.T2inv[dynsim_edo_B.loop_ub_o + 1] = dynsim_edo_B.R_g[3 *
        dynsim_edo_B.ret_e + 1];
      dynsim_edo_B.endeffectorIndex += dynsim_edo_B.R_l[dynsim_edo_B.ret_e + 6] *
        Ttree->data[dynsim_edo_B.b_i].f1[14];
      dynsim_edo_B.T2inv[dynsim_edo_B.loop_ub_o + 2] = dynsim_edo_B.R_g[3 *
        dynsim_edo_B.ret_e + 2];
      dynsim_edo_B.T2inv[dynsim_edo_B.ret_e + 12] =
        dynsim_edo_B.endeffectorIndex;
    }

    dynsim_edo_B.T2inv[3] = 0.0;
    dynsim_edo_B.T2inv[7] = 0.0;
    dynsim_edo_B.T2inv[11] = 0.0;
    dynsim_edo_B.T2inv[15] = 1.0;
    dynsim_edo_B.chainmask[dynsim_edo_B.b_i] = 1;
    while (body->ParentIndex > 0.0) {
      body = obj->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
      dynsim_edo_B.chainmask[static_cast<int32_T>(body->Index) - 1] = 1;
    }
  }

  dynsim_edo_B.idx_idx_1 = obj->NumBodies;
  dynsim_edo_B.c_a = static_cast<int32_T>(dynsim_edo_B.idx_idx_1) - 1;
  dynsim_edo_emxInit_real_T(&JacSlice, 2);
  dynsim_edo_emxInit_real_T(&b, 2);
  if (0 <= dynsim_edo_B.c_a) {
    for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 5; dynsim_edo_B.ret_e++) {
      dynsim_edo_B.b_me[dynsim_edo_B.ret_e] = tmp_0[dynsim_edo_B.ret_e];
    }
  }

  for (dynsim_edo_B.b_i = 0; dynsim_edo_B.b_i <= dynsim_edo_B.c_a;
       dynsim_edo_B.b_i++) {
    body = obj->Bodies[dynsim_edo_B.b_i];
    dynsim_edo_B.ret_e = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = body->JointInternal.Type->size[1];
    dynsim_emxEnsureCapacity_char_T(bname, dynsim_edo_B.ret_e);
    dynsim_edo_B.loop_ub_o = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e <= dynsim_edo_B.loop_ub_o;
         dynsim_edo_B.ret_e++) {
      bname->data[dynsim_edo_B.ret_e] = body->JointInternal.Type->
        data[dynsim_edo_B.ret_e];
    }

    dynsim_edo_B.b_bool_a = false;
    if (bname->size[1] == 5) {
      dynsim_edo_B.ret_e = 1;
      do {
        exitg1 = 0;
        if (dynsim_edo_B.ret_e - 1 < 5) {
          dynsim_edo_B.kstr = dynsim_edo_B.ret_e - 1;
          if (bname->data[dynsim_edo_B.kstr] !=
              dynsim_edo_B.b_me[dynsim_edo_B.kstr]) {
            exitg1 = 1;
          } else {
            dynsim_edo_B.ret_e++;
          }
        } else {
          dynsim_edo_B.b_bool_a = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if ((!dynsim_edo_B.b_bool_a) && (dynsim_edo_B.chainmask[dynsim_edo_B.b_i] !=
         0)) {
      for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 16; dynsim_edo_B.ret_e++)
      {
        dynsim_edo_B.T1_b[dynsim_edo_B.ret_e] = Ttree->data[static_cast<int32_T>
          (body->Index) - 1].f1[dynsim_edo_B.ret_e];
      }

      for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 16; dynsim_edo_B.ret_e++)
      {
        dynsim_edo_B.Tdh[dynsim_edo_B.ret_e] =
          body->JointInternal.ChildToJointTransform[dynsim_edo_B.ret_e];
      }

      for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 3; dynsim_edo_B.ret_e++)
      {
        dynsim_edo_B.R_g[3 * dynsim_edo_B.ret_e] =
          dynsim_edo_B.Tdh[dynsim_edo_B.ret_e];
        dynsim_edo_B.R_g[3 * dynsim_edo_B.ret_e + 1] =
          dynsim_edo_B.Tdh[dynsim_edo_B.ret_e + 4];
        dynsim_edo_B.R_g[3 * dynsim_edo_B.ret_e + 2] =
          dynsim_edo_B.Tdh[dynsim_edo_B.ret_e + 8];
      }

      for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 9; dynsim_edo_B.ret_e++)
      {
        dynsim_edo_B.R_l[dynsim_edo_B.ret_e] =
          -dynsim_edo_B.R_g[dynsim_edo_B.ret_e];
      }

      for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 3; dynsim_edo_B.ret_e++)
      {
        dynsim_edo_B.R_a[dynsim_edo_B.ret_e] =
          dynsim_edo_B.R_l[dynsim_edo_B.ret_e + 6] * dynsim_edo_B.Tdh[14] +
          (dynsim_edo_B.R_l[dynsim_edo_B.ret_e + 3] * dynsim_edo_B.Tdh[13] +
           dynsim_edo_B.R_l[dynsim_edo_B.ret_e] * dynsim_edo_B.Tdh[12]);
      }

      for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 4; dynsim_edo_B.ret_e++)
      {
        for (dynsim_edo_B.kstr = 0; dynsim_edo_B.kstr < 4; dynsim_edo_B.kstr++)
        {
          dynsim_edo_B.n_j = dynsim_edo_B.kstr << 2;
          dynsim_edo_B.loop_ub_o = dynsim_edo_B.ret_e + dynsim_edo_B.n_j;
          dynsim_edo_B.Tdh[dynsim_edo_B.loop_ub_o] = 0.0;
          dynsim_edo_B.Tdh[dynsim_edo_B.loop_ub_o] +=
            dynsim_edo_B.T1_b[dynsim_edo_B.n_j] *
            dynsim_edo_B.T2inv[dynsim_edo_B.ret_e];
          dynsim_edo_B.Tdh[dynsim_edo_B.loop_ub_o] +=
            dynsim_edo_B.T1_b[dynsim_edo_B.n_j + 1] *
            dynsim_edo_B.T2inv[dynsim_edo_B.ret_e + 4];
          dynsim_edo_B.Tdh[dynsim_edo_B.loop_ub_o] +=
            dynsim_edo_B.T1_b[dynsim_edo_B.n_j + 2] *
            dynsim_edo_B.T2inv[dynsim_edo_B.ret_e + 8];
          dynsim_edo_B.Tdh[dynsim_edo_B.loop_ub_o] +=
            dynsim_edo_B.T1_b[dynsim_edo_B.n_j + 3] *
            dynsim_edo_B.T2inv[dynsim_edo_B.ret_e + 12];
        }
      }

      for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 3; dynsim_edo_B.ret_e++)
      {
        dynsim_edo_B.kstr = dynsim_edo_B.ret_e << 2;
        dynsim_edo_B.T1_b[dynsim_edo_B.kstr] = dynsim_edo_B.R_g[3 *
          dynsim_edo_B.ret_e];
        dynsim_edo_B.T1_b[dynsim_edo_B.kstr + 1] = dynsim_edo_B.R_g[3 *
          dynsim_edo_B.ret_e + 1];
        dynsim_edo_B.T1_b[dynsim_edo_B.kstr + 2] = dynsim_edo_B.R_g[3 *
          dynsim_edo_B.ret_e + 2];
        dynsim_edo_B.T1_b[dynsim_edo_B.ret_e + 12] =
          dynsim_edo_B.R_a[dynsim_edo_B.ret_e];
      }

      dynsim_edo_B.T1_b[3] = 0.0;
      dynsim_edo_B.T1_b[7] = 0.0;
      dynsim_edo_B.T1_b[11] = 0.0;
      dynsim_edo_B.T1_b[15] = 1.0;
      for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 4; dynsim_edo_B.ret_e++)
      {
        for (dynsim_edo_B.kstr = 0; dynsim_edo_B.kstr < 4; dynsim_edo_B.kstr++)
        {
          dynsim_edo_B.loop_ub_o = dynsim_edo_B.kstr << 2;
          dynsim_edo_B.n_j = dynsim_edo_B.ret_e + dynsim_edo_B.loop_ub_o;
          dynsim_edo_B.T[dynsim_edo_B.n_j] = 0.0;
          dynsim_edo_B.T[dynsim_edo_B.n_j] +=
            dynsim_edo_B.T1_b[dynsim_edo_B.loop_ub_o] *
            dynsim_edo_B.Tdh[dynsim_edo_B.ret_e];
          dynsim_edo_B.T[dynsim_edo_B.n_j] +=
            dynsim_edo_B.T1_b[dynsim_edo_B.loop_ub_o + 1] *
            dynsim_edo_B.Tdh[dynsim_edo_B.ret_e + 4];
          dynsim_edo_B.T[dynsim_edo_B.n_j] +=
            dynsim_edo_B.T1_b[dynsim_edo_B.loop_ub_o + 2] *
            dynsim_edo_B.Tdh[dynsim_edo_B.ret_e + 8];
          dynsim_edo_B.T[dynsim_edo_B.n_j] +=
            dynsim_edo_B.T1_b[dynsim_edo_B.loop_ub_o + 3] *
            dynsim_edo_B.Tdh[dynsim_edo_B.ret_e + 12];
        }
      }

      dynsim_edo_B.endeffectorIndex = obj->PositionDoFMap[dynsim_edo_B.b_i];
      dynsim_edo_B.idx_idx_1 = obj->PositionDoFMap[dynsim_edo_B.b_i + 6];
      dynsim_edo_B.R_g[0] = 0.0;
      dynsim_edo_B.R_g[3] = -dynsim_edo_B.T[14];
      dynsim_edo_B.R_g[6] = dynsim_edo_B.T[13];
      dynsim_edo_B.R_g[1] = dynsim_edo_B.T[14];
      dynsim_edo_B.R_g[4] = 0.0;
      dynsim_edo_B.R_g[7] = -dynsim_edo_B.T[12];
      dynsim_edo_B.R_g[2] = -dynsim_edo_B.T[13];
      dynsim_edo_B.R_g[5] = dynsim_edo_B.T[12];
      dynsim_edo_B.R_g[8] = 0.0;
      for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 3; dynsim_edo_B.ret_e++)
      {
        for (dynsim_edo_B.kstr = 0; dynsim_edo_B.kstr < 3; dynsim_edo_B.kstr++)
        {
          dynsim_edo_B.loop_ub_o = dynsim_edo_B.ret_e + 3 * dynsim_edo_B.kstr;
          dynsim_edo_B.R_l[dynsim_edo_B.loop_ub_o] = 0.0;
          dynsim_edo_B.n_j = dynsim_edo_B.kstr << 2;
          dynsim_edo_B.R_l[dynsim_edo_B.loop_ub_o] +=
            dynsim_edo_B.T[dynsim_edo_B.n_j] *
            dynsim_edo_B.R_g[dynsim_edo_B.ret_e];
          dynsim_edo_B.R_l[dynsim_edo_B.loop_ub_o] +=
            dynsim_edo_B.T[dynsim_edo_B.n_j + 1] *
            dynsim_edo_B.R_g[dynsim_edo_B.ret_e + 3];
          dynsim_edo_B.R_l[dynsim_edo_B.loop_ub_o] +=
            dynsim_edo_B.T[dynsim_edo_B.n_j + 2] *
            dynsim_edo_B.R_g[dynsim_edo_B.ret_e + 6];
          dynsim_edo_B.X[dynsim_edo_B.kstr + 6 * dynsim_edo_B.ret_e] =
            dynsim_edo_B.T[(dynsim_edo_B.ret_e << 2) + dynsim_edo_B.kstr];
          dynsim_edo_B.X[dynsim_edo_B.kstr + 6 * (dynsim_edo_B.ret_e + 3)] = 0.0;
        }
      }

      for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 3; dynsim_edo_B.ret_e++)
      {
        dynsim_edo_B.X[6 * dynsim_edo_B.ret_e + 3] = dynsim_edo_B.R_l[3 *
          dynsim_edo_B.ret_e];
        dynsim_edo_B.kstr = dynsim_edo_B.ret_e << 2;
        dynsim_edo_B.loop_ub_o = 6 * (dynsim_edo_B.ret_e + 3);
        dynsim_edo_B.X[dynsim_edo_B.loop_ub_o + 3] =
          dynsim_edo_B.T[dynsim_edo_B.kstr];
        dynsim_edo_B.X[6 * dynsim_edo_B.ret_e + 4] = dynsim_edo_B.R_l[3 *
          dynsim_edo_B.ret_e + 1];
        dynsim_edo_B.X[dynsim_edo_B.loop_ub_o + 4] =
          dynsim_edo_B.T[dynsim_edo_B.kstr + 1];
        dynsim_edo_B.X[6 * dynsim_edo_B.ret_e + 5] = dynsim_edo_B.R_l[3 *
          dynsim_edo_B.ret_e + 2];
        dynsim_edo_B.X[dynsim_edo_B.loop_ub_o + 5] =
          dynsim_edo_B.T[dynsim_edo_B.kstr + 2];
      }

      dynsim_edo_B.ret_e = b->size[0] * b->size[1];
      b->size[0] = 6;
      b->size[1] = body->JointInternal.MotionSubspace->size[1];
      dynsim_emxEnsureCapacity_real_T(b, dynsim_edo_B.ret_e);
      dynsim_edo_B.loop_ub_o = body->JointInternal.MotionSubspace->size[0] *
        body->JointInternal.MotionSubspace->size[1] - 1;
      for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e <= dynsim_edo_B.loop_ub_o;
           dynsim_edo_B.ret_e++) {
        b->data[dynsim_edo_B.ret_e] = body->JointInternal.MotionSubspace->
          data[dynsim_edo_B.ret_e];
      }

      dynsim_edo_B.n_j = b->size[1] - 1;
      dynsim_edo_B.ret_e = JacSlice->size[0] * JacSlice->size[1];
      JacSlice->size[0] = 6;
      JacSlice->size[1] = b->size[1];
      dynsim_emxEnsureCapacity_real_T(JacSlice, dynsim_edo_B.ret_e);
      for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e <= dynsim_edo_B.n_j;
           dynsim_edo_B.ret_e++) {
        dynsim_edo_B.coffset_tmp = dynsim_edo_B.ret_e * 6 - 1;
        for (dynsim_edo_B.kstr = 0; dynsim_edo_B.kstr < 6; dynsim_edo_B.kstr++)
        {
          dynsim_edo_B.s = 0.0;
          for (dynsim_edo_B.loop_ub_o = 0; dynsim_edo_B.loop_ub_o < 6;
               dynsim_edo_B.loop_ub_o++) {
            dynsim_edo_B.s += dynsim_edo_B.X[dynsim_edo_B.loop_ub_o * 6 +
              dynsim_edo_B.kstr] * b->data[(dynsim_edo_B.coffset_tmp +
              dynsim_edo_B.loop_ub_o) + 1];
          }

          JacSlice->data[(dynsim_edo_B.coffset_tmp + dynsim_edo_B.kstr) + 1] =
            dynsim_edo_B.s;
        }
      }

      if (dynsim_edo_B.endeffectorIndex > dynsim_edo_B.idx_idx_1) {
        dynsim_edo_B.n_j = 0;
      } else {
        dynsim_edo_B.n_j = static_cast<int32_T>(dynsim_edo_B.endeffectorIndex) -
          1;
      }

      dynsim_edo_B.loop_ub_o = JacSlice->size[1];
      for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < dynsim_edo_B.loop_ub_o;
           dynsim_edo_B.ret_e++) {
        for (dynsim_edo_B.kstr = 0; dynsim_edo_B.kstr < 6; dynsim_edo_B.kstr++)
        {
          Jac->data[dynsim_edo_B.kstr + 6 * (dynsim_edo_B.n_j +
            dynsim_edo_B.ret_e)] = JacSlice->data[6 * dynsim_edo_B.ret_e +
            dynsim_edo_B.kstr];
        }
      }
    }
  }

  dynsim_edo_emxFree_char_T(&bname);
  dynsim_edo_emxFree_real_T(&JacSlice);
  dynsim_edo_emxFree_f_cell_wrap(&Ttree);
  for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < 3; dynsim_edo_B.ret_e++) {
    dynsim_edo_B.b_i = dynsim_edo_B.ret_e << 2;
    dynsim_edo_B.X[6 * dynsim_edo_B.ret_e] = dynsim_edo_B.T2[dynsim_edo_B.b_i];
    dynsim_edo_B.kstr = 6 * (dynsim_edo_B.ret_e + 3);
    dynsim_edo_B.X[dynsim_edo_B.kstr] = 0.0;
    dynsim_edo_B.X[6 * dynsim_edo_B.ret_e + 3] = 0.0;
    dynsim_edo_B.X[dynsim_edo_B.kstr + 3] = dynsim_edo_B.T2[dynsim_edo_B.b_i];
    dynsim_edo_B.endeffectorIndex = dynsim_edo_B.T2[dynsim_edo_B.b_i + 1];
    dynsim_edo_B.X[6 * dynsim_edo_B.ret_e + 1] = dynsim_edo_B.endeffectorIndex;
    dynsim_edo_B.X[dynsim_edo_B.kstr + 1] = 0.0;
    dynsim_edo_B.X[6 * dynsim_edo_B.ret_e + 4] = 0.0;
    dynsim_edo_B.X[dynsim_edo_B.kstr + 4] = dynsim_edo_B.endeffectorIndex;
    dynsim_edo_B.endeffectorIndex = dynsim_edo_B.T2[dynsim_edo_B.b_i + 2];
    dynsim_edo_B.X[6 * dynsim_edo_B.ret_e + 2] = dynsim_edo_B.endeffectorIndex;
    dynsim_edo_B.X[dynsim_edo_B.kstr + 2] = 0.0;
    dynsim_edo_B.X[6 * dynsim_edo_B.ret_e + 5] = 0.0;
    dynsim_edo_B.X[dynsim_edo_B.kstr + 5] = dynsim_edo_B.endeffectorIndex;
  }

  dynsim_edo_B.n_j = Jac->size[1];
  dynsim_edo_B.ret_e = b->size[0] * b->size[1];
  b->size[0] = 6;
  b->size[1] = Jac->size[1];
  dynsim_emxEnsureCapacity_real_T(b, dynsim_edo_B.ret_e);
  dynsim_edo_B.loop_ub_o = Jac->size[0] * Jac->size[1] - 1;
  for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e <= dynsim_edo_B.loop_ub_o;
       dynsim_edo_B.ret_e++) {
    b->data[dynsim_edo_B.ret_e] = Jac->data[dynsim_edo_B.ret_e];
  }

  dynsim_edo_B.ret_e = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = dynsim_edo_B.n_j;
  dynsim_emxEnsureCapacity_real_T(Jac, dynsim_edo_B.ret_e);
  for (dynsim_edo_B.ret_e = 0; dynsim_edo_B.ret_e < dynsim_edo_B.n_j;
       dynsim_edo_B.ret_e++) {
    dynsim_edo_B.coffset_tmp = dynsim_edo_B.ret_e * 6 - 1;
    for (dynsim_edo_B.b_i = 0; dynsim_edo_B.b_i < 6; dynsim_edo_B.b_i++) {
      dynsim_edo_B.s = 0.0;
      for (dynsim_edo_B.loop_ub_o = 0; dynsim_edo_B.loop_ub_o < 6;
           dynsim_edo_B.loop_ub_o++) {
        dynsim_edo_B.s += dynsim_edo_B.X[dynsim_edo_B.loop_ub_o * 6 +
          dynsim_edo_B.b_i] * b->data[(dynsim_edo_B.coffset_tmp +
          dynsim_edo_B.loop_ub_o) + 1];
      }

      Jac->data[(dynsim_edo_B.coffset_tmp + dynsim_edo_B.b_i) + 1] =
        dynsim_edo_B.s;
    }
  }

  dynsim_edo_emxFree_real_T(&b);
}

static void rigidBodyJoint_get_JointAxis_h(const c_rigidBodyJoint_dynsim_edo_h_T
  *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (dynsim_edo_B.b_kstr_m = 0; dynsim_edo_B.b_kstr_m < 8;
       dynsim_edo_B.b_kstr_m++) {
    dynsim_edo_B.b_p5[dynsim_edo_B.b_kstr_m] = tmp[dynsim_edo_B.b_kstr_m];
  }

  dynsim_edo_B.b_bool_nj = false;
  if (obj->Type->size[1] == 8) {
    dynsim_edo_B.b_kstr_m = 1;
    do {
      exitg1 = 0;
      if (dynsim_edo_B.b_kstr_m - 1 < 8) {
        dynsim_edo_B.kstr_h = dynsim_edo_B.b_kstr_m - 1;
        if (obj->Type->data[dynsim_edo_B.kstr_h] !=
            dynsim_edo_B.b_p5[dynsim_edo_B.kstr_h]) {
          exitg1 = 1;
        } else {
          dynsim_edo_B.b_kstr_m++;
        }
      } else {
        dynsim_edo_B.b_bool_nj = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (dynsim_edo_B.b_bool_nj) {
    guard1 = true;
  } else {
    for (dynsim_edo_B.b_kstr_m = 0; dynsim_edo_B.b_kstr_m < 9;
         dynsim_edo_B.b_kstr_m++) {
      dynsim_edo_B.b_c[dynsim_edo_B.b_kstr_m] = tmp_0[dynsim_edo_B.b_kstr_m];
    }

    dynsim_edo_B.b_bool_nj = false;
    if (obj->Type->size[1] == 9) {
      dynsim_edo_B.b_kstr_m = 1;
      do {
        exitg1 = 0;
        if (dynsim_edo_B.b_kstr_m - 1 < 9) {
          dynsim_edo_B.kstr_h = dynsim_edo_B.b_kstr_m - 1;
          if (obj->Type->data[dynsim_edo_B.kstr_h] !=
              dynsim_edo_B.b_c[dynsim_edo_B.kstr_h]) {
            exitg1 = 1;
          } else {
            dynsim_edo_B.b_kstr_m++;
          }
        } else {
          dynsim_edo_B.b_bool_nj = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_edo_B.b_bool_nj) {
      guard1 = true;
    } else {
      ax[0] = (rtNaN);
      ax[1] = (rtNaN);
      ax[2] = (rtNaN);
    }
  }

  if (guard1) {
    ax[0] = obj->JointAxisInternal[0];
    ax[1] = obj->JointAxisInternal[1];
    ax[2] = obj->JointAxisInternal[2];
  }
}

static void RigidBodyTree_forwardKinemati_h(n_robotics_manip_internal_R_h_T *obj,
  const real_T qvec[3], emxArray_f_cell_wrap_dynsim_e_T *Ttree)
{
  l_robotics_manip_internal_R_h_T *body;
  emxArray_char_T_dynsim_edo_T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  dynsim_edo_B.n_l = obj->NumBodies;
  for (dynsim_edo_B.b_kstr_cu = 0; dynsim_edo_B.b_kstr_cu < 16;
       dynsim_edo_B.b_kstr_cu++) {
    dynsim_edo_B.c_f1_m[dynsim_edo_B.b_kstr_cu] = tmp[dynsim_edo_B.b_kstr_cu];
  }

  dynsim_edo_B.b_kstr_cu = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  dynsim_edo_B.e_i = static_cast<int32_T>(dynsim_edo_B.n_l);
  Ttree->size[1] = dynsim_edo_B.e_i;
  d_emxEnsureCapacity_f_cell_wrap(Ttree, dynsim_edo_B.b_kstr_cu);
  if (dynsim_edo_B.e_i != 0) {
    dynsim_edo_B.ntilecols_o = dynsim_edo_B.e_i - 1;
    if (0 <= dynsim_edo_B.ntilecols_o) {
      memcpy(&dynsim_edo_B.expl_temp_c.f1[0], &dynsim_edo_B.c_f1_m[0], sizeof
             (real_T) << 4U);
    }

    for (dynsim_edo_B.b_jtilecol_m = 0; dynsim_edo_B.b_jtilecol_m <=
         dynsim_edo_B.ntilecols_o; dynsim_edo_B.b_jtilecol_m++) {
      Ttree->data[dynsim_edo_B.b_jtilecol_m] = dynsim_edo_B.expl_temp_c;
    }
  }

  dynsim_edo_B.k_m = 1.0;
  dynsim_edo_B.ntilecols_o = static_cast<int32_T>(dynsim_edo_B.n_l) - 1;
  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  if (0 <= dynsim_edo_B.ntilecols_o) {
    for (dynsim_edo_B.b_kstr_cu = 0; dynsim_edo_B.b_kstr_cu < 5;
         dynsim_edo_B.b_kstr_cu++) {
      dynsim_edo_B.b_pxv[dynsim_edo_B.b_kstr_cu] = tmp_0[dynsim_edo_B.b_kstr_cu];
    }
  }

  for (dynsim_edo_B.b_jtilecol_m = 0; dynsim_edo_B.b_jtilecol_m <=
       dynsim_edo_B.ntilecols_o; dynsim_edo_B.b_jtilecol_m++) {
    body = obj->Bodies[dynsim_edo_B.b_jtilecol_m];
    dynsim_edo_B.n_l = body->JointInternal.PositionNumber;
    dynsim_edo_B.n_l += dynsim_edo_B.k_m;
    if (dynsim_edo_B.k_m > dynsim_edo_B.n_l - 1.0) {
      dynsim_edo_B.e_i = 0;
      dynsim_edo_B.d_k = 0;
    } else {
      dynsim_edo_B.e_i = static_cast<int32_T>(dynsim_edo_B.k_m) - 1;
      dynsim_edo_B.d_k = static_cast<int32_T>(dynsim_edo_B.n_l - 1.0);
    }

    dynsim_edo_B.b_kstr_cu = switch_expression->size[0] *
      switch_expression->size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    dynsim_emxEnsureCapacity_char_T(switch_expression, dynsim_edo_B.b_kstr_cu);
    dynsim_edo_B.loop_ub_f = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (dynsim_edo_B.b_kstr_cu = 0; dynsim_edo_B.b_kstr_cu <=
         dynsim_edo_B.loop_ub_f; dynsim_edo_B.b_kstr_cu++) {
      switch_expression->data[dynsim_edo_B.b_kstr_cu] = body->
        JointInternal.Type->data[dynsim_edo_B.b_kstr_cu];
    }

    dynsim_edo_B.b_bool_p2 = false;
    if (switch_expression->size[1] == 5) {
      dynsim_edo_B.b_kstr_cu = 1;
      do {
        exitg1 = 0;
        if (dynsim_edo_B.b_kstr_cu - 1 < 5) {
          dynsim_edo_B.loop_ub_f = dynsim_edo_B.b_kstr_cu - 1;
          if (switch_expression->data[dynsim_edo_B.loop_ub_f] !=
              dynsim_edo_B.b_pxv[dynsim_edo_B.loop_ub_f]) {
            exitg1 = 1;
          } else {
            dynsim_edo_B.b_kstr_cu++;
          }
        } else {
          dynsim_edo_B.b_bool_p2 = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_edo_B.b_bool_p2) {
      dynsim_edo_B.b_kstr_cu = 0;
    } else {
      for (dynsim_edo_B.b_kstr_cu = 0; dynsim_edo_B.b_kstr_cu < 8;
           dynsim_edo_B.b_kstr_cu++) {
        dynsim_edo_B.b_px[dynsim_edo_B.b_kstr_cu] = tmp_1[dynsim_edo_B.b_kstr_cu];
      }

      dynsim_edo_B.b_bool_p2 = false;
      if (switch_expression->size[1] == 8) {
        dynsim_edo_B.b_kstr_cu = 1;
        do {
          exitg1 = 0;
          if (dynsim_edo_B.b_kstr_cu - 1 < 8) {
            dynsim_edo_B.loop_ub_f = dynsim_edo_B.b_kstr_cu - 1;
            if (switch_expression->data[dynsim_edo_B.loop_ub_f] !=
                dynsim_edo_B.b_px[dynsim_edo_B.loop_ub_f]) {
              exitg1 = 1;
            } else {
              dynsim_edo_B.b_kstr_cu++;
            }
          } else {
            dynsim_edo_B.b_bool_p2 = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (dynsim_edo_B.b_bool_p2) {
        dynsim_edo_B.b_kstr_cu = 1;
      } else {
        dynsim_edo_B.b_kstr_cu = -1;
      }
    }

    switch (dynsim_edo_B.b_kstr_cu) {
     case 0:
      memset(&dynsim_edo_B.c_f1_m[0], 0, sizeof(real_T) << 4U);
      dynsim_edo_B.c_f1_m[0] = 1.0;
      dynsim_edo_B.c_f1_m[5] = 1.0;
      dynsim_edo_B.c_f1_m[10] = 1.0;
      dynsim_edo_B.c_f1_m[15] = 1.0;
      break;

     case 1:
      rigidBodyJoint_get_JointAxis_h(&body->JointInternal, dynsim_edo_B.v_n);
      dynsim_edo_B.d_k -= dynsim_edo_B.e_i;
      for (dynsim_edo_B.b_kstr_cu = 0; dynsim_edo_B.b_kstr_cu < dynsim_edo_B.d_k;
           dynsim_edo_B.b_kstr_cu++) {
        dynsim_edo_B.e_data_o[dynsim_edo_B.b_kstr_cu] = dynsim_edo_B.e_i +
          dynsim_edo_B.b_kstr_cu;
      }

      dynsim_edo_B.result_data_f[0] = dynsim_edo_B.v_n[0];
      dynsim_edo_B.result_data_f[1] = dynsim_edo_B.v_n[1];
      dynsim_edo_B.result_data_f[2] = dynsim_edo_B.v_n[2];
      if (0 <= (dynsim_edo_B.d_k != 0) - 1) {
        dynsim_edo_B.result_data_f[3] = qvec[dynsim_edo_B.e_data_o[0]];
      }

      dynsim_edo_B.k_m = 1.0 / sqrt((dynsim_edo_B.result_data_f[0] *
        dynsim_edo_B.result_data_f[0] + dynsim_edo_B.result_data_f[1] *
        dynsim_edo_B.result_data_f[1]) + dynsim_edo_B.result_data_f[2] *
        dynsim_edo_B.result_data_f[2]);
      dynsim_edo_B.v_n[0] = dynsim_edo_B.result_data_f[0] * dynsim_edo_B.k_m;
      dynsim_edo_B.v_n[1] = dynsim_edo_B.result_data_f[1] * dynsim_edo_B.k_m;
      dynsim_edo_B.v_n[2] = dynsim_edo_B.result_data_f[2] * dynsim_edo_B.k_m;
      dynsim_edo_B.k_m = cos(dynsim_edo_B.result_data_f[3]);
      dynsim_edo_B.sth_m = sin(dynsim_edo_B.result_data_f[3]);
      dynsim_edo_B.tempR_h[0] = dynsim_edo_B.v_n[0] * dynsim_edo_B.v_n[0] * (1.0
        - dynsim_edo_B.k_m) + dynsim_edo_B.k_m;
      dynsim_edo_B.tempR_tmp_c = dynsim_edo_B.v_n[1] * dynsim_edo_B.v_n[0] *
        (1.0 - dynsim_edo_B.k_m);
      dynsim_edo_B.tempR_tmp_fm = dynsim_edo_B.v_n[2] * dynsim_edo_B.sth_m;
      dynsim_edo_B.tempR_h[1] = dynsim_edo_B.tempR_tmp_c -
        dynsim_edo_B.tempR_tmp_fm;
      dynsim_edo_B.tempR_tmp_p = dynsim_edo_B.v_n[2] * dynsim_edo_B.v_n[0] *
        (1.0 - dynsim_edo_B.k_m);
      dynsim_edo_B.tempR_tmp_e1 = dynsim_edo_B.v_n[1] * dynsim_edo_B.sth_m;
      dynsim_edo_B.tempR_h[2] = dynsim_edo_B.tempR_tmp_p +
        dynsim_edo_B.tempR_tmp_e1;
      dynsim_edo_B.tempR_h[3] = dynsim_edo_B.tempR_tmp_c +
        dynsim_edo_B.tempR_tmp_fm;
      dynsim_edo_B.tempR_h[4] = dynsim_edo_B.v_n[1] * dynsim_edo_B.v_n[1] * (1.0
        - dynsim_edo_B.k_m) + dynsim_edo_B.k_m;
      dynsim_edo_B.tempR_tmp_c = dynsim_edo_B.v_n[2] * dynsim_edo_B.v_n[1] *
        (1.0 - dynsim_edo_B.k_m);
      dynsim_edo_B.tempR_tmp_fm = dynsim_edo_B.v_n[0] * dynsim_edo_B.sth_m;
      dynsim_edo_B.tempR_h[5] = dynsim_edo_B.tempR_tmp_c -
        dynsim_edo_B.tempR_tmp_fm;
      dynsim_edo_B.tempR_h[6] = dynsim_edo_B.tempR_tmp_p -
        dynsim_edo_B.tempR_tmp_e1;
      dynsim_edo_B.tempR_h[7] = dynsim_edo_B.tempR_tmp_c +
        dynsim_edo_B.tempR_tmp_fm;
      dynsim_edo_B.tempR_h[8] = dynsim_edo_B.v_n[2] * dynsim_edo_B.v_n[2] * (1.0
        - dynsim_edo_B.k_m) + dynsim_edo_B.k_m;
      for (dynsim_edo_B.b_kstr_cu = 0; dynsim_edo_B.b_kstr_cu < 3;
           dynsim_edo_B.b_kstr_cu++) {
        dynsim_edo_B.e_i = dynsim_edo_B.b_kstr_cu + 1;
        dynsim_edo_B.R_ln[dynsim_edo_B.e_i - 1] = dynsim_edo_B.tempR_h
          [(dynsim_edo_B.e_i - 1) * 3];
        dynsim_edo_B.e_i = dynsim_edo_B.b_kstr_cu + 1;
        dynsim_edo_B.R_ln[dynsim_edo_B.e_i + 2] = dynsim_edo_B.tempR_h
          [(dynsim_edo_B.e_i - 1) * 3 + 1];
        dynsim_edo_B.e_i = dynsim_edo_B.b_kstr_cu + 1;
        dynsim_edo_B.R_ln[dynsim_edo_B.e_i + 5] = dynsim_edo_B.tempR_h
          [(dynsim_edo_B.e_i - 1) * 3 + 2];
      }

      memset(&dynsim_edo_B.c_f1_m[0], 0, sizeof(real_T) << 4U);
      for (dynsim_edo_B.b_kstr_cu = 0; dynsim_edo_B.b_kstr_cu < 3;
           dynsim_edo_B.b_kstr_cu++) {
        dynsim_edo_B.d_k = dynsim_edo_B.b_kstr_cu << 2;
        dynsim_edo_B.c_f1_m[dynsim_edo_B.d_k] = dynsim_edo_B.R_ln[3 *
          dynsim_edo_B.b_kstr_cu];
        dynsim_edo_B.c_f1_m[dynsim_edo_B.d_k + 1] = dynsim_edo_B.R_ln[3 *
          dynsim_edo_B.b_kstr_cu + 1];
        dynsim_edo_B.c_f1_m[dynsim_edo_B.d_k + 2] = dynsim_edo_B.R_ln[3 *
          dynsim_edo_B.b_kstr_cu + 2];
      }

      dynsim_edo_B.c_f1_m[15] = 1.0;
      break;

     default:
      rigidBodyJoint_get_JointAxis_h(&body->JointInternal, dynsim_edo_B.v_n);
      memset(&dynsim_edo_B.tempR_h[0], 0, 9U * sizeof(real_T));
      dynsim_edo_B.tempR_h[0] = 1.0;
      dynsim_edo_B.tempR_h[4] = 1.0;
      dynsim_edo_B.tempR_h[8] = 1.0;
      for (dynsim_edo_B.b_kstr_cu = 0; dynsim_edo_B.b_kstr_cu < 3;
           dynsim_edo_B.b_kstr_cu++) {
        dynsim_edo_B.d_k = dynsim_edo_B.b_kstr_cu << 2;
        dynsim_edo_B.c_f1_m[dynsim_edo_B.d_k] = dynsim_edo_B.tempR_h[3 *
          dynsim_edo_B.b_kstr_cu];
        dynsim_edo_B.c_f1_m[dynsim_edo_B.d_k + 1] = dynsim_edo_B.tempR_h[3 *
          dynsim_edo_B.b_kstr_cu + 1];
        dynsim_edo_B.c_f1_m[dynsim_edo_B.d_k + 2] = dynsim_edo_B.tempR_h[3 *
          dynsim_edo_B.b_kstr_cu + 2];
        dynsim_edo_B.c_f1_m[dynsim_edo_B.b_kstr_cu + 12] =
          dynsim_edo_B.v_n[dynsim_edo_B.b_kstr_cu] * qvec[dynsim_edo_B.e_i];
      }

      dynsim_edo_B.c_f1_m[3] = 0.0;
      dynsim_edo_B.c_f1_m[7] = 0.0;
      dynsim_edo_B.c_f1_m[11] = 0.0;
      dynsim_edo_B.c_f1_m[15] = 1.0;
      break;
    }

    for (dynsim_edo_B.b_kstr_cu = 0; dynsim_edo_B.b_kstr_cu < 16;
         dynsim_edo_B.b_kstr_cu++) {
      dynsim_edo_B.a_n[dynsim_edo_B.b_kstr_cu] =
        body->JointInternal.JointToParentTransform[dynsim_edo_B.b_kstr_cu];
    }

    for (dynsim_edo_B.b_kstr_cu = 0; dynsim_edo_B.b_kstr_cu < 16;
         dynsim_edo_B.b_kstr_cu++) {
      dynsim_edo_B.b_p[dynsim_edo_B.b_kstr_cu] =
        body->JointInternal.ChildToJointTransform[dynsim_edo_B.b_kstr_cu];
    }

    for (dynsim_edo_B.b_kstr_cu = 0; dynsim_edo_B.b_kstr_cu < 4;
         dynsim_edo_B.b_kstr_cu++) {
      for (dynsim_edo_B.e_i = 0; dynsim_edo_B.e_i < 4; dynsim_edo_B.e_i++) {
        dynsim_edo_B.d_k = dynsim_edo_B.e_i << 2;
        dynsim_edo_B.loop_ub_f = dynsim_edo_B.b_kstr_cu + dynsim_edo_B.d_k;
        dynsim_edo_B.a_l[dynsim_edo_B.loop_ub_f] = 0.0;
        dynsim_edo_B.a_l[dynsim_edo_B.loop_ub_f] +=
          dynsim_edo_B.c_f1_m[dynsim_edo_B.d_k] *
          dynsim_edo_B.a_n[dynsim_edo_B.b_kstr_cu];
        dynsim_edo_B.a_l[dynsim_edo_B.loop_ub_f] +=
          dynsim_edo_B.c_f1_m[dynsim_edo_B.d_k + 1] *
          dynsim_edo_B.a_n[dynsim_edo_B.b_kstr_cu + 4];
        dynsim_edo_B.a_l[dynsim_edo_B.loop_ub_f] +=
          dynsim_edo_B.c_f1_m[dynsim_edo_B.d_k + 2] *
          dynsim_edo_B.a_n[dynsim_edo_B.b_kstr_cu + 8];
        dynsim_edo_B.a_l[dynsim_edo_B.loop_ub_f] +=
          dynsim_edo_B.c_f1_m[dynsim_edo_B.d_k + 3] *
          dynsim_edo_B.a_n[dynsim_edo_B.b_kstr_cu + 12];
      }

      for (dynsim_edo_B.e_i = 0; dynsim_edo_B.e_i < 4; dynsim_edo_B.e_i++) {
        dynsim_edo_B.d_k = dynsim_edo_B.e_i << 2;
        dynsim_edo_B.loop_ub_f = dynsim_edo_B.b_kstr_cu + dynsim_edo_B.d_k;
        Ttree->data[dynsim_edo_B.b_jtilecol_m].f1[dynsim_edo_B.loop_ub_f] = 0.0;
        Ttree->data[dynsim_edo_B.b_jtilecol_m].f1[dynsim_edo_B.loop_ub_f] +=
          dynsim_edo_B.b_p[dynsim_edo_B.d_k] *
          dynsim_edo_B.a_l[dynsim_edo_B.b_kstr_cu];
        Ttree->data[dynsim_edo_B.b_jtilecol_m].f1[dynsim_edo_B.loop_ub_f] +=
          dynsim_edo_B.b_p[dynsim_edo_B.d_k + 1] *
          dynsim_edo_B.a_l[dynsim_edo_B.b_kstr_cu + 4];
        Ttree->data[dynsim_edo_B.b_jtilecol_m].f1[dynsim_edo_B.loop_ub_f] +=
          dynsim_edo_B.b_p[dynsim_edo_B.d_k + 2] *
          dynsim_edo_B.a_l[dynsim_edo_B.b_kstr_cu + 8];
        Ttree->data[dynsim_edo_B.b_jtilecol_m].f1[dynsim_edo_B.loop_ub_f] +=
          dynsim_edo_B.b_p[dynsim_edo_B.d_k + 3] *
          dynsim_edo_B.a_l[dynsim_edo_B.b_kstr_cu + 12];
      }
    }

    dynsim_edo_B.k_m = dynsim_edo_B.n_l;
    if (body->ParentIndex > 0.0) {
      for (dynsim_edo_B.b_kstr_cu = 0; dynsim_edo_B.b_kstr_cu < 16;
           dynsim_edo_B.b_kstr_cu++) {
        dynsim_edo_B.a_n[dynsim_edo_B.b_kstr_cu] = Ttree->data
          [static_cast<int32_T>(body->ParentIndex) - 1]
          .f1[dynsim_edo_B.b_kstr_cu];
      }

      for (dynsim_edo_B.b_kstr_cu = 0; dynsim_edo_B.b_kstr_cu < 4;
           dynsim_edo_B.b_kstr_cu++) {
        for (dynsim_edo_B.e_i = 0; dynsim_edo_B.e_i < 4; dynsim_edo_B.e_i++) {
          dynsim_edo_B.d_k = dynsim_edo_B.e_i << 2;
          dynsim_edo_B.loop_ub_f = dynsim_edo_B.b_kstr_cu + dynsim_edo_B.d_k;
          dynsim_edo_B.a_l[dynsim_edo_B.loop_ub_f] = 0.0;
          dynsim_edo_B.a_l[dynsim_edo_B.loop_ub_f] += Ttree->
            data[dynsim_edo_B.b_jtilecol_m].f1[dynsim_edo_B.d_k] *
            dynsim_edo_B.a_n[dynsim_edo_B.b_kstr_cu];
          dynsim_edo_B.a_l[dynsim_edo_B.loop_ub_f] += Ttree->
            data[dynsim_edo_B.b_jtilecol_m].f1[dynsim_edo_B.d_k + 1] *
            dynsim_edo_B.a_n[dynsim_edo_B.b_kstr_cu + 4];
          dynsim_edo_B.a_l[dynsim_edo_B.loop_ub_f] += Ttree->
            data[dynsim_edo_B.b_jtilecol_m].f1[dynsim_edo_B.d_k + 2] *
            dynsim_edo_B.a_n[dynsim_edo_B.b_kstr_cu + 8];
          dynsim_edo_B.a_l[dynsim_edo_B.loop_ub_f] += Ttree->
            data[dynsim_edo_B.b_jtilecol_m].f1[dynsim_edo_B.d_k + 3] *
            dynsim_edo_B.a_n[dynsim_edo_B.b_kstr_cu + 12];
        }
      }

      for (dynsim_edo_B.b_kstr_cu = 0; dynsim_edo_B.b_kstr_cu < 16;
           dynsim_edo_B.b_kstr_cu++) {
        Ttree->data[dynsim_edo_B.b_jtilecol_m].f1[dynsim_edo_B.b_kstr_cu] =
          dynsim_edo_B.a_l[dynsim_edo_B.b_kstr_cu];
      }
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
}

static void dynsim_edo_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[128], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_dynsim_edo_std_msgs_MultiArrayDimension varargout_2_Layout_Dim[16],
  uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0)
{
  *varargout_1 = Sub_dynsim_edo_16.getLatestMessage(&dynsim_edo_B.b_varargout_2);
  memcpy(&varargout_2_Data[0], &dynsim_edo_B.b_varargout_2.Data[0], sizeof
         (real_T) << 7U);
  *varargout_2_Data_SL_Info_Curren =
    dynsim_edo_B.b_varargout_2.Data_SL_Info.CurrentLength;
  *varargout_2_Data_SL_Info_Receiv =
    dynsim_edo_B.b_varargout_2.Data_SL_Info.ReceivedLength;
  *varargout_2_Layout_DataOffset = dynsim_edo_B.b_varargout_2.Layout.DataOffset;
  memcpy(&varargout_2_Layout_Dim[0], &dynsim_edo_B.b_varargout_2.Layout.Dim[0],
         sizeof(SL_Bus_dynsim_edo_std_msgs_MultiArrayDimension) << 4U);
  *varargout_2_Layout_Dim_SL_Info_ =
    dynsim_edo_B.b_varargout_2.Layout.Dim_SL_Info.CurrentLength;
  *varargout_2_Layout_Dim_SL_Inf_0 =
    dynsim_edo_B.b_varargout_2.Layout.Dim_SL_Info.ReceivedLength;
}

static void dynsim_edo_emxInit_f_cell_wrap1(emxArray_f_cell_wrap_dynsim_h_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_cell_wrap_dynsim_h_T *emxArray;
  *pEmxArray = (emxArray_f_cell_wrap_dynsim_h_T *)malloc(sizeof
    (emxArray_f_cell_wrap_dynsim_h_T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_dynsim_edo_h_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (dynsim_edo_B.i_e = 0; dynsim_edo_B.i_e < numDimensions; dynsim_edo_B.i_e
       ++) {
    emxArray->size[dynsim_edo_B.i_e] = 0;
  }
}

static void emxEnsureCapacity_f_cell_wrap1(emxArray_f_cell_wrap_dynsim_h_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  dynsim_edo_B.newNumel_n = 1;
  for (dynsim_edo_B.i_i = 0; dynsim_edo_B.i_i < emxArray->numDimensions;
       dynsim_edo_B.i_i++) {
    dynsim_edo_B.newNumel_n *= emxArray->size[dynsim_edo_B.i_i];
  }

  if (dynsim_edo_B.newNumel_n > emxArray->allocatedSize) {
    dynsim_edo_B.i_i = emxArray->allocatedSize;
    if (dynsim_edo_B.i_i < 16) {
      dynsim_edo_B.i_i = 16;
    }

    while (dynsim_edo_B.i_i < dynsim_edo_B.newNumel_n) {
      if (dynsim_edo_B.i_i > 1073741823) {
        dynsim_edo_B.i_i = MAX_int32_T;
      } else {
        dynsim_edo_B.i_i <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(dynsim_edo_B.i_i), sizeof
                     (f_cell_wrap_dynsim_edo_h_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_dynsim_edo_h_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_dynsim_edo_h_T *)newData;
    emxArray->allocatedSize = dynsim_edo_B.i_i;
    emxArray->canFreeData = true;
  }
}

static void rigidBodyJoint_get_JointAxis_ha(const
  c_rigidBodyJoint_dynsim_ed_ha_T *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (dynsim_edo_B.b_kstr_d = 0; dynsim_edo_B.b_kstr_d < 8;
       dynsim_edo_B.b_kstr_d++) {
    dynsim_edo_B.b_c0[dynsim_edo_B.b_kstr_d] = tmp[dynsim_edo_B.b_kstr_d];
  }

  dynsim_edo_B.b_bool_c = false;
  if (obj->Type->size[1] == 8) {
    dynsim_edo_B.b_kstr_d = 1;
    do {
      exitg1 = 0;
      if (dynsim_edo_B.b_kstr_d - 1 < 8) {
        dynsim_edo_B.kstr_a = dynsim_edo_B.b_kstr_d - 1;
        if (obj->Type->data[dynsim_edo_B.kstr_a] !=
            dynsim_edo_B.b_c0[dynsim_edo_B.kstr_a]) {
          exitg1 = 1;
        } else {
          dynsim_edo_B.b_kstr_d++;
        }
      } else {
        dynsim_edo_B.b_bool_c = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (dynsim_edo_B.b_bool_c) {
    guard1 = true;
  } else {
    for (dynsim_edo_B.b_kstr_d = 0; dynsim_edo_B.b_kstr_d < 9;
         dynsim_edo_B.b_kstr_d++) {
      dynsim_edo_B.b_m[dynsim_edo_B.b_kstr_d] = tmp_0[dynsim_edo_B.b_kstr_d];
    }

    dynsim_edo_B.b_bool_c = false;
    if (obj->Type->size[1] == 9) {
      dynsim_edo_B.b_kstr_d = 1;
      do {
        exitg1 = 0;
        if (dynsim_edo_B.b_kstr_d - 1 < 9) {
          dynsim_edo_B.kstr_a = dynsim_edo_B.b_kstr_d - 1;
          if (obj->Type->data[dynsim_edo_B.kstr_a] !=
              dynsim_edo_B.b_m[dynsim_edo_B.kstr_a]) {
            exitg1 = 1;
          } else {
            dynsim_edo_B.b_kstr_d++;
          }
        } else {
          dynsim_edo_B.b_bool_c = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_edo_B.b_bool_c) {
      guard1 = true;
    } else {
      ax[0] = (rtNaN);
      ax[1] = (rtNaN);
      ax[2] = (rtNaN);
    }
  }

  if (guard1) {
    ax[0] = obj->JointAxisInternal[0];
    ax[1] = obj->JointAxisInternal[1];
    ax[2] = obj->JointAxisInternal[2];
  }
}

static void dynsim_edo_cat(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9])
{
  y[0] = varargin_1;
  y[1] = varargin_2;
  y[2] = varargin_3;
  y[3] = varargin_4;
  y[4] = varargin_5;
  y[5] = varargin_6;
  y[6] = varargin_7;
  y[7] = varargin_8;
  y[8] = varargin_9;
}

static void rigidBodyJoint_transformBodyT_h(const
  c_rigidBodyJoint_dynsim_ed_ha_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (dynsim_edo_B.b_kstr_c = 0; dynsim_edo_B.b_kstr_c < 5;
       dynsim_edo_B.b_kstr_c++) {
    dynsim_edo_B.b_cs[dynsim_edo_B.b_kstr_c] = tmp[dynsim_edo_B.b_kstr_c];
  }

  dynsim_edo_B.b_bool_na = false;
  if (obj->Type->size[1] == 5) {
    dynsim_edo_B.b_kstr_c = 1;
    do {
      exitg1 = 0;
      if (dynsim_edo_B.b_kstr_c - 1 < 5) {
        dynsim_edo_B.kstr_e = dynsim_edo_B.b_kstr_c - 1;
        if (obj->Type->data[dynsim_edo_B.kstr_e] !=
            dynsim_edo_B.b_cs[dynsim_edo_B.kstr_e]) {
          exitg1 = 1;
        } else {
          dynsim_edo_B.b_kstr_c++;
        }
      } else {
        dynsim_edo_B.b_bool_na = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (dynsim_edo_B.b_bool_na) {
    dynsim_edo_B.b_kstr_c = 0;
  } else {
    for (dynsim_edo_B.b_kstr_c = 0; dynsim_edo_B.b_kstr_c < 8;
         dynsim_edo_B.b_kstr_c++) {
      dynsim_edo_B.b_h[dynsim_edo_B.b_kstr_c] = tmp_0[dynsim_edo_B.b_kstr_c];
    }

    dynsim_edo_B.b_bool_na = false;
    if (obj->Type->size[1] == 8) {
      dynsim_edo_B.b_kstr_c = 1;
      do {
        exitg1 = 0;
        if (dynsim_edo_B.b_kstr_c - 1 < 8) {
          dynsim_edo_B.kstr_e = dynsim_edo_B.b_kstr_c - 1;
          if (obj->Type->data[dynsim_edo_B.kstr_e] !=
              dynsim_edo_B.b_h[dynsim_edo_B.kstr_e]) {
            exitg1 = 1;
          } else {
            dynsim_edo_B.b_kstr_c++;
          }
        } else {
          dynsim_edo_B.b_bool_na = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_edo_B.b_bool_na) {
      dynsim_edo_B.b_kstr_c = 1;
    } else {
      dynsim_edo_B.b_kstr_c = -1;
    }
  }

  switch (dynsim_edo_B.b_kstr_c) {
   case 0:
    memset(&dynsim_edo_B.TJ[0], 0, sizeof(real_T) << 4U);
    dynsim_edo_B.TJ[0] = 1.0;
    dynsim_edo_B.TJ[5] = 1.0;
    dynsim_edo_B.TJ[10] = 1.0;
    dynsim_edo_B.TJ[15] = 1.0;
    break;

   case 1:
    rigidBodyJoint_get_JointAxis_ha(obj, dynsim_edo_B.v_j);
    dynsim_edo_B.result_data_j[0] = dynsim_edo_B.v_j[0];
    dynsim_edo_B.result_data_j[1] = dynsim_edo_B.v_j[1];
    dynsim_edo_B.result_data_j[2] = dynsim_edo_B.v_j[2];
    if (0 <= (*q_size != 0) - 1) {
      dynsim_edo_B.result_data_j[3] = q_data[0];
    }

    dynsim_edo_B.cth = 1.0 / sqrt((dynsim_edo_B.result_data_j[0] *
      dynsim_edo_B.result_data_j[0] + dynsim_edo_B.result_data_j[1] *
      dynsim_edo_B.result_data_j[1]) + dynsim_edo_B.result_data_j[2] *
      dynsim_edo_B.result_data_j[2]);
    dynsim_edo_B.v_j[0] = dynsim_edo_B.result_data_j[0] * dynsim_edo_B.cth;
    dynsim_edo_B.v_j[1] = dynsim_edo_B.result_data_j[1] * dynsim_edo_B.cth;
    dynsim_edo_B.v_j[2] = dynsim_edo_B.result_data_j[2] * dynsim_edo_B.cth;
    dynsim_edo_B.cth = cos(dynsim_edo_B.result_data_j[3]);
    dynsim_edo_B.sth_i = sin(dynsim_edo_B.result_data_j[3]);
    dynsim_edo_B.tempR_tmp_l = dynsim_edo_B.v_j[1] * dynsim_edo_B.v_j[0] * (1.0
      - dynsim_edo_B.cth);
    dynsim_edo_B.tempR_tmp_o = dynsim_edo_B.v_j[2] * dynsim_edo_B.sth_i;
    dynsim_edo_B.tempR_tmp_o2 = dynsim_edo_B.v_j[2] * dynsim_edo_B.v_j[0] * (1.0
      - dynsim_edo_B.cth);
    dynsim_edo_B.tempR_tmp_i = dynsim_edo_B.v_j[1] * dynsim_edo_B.sth_i;
    dynsim_edo_B.tempR_tmp_f = dynsim_edo_B.v_j[2] * dynsim_edo_B.v_j[1] * (1.0
      - dynsim_edo_B.cth);
    dynsim_edo_B.sth_i *= dynsim_edo_B.v_j[0];
    dynsim_edo_cat(dynsim_edo_B.v_j[0] * dynsim_edo_B.v_j[0] * (1.0 -
      dynsim_edo_B.cth) + dynsim_edo_B.cth, dynsim_edo_B.tempR_tmp_l -
                   dynsim_edo_B.tempR_tmp_o, dynsim_edo_B.tempR_tmp_o2 +
                   dynsim_edo_B.tempR_tmp_i, dynsim_edo_B.tempR_tmp_l +
                   dynsim_edo_B.tempR_tmp_o, dynsim_edo_B.v_j[1] *
                   dynsim_edo_B.v_j[1] * (1.0 - dynsim_edo_B.cth) +
                   dynsim_edo_B.cth, dynsim_edo_B.tempR_tmp_f -
                   dynsim_edo_B.sth_i, dynsim_edo_B.tempR_tmp_o2 -
                   dynsim_edo_B.tempR_tmp_i, dynsim_edo_B.tempR_tmp_f +
                   dynsim_edo_B.sth_i, dynsim_edo_B.v_j[2] * dynsim_edo_B.v_j[2]
                   * (1.0 - dynsim_edo_B.cth) + dynsim_edo_B.cth,
                   dynsim_edo_B.tempR_b);
    for (dynsim_edo_B.b_kstr_c = 0; dynsim_edo_B.b_kstr_c < 3;
         dynsim_edo_B.b_kstr_c++) {
      dynsim_edo_B.kstr_e = dynsim_edo_B.b_kstr_c + 1;
      dynsim_edo_B.R_o[dynsim_edo_B.kstr_e - 1] = dynsim_edo_B.tempR_b
        [(dynsim_edo_B.kstr_e - 1) * 3];
      dynsim_edo_B.kstr_e = dynsim_edo_B.b_kstr_c + 1;
      dynsim_edo_B.R_o[dynsim_edo_B.kstr_e + 2] = dynsim_edo_B.tempR_b
        [(dynsim_edo_B.kstr_e - 1) * 3 + 1];
      dynsim_edo_B.kstr_e = dynsim_edo_B.b_kstr_c + 1;
      dynsim_edo_B.R_o[dynsim_edo_B.kstr_e + 5] = dynsim_edo_B.tempR_b
        [(dynsim_edo_B.kstr_e - 1) * 3 + 2];
    }

    memset(&dynsim_edo_B.TJ[0], 0, sizeof(real_T) << 4U);
    for (dynsim_edo_B.b_kstr_c = 0; dynsim_edo_B.b_kstr_c < 3;
         dynsim_edo_B.b_kstr_c++) {
      dynsim_edo_B.kstr_e = dynsim_edo_B.b_kstr_c << 2;
      dynsim_edo_B.TJ[dynsim_edo_B.kstr_e] = dynsim_edo_B.R_o[3 *
        dynsim_edo_B.b_kstr_c];
      dynsim_edo_B.TJ[dynsim_edo_B.kstr_e + 1] = dynsim_edo_B.R_o[3 *
        dynsim_edo_B.b_kstr_c + 1];
      dynsim_edo_B.TJ[dynsim_edo_B.kstr_e + 2] = dynsim_edo_B.R_o[3 *
        dynsim_edo_B.b_kstr_c + 2];
    }

    dynsim_edo_B.TJ[15] = 1.0;
    break;

   default:
    rigidBodyJoint_get_JointAxis_ha(obj, dynsim_edo_B.v_j);
    memset(&dynsim_edo_B.tempR_b[0], 0, 9U * sizeof(real_T));
    dynsim_edo_B.tempR_b[0] = 1.0;
    dynsim_edo_B.tempR_b[4] = 1.0;
    dynsim_edo_B.tempR_b[8] = 1.0;
    for (dynsim_edo_B.b_kstr_c = 0; dynsim_edo_B.b_kstr_c < 3;
         dynsim_edo_B.b_kstr_c++) {
      dynsim_edo_B.kstr_e = dynsim_edo_B.b_kstr_c << 2;
      dynsim_edo_B.TJ[dynsim_edo_B.kstr_e] = dynsim_edo_B.tempR_b[3 *
        dynsim_edo_B.b_kstr_c];
      dynsim_edo_B.TJ[dynsim_edo_B.kstr_e + 1] = dynsim_edo_B.tempR_b[3 *
        dynsim_edo_B.b_kstr_c + 1];
      dynsim_edo_B.TJ[dynsim_edo_B.kstr_e + 2] = dynsim_edo_B.tempR_b[3 *
        dynsim_edo_B.b_kstr_c + 2];
      dynsim_edo_B.TJ[dynsim_edo_B.b_kstr_c + 12] =
        dynsim_edo_B.v_j[dynsim_edo_B.b_kstr_c] * q_data[0];
    }

    dynsim_edo_B.TJ[3] = 0.0;
    dynsim_edo_B.TJ[7] = 0.0;
    dynsim_edo_B.TJ[11] = 0.0;
    dynsim_edo_B.TJ[15] = 1.0;
    break;
  }

  for (dynsim_edo_B.b_kstr_c = 0; dynsim_edo_B.b_kstr_c < 4;
       dynsim_edo_B.b_kstr_c++) {
    for (dynsim_edo_B.kstr_e = 0; dynsim_edo_B.kstr_e < 4; dynsim_edo_B.kstr_e++)
    {
      dynsim_edo_B.obj_tmp_tmp = dynsim_edo_B.kstr_e << 2;
      dynsim_edo_B.obj_tmp = dynsim_edo_B.b_kstr_c + dynsim_edo_B.obj_tmp_tmp;
      dynsim_edo_B.obj[dynsim_edo_B.obj_tmp] = 0.0;
      dynsim_edo_B.obj[dynsim_edo_B.obj_tmp] +=
        dynsim_edo_B.TJ[dynsim_edo_B.obj_tmp_tmp] * obj->
        JointToParentTransform[dynsim_edo_B.b_kstr_c];
      dynsim_edo_B.obj[dynsim_edo_B.obj_tmp] +=
        dynsim_edo_B.TJ[dynsim_edo_B.obj_tmp_tmp + 1] *
        obj->JointToParentTransform[dynsim_edo_B.b_kstr_c + 4];
      dynsim_edo_B.obj[dynsim_edo_B.obj_tmp] +=
        dynsim_edo_B.TJ[dynsim_edo_B.obj_tmp_tmp + 2] *
        obj->JointToParentTransform[dynsim_edo_B.b_kstr_c + 8];
      dynsim_edo_B.obj[dynsim_edo_B.obj_tmp] +=
        dynsim_edo_B.TJ[dynsim_edo_B.obj_tmp_tmp + 3] *
        obj->JointToParentTransform[dynsim_edo_B.b_kstr_c + 12];
    }

    for (dynsim_edo_B.kstr_e = 0; dynsim_edo_B.kstr_e < 4; dynsim_edo_B.kstr_e++)
    {
      dynsim_edo_B.obj_tmp_tmp = dynsim_edo_B.kstr_e << 2;
      dynsim_edo_B.obj_tmp = dynsim_edo_B.b_kstr_c + dynsim_edo_B.obj_tmp_tmp;
      T[dynsim_edo_B.obj_tmp] = 0.0;
      T[dynsim_edo_B.obj_tmp] += obj->
        ChildToJointTransform[dynsim_edo_B.obj_tmp_tmp] *
        dynsim_edo_B.obj[dynsim_edo_B.b_kstr_c];
      T[dynsim_edo_B.obj_tmp] += obj->
        ChildToJointTransform[dynsim_edo_B.obj_tmp_tmp + 1] *
        dynsim_edo_B.obj[dynsim_edo_B.b_kstr_c + 4];
      T[dynsim_edo_B.obj_tmp] += obj->
        ChildToJointTransform[dynsim_edo_B.obj_tmp_tmp + 2] *
        dynsim_edo_B.obj[dynsim_edo_B.b_kstr_c + 8];
      T[dynsim_edo_B.obj_tmp] += obj->
        ChildToJointTransform[dynsim_edo_B.obj_tmp_tmp + 3] *
        dynsim_edo_B.obj[dynsim_edo_B.b_kstr_c + 12];
    }
  }
}

static void rigidBodyJoint_transformBodyToP(const
  c_rigidBodyJoint_dynsim_ed_ha_T *obj, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (dynsim_edo_B.b_kstr_f = 0; dynsim_edo_B.b_kstr_f < 5;
       dynsim_edo_B.b_kstr_f++) {
    dynsim_edo_B.b_pc[dynsim_edo_B.b_kstr_f] = tmp[dynsim_edo_B.b_kstr_f];
  }

  dynsim_edo_B.b_bool_p = false;
  if (obj->Type->size[1] == 5) {
    dynsim_edo_B.b_kstr_f = 1;
    do {
      exitg1 = 0;
      if (dynsim_edo_B.b_kstr_f - 1 < 5) {
        dynsim_edo_B.kstr_p = dynsim_edo_B.b_kstr_f - 1;
        if (obj->Type->data[dynsim_edo_B.kstr_p] !=
            dynsim_edo_B.b_pc[dynsim_edo_B.kstr_p]) {
          exitg1 = 1;
        } else {
          dynsim_edo_B.b_kstr_f++;
        }
      } else {
        dynsim_edo_B.b_bool_p = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (dynsim_edo_B.b_bool_p) {
    dynsim_edo_B.b_kstr_f = 0;
  } else {
    for (dynsim_edo_B.b_kstr_f = 0; dynsim_edo_B.b_kstr_f < 8;
         dynsim_edo_B.b_kstr_f++) {
      dynsim_edo_B.b_ct[dynsim_edo_B.b_kstr_f] = tmp_0[dynsim_edo_B.b_kstr_f];
    }

    dynsim_edo_B.b_bool_p = false;
    if (obj->Type->size[1] == 8) {
      dynsim_edo_B.b_kstr_f = 1;
      do {
        exitg1 = 0;
        if (dynsim_edo_B.b_kstr_f - 1 < 8) {
          dynsim_edo_B.kstr_p = dynsim_edo_B.b_kstr_f - 1;
          if (obj->Type->data[dynsim_edo_B.kstr_p] !=
              dynsim_edo_B.b_ct[dynsim_edo_B.kstr_p]) {
            exitg1 = 1;
          } else {
            dynsim_edo_B.b_kstr_f++;
          }
        } else {
          dynsim_edo_B.b_bool_p = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_edo_B.b_bool_p) {
      dynsim_edo_B.b_kstr_f = 1;
    } else {
      dynsim_edo_B.b_kstr_f = -1;
    }
  }

  switch (dynsim_edo_B.b_kstr_f) {
   case 0:
    memset(&dynsim_edo_B.TJ_g[0], 0, sizeof(real_T) << 4U);
    dynsim_edo_B.TJ_g[0] = 1.0;
    dynsim_edo_B.TJ_g[5] = 1.0;
    dynsim_edo_B.TJ_g[10] = 1.0;
    dynsim_edo_B.TJ_g[15] = 1.0;
    break;

   case 1:
    rigidBodyJoint_get_JointAxis_ha(obj, dynsim_edo_B.v_o);
    dynsim_edo_B.axang_idx_0 = dynsim_edo_B.v_o[0];
    dynsim_edo_B.axang_idx_1 = dynsim_edo_B.v_o[1];
    dynsim_edo_B.axang_idx_2 = dynsim_edo_B.v_o[2];
    dynsim_edo_B.b_o = 1.0 / sqrt((dynsim_edo_B.axang_idx_0 *
      dynsim_edo_B.axang_idx_0 + dynsim_edo_B.axang_idx_1 *
      dynsim_edo_B.axang_idx_1) + dynsim_edo_B.axang_idx_2 *
      dynsim_edo_B.axang_idx_2);
    dynsim_edo_B.v_o[0] = dynsim_edo_B.axang_idx_0 * dynsim_edo_B.b_o;
    dynsim_edo_B.v_o[1] = dynsim_edo_B.axang_idx_1 * dynsim_edo_B.b_o;
    dynsim_edo_B.v_o[2] = dynsim_edo_B.axang_idx_2 * dynsim_edo_B.b_o;
    dynsim_edo_B.axang_idx_0 = dynsim_edo_B.v_o[1] * dynsim_edo_B.v_o[0] * 0.0;
    dynsim_edo_B.axang_idx_1 = dynsim_edo_B.v_o[2] * dynsim_edo_B.v_o[0] * 0.0;
    dynsim_edo_B.axang_idx_2 = dynsim_edo_B.v_o[2] * dynsim_edo_B.v_o[1] * 0.0;
    dynsim_edo_cat(dynsim_edo_B.v_o[0] * dynsim_edo_B.v_o[0] * 0.0 + 1.0,
                   dynsim_edo_B.axang_idx_0 - dynsim_edo_B.v_o[2] * 0.0,
                   dynsim_edo_B.axang_idx_1 + dynsim_edo_B.v_o[1] * 0.0,
                   dynsim_edo_B.axang_idx_0 + dynsim_edo_B.v_o[2] * 0.0,
                   dynsim_edo_B.v_o[1] * dynsim_edo_B.v_o[1] * 0.0 + 1.0,
                   dynsim_edo_B.axang_idx_2 - dynsim_edo_B.v_o[0] * 0.0,
                   dynsim_edo_B.axang_idx_1 - dynsim_edo_B.v_o[1] * 0.0,
                   dynsim_edo_B.axang_idx_2 + dynsim_edo_B.v_o[0] * 0.0,
                   dynsim_edo_B.v_o[2] * dynsim_edo_B.v_o[2] * 0.0 + 1.0,
                   dynsim_edo_B.tempR_bs);
    for (dynsim_edo_B.b_kstr_f = 0; dynsim_edo_B.b_kstr_f < 3;
         dynsim_edo_B.b_kstr_f++) {
      dynsim_edo_B.kstr_p = dynsim_edo_B.b_kstr_f + 1;
      dynsim_edo_B.R_n[dynsim_edo_B.kstr_p - 1] = dynsim_edo_B.tempR_bs
        [(dynsim_edo_B.kstr_p - 1) * 3];
      dynsim_edo_B.kstr_p = dynsim_edo_B.b_kstr_f + 1;
      dynsim_edo_B.R_n[dynsim_edo_B.kstr_p + 2] = dynsim_edo_B.tempR_bs
        [(dynsim_edo_B.kstr_p - 1) * 3 + 1];
      dynsim_edo_B.kstr_p = dynsim_edo_B.b_kstr_f + 1;
      dynsim_edo_B.R_n[dynsim_edo_B.kstr_p + 5] = dynsim_edo_B.tempR_bs
        [(dynsim_edo_B.kstr_p - 1) * 3 + 2];
    }

    memset(&dynsim_edo_B.TJ_g[0], 0, sizeof(real_T) << 4U);
    for (dynsim_edo_B.b_kstr_f = 0; dynsim_edo_B.b_kstr_f < 3;
         dynsim_edo_B.b_kstr_f++) {
      dynsim_edo_B.kstr_p = dynsim_edo_B.b_kstr_f << 2;
      dynsim_edo_B.TJ_g[dynsim_edo_B.kstr_p] = dynsim_edo_B.R_n[3 *
        dynsim_edo_B.b_kstr_f];
      dynsim_edo_B.TJ_g[dynsim_edo_B.kstr_p + 1] = dynsim_edo_B.R_n[3 *
        dynsim_edo_B.b_kstr_f + 1];
      dynsim_edo_B.TJ_g[dynsim_edo_B.kstr_p + 2] = dynsim_edo_B.R_n[3 *
        dynsim_edo_B.b_kstr_f + 2];
    }

    dynsim_edo_B.TJ_g[15] = 1.0;
    break;

   default:
    rigidBodyJoint_get_JointAxis_ha(obj, dynsim_edo_B.v_o);
    memset(&dynsim_edo_B.tempR_bs[0], 0, 9U * sizeof(real_T));
    dynsim_edo_B.tempR_bs[0] = 1.0;
    dynsim_edo_B.tempR_bs[4] = 1.0;
    dynsim_edo_B.tempR_bs[8] = 1.0;
    for (dynsim_edo_B.b_kstr_f = 0; dynsim_edo_B.b_kstr_f < 3;
         dynsim_edo_B.b_kstr_f++) {
      dynsim_edo_B.kstr_p = dynsim_edo_B.b_kstr_f << 2;
      dynsim_edo_B.TJ_g[dynsim_edo_B.kstr_p] = dynsim_edo_B.tempR_bs[3 *
        dynsim_edo_B.b_kstr_f];
      dynsim_edo_B.TJ_g[dynsim_edo_B.kstr_p + 1] = dynsim_edo_B.tempR_bs[3 *
        dynsim_edo_B.b_kstr_f + 1];
      dynsim_edo_B.TJ_g[dynsim_edo_B.kstr_p + 2] = dynsim_edo_B.tempR_bs[3 *
        dynsim_edo_B.b_kstr_f + 2];
      dynsim_edo_B.TJ_g[dynsim_edo_B.b_kstr_f + 12] =
        dynsim_edo_B.v_o[dynsim_edo_B.b_kstr_f] * 0.0;
    }

    dynsim_edo_B.TJ_g[3] = 0.0;
    dynsim_edo_B.TJ_g[7] = 0.0;
    dynsim_edo_B.TJ_g[11] = 0.0;
    dynsim_edo_B.TJ_g[15] = 1.0;
    break;
  }

  for (dynsim_edo_B.b_kstr_f = 0; dynsim_edo_B.b_kstr_f < 4;
       dynsim_edo_B.b_kstr_f++) {
    for (dynsim_edo_B.kstr_p = 0; dynsim_edo_B.kstr_p < 4; dynsim_edo_B.kstr_p++)
    {
      dynsim_edo_B.obj_tmp_tmp_o = dynsim_edo_B.kstr_p << 2;
      dynsim_edo_B.obj_tmp_i = dynsim_edo_B.b_kstr_f +
        dynsim_edo_B.obj_tmp_tmp_o;
      dynsim_edo_B.obj_g[dynsim_edo_B.obj_tmp_i] = 0.0;
      dynsim_edo_B.obj_g[dynsim_edo_B.obj_tmp_i] +=
        dynsim_edo_B.TJ_g[dynsim_edo_B.obj_tmp_tmp_o] *
        obj->JointToParentTransform[dynsim_edo_B.b_kstr_f];
      dynsim_edo_B.obj_g[dynsim_edo_B.obj_tmp_i] +=
        dynsim_edo_B.TJ_g[dynsim_edo_B.obj_tmp_tmp_o + 1] *
        obj->JointToParentTransform[dynsim_edo_B.b_kstr_f + 4];
      dynsim_edo_B.obj_g[dynsim_edo_B.obj_tmp_i] +=
        dynsim_edo_B.TJ_g[dynsim_edo_B.obj_tmp_tmp_o + 2] *
        obj->JointToParentTransform[dynsim_edo_B.b_kstr_f + 8];
      dynsim_edo_B.obj_g[dynsim_edo_B.obj_tmp_i] +=
        dynsim_edo_B.TJ_g[dynsim_edo_B.obj_tmp_tmp_o + 3] *
        obj->JointToParentTransform[dynsim_edo_B.b_kstr_f + 12];
    }

    for (dynsim_edo_B.kstr_p = 0; dynsim_edo_B.kstr_p < 4; dynsim_edo_B.kstr_p++)
    {
      dynsim_edo_B.obj_tmp_tmp_o = dynsim_edo_B.kstr_p << 2;
      dynsim_edo_B.obj_tmp_i = dynsim_edo_B.b_kstr_f +
        dynsim_edo_B.obj_tmp_tmp_o;
      T[dynsim_edo_B.obj_tmp_i] = 0.0;
      T[dynsim_edo_B.obj_tmp_i] += obj->
        ChildToJointTransform[dynsim_edo_B.obj_tmp_tmp_o] *
        dynsim_edo_B.obj_g[dynsim_edo_B.b_kstr_f];
      T[dynsim_edo_B.obj_tmp_i] += obj->
        ChildToJointTransform[dynsim_edo_B.obj_tmp_tmp_o + 1] *
        dynsim_edo_B.obj_g[dynsim_edo_B.b_kstr_f + 4];
      T[dynsim_edo_B.obj_tmp_i] += obj->
        ChildToJointTransform[dynsim_edo_B.obj_tmp_tmp_o + 2] *
        dynsim_edo_B.obj_g[dynsim_edo_B.b_kstr_f + 8];
      T[dynsim_edo_B.obj_tmp_i] += obj->
        ChildToJointTransform[dynsim_edo_B.obj_tmp_tmp_o + 3] *
        dynsim_edo_B.obj_g[dynsim_edo_B.b_kstr_f + 12];
    }
  }
}

static void dynsim_edo_tforminv(const real_T T[16], real_T Tinv[16])
{
  for (dynsim_edo_B.i3 = 0; dynsim_edo_B.i3 < 3; dynsim_edo_B.i3++) {
    dynsim_edo_B.R_b[3 * dynsim_edo_B.i3] = T[dynsim_edo_B.i3];
    dynsim_edo_B.R_b[3 * dynsim_edo_B.i3 + 1] = T[dynsim_edo_B.i3 + 4];
    dynsim_edo_B.R_b[3 * dynsim_edo_B.i3 + 2] = T[dynsim_edo_B.i3 + 8];
  }

  for (dynsim_edo_B.i3 = 0; dynsim_edo_B.i3 < 9; dynsim_edo_B.i3++) {
    dynsim_edo_B.R_da[dynsim_edo_B.i3] = -dynsim_edo_B.R_b[dynsim_edo_B.i3];
  }

  for (dynsim_edo_B.i3 = 0; dynsim_edo_B.i3 < 3; dynsim_edo_B.i3++) {
    dynsim_edo_B.Tinv_tmp = dynsim_edo_B.i3 << 2;
    Tinv[dynsim_edo_B.Tinv_tmp] = dynsim_edo_B.R_b[3 * dynsim_edo_B.i3];
    Tinv[dynsim_edo_B.Tinv_tmp + 1] = dynsim_edo_B.R_b[3 * dynsim_edo_B.i3 + 1];
    Tinv[dynsim_edo_B.Tinv_tmp + 2] = dynsim_edo_B.R_b[3 * dynsim_edo_B.i3 + 2];
    Tinv[dynsim_edo_B.i3 + 12] = dynsim_edo_B.R_da[dynsim_edo_B.i3 + 6] * T[14]
      + (dynsim_edo_B.R_da[dynsim_edo_B.i3 + 3] * T[13] +
         dynsim_edo_B.R_da[dynsim_edo_B.i3] * T[12]);
  }

  Tinv[3] = 0.0;
  Tinv[7] = 0.0;
  Tinv[11] = 0.0;
  Tinv[15] = 1.0;
}

static void dynsim_edo_tformToSpatialXform(const real_T T[16], real_T X[36])
{
  dynsim_edo_B.dv2[0] = 0.0;
  dynsim_edo_B.dv2[3] = -T[14];
  dynsim_edo_B.dv2[6] = T[13];
  dynsim_edo_B.dv2[1] = T[14];
  dynsim_edo_B.dv2[4] = 0.0;
  dynsim_edo_B.dv2[7] = -T[12];
  dynsim_edo_B.dv2[2] = -T[13];
  dynsim_edo_B.dv2[5] = T[12];
  dynsim_edo_B.dv2[8] = 0.0;
  for (dynsim_edo_B.i1 = 0; dynsim_edo_B.i1 < 3; dynsim_edo_B.i1++) {
    for (dynsim_edo_B.X_tmp_a = 0; dynsim_edo_B.X_tmp_a < 3;
         dynsim_edo_B.X_tmp_a++) {
      dynsim_edo_B.X_tmp_k = dynsim_edo_B.i1 + 3 * dynsim_edo_B.X_tmp_a;
      dynsim_edo_B.dv3[dynsim_edo_B.X_tmp_k] = 0.0;
      dynsim_edo_B.i2 = dynsim_edo_B.X_tmp_a << 2;
      dynsim_edo_B.dv3[dynsim_edo_B.X_tmp_k] += T[dynsim_edo_B.i2] *
        dynsim_edo_B.dv2[dynsim_edo_B.i1];
      dynsim_edo_B.dv3[dynsim_edo_B.X_tmp_k] += T[dynsim_edo_B.i2 + 1] *
        dynsim_edo_B.dv2[dynsim_edo_B.i1 + 3];
      dynsim_edo_B.dv3[dynsim_edo_B.X_tmp_k] += T[dynsim_edo_B.i2 + 2] *
        dynsim_edo_B.dv2[dynsim_edo_B.i1 + 6];
      X[dynsim_edo_B.X_tmp_a + 6 * dynsim_edo_B.i1] = T[(dynsim_edo_B.i1 << 2) +
        dynsim_edo_B.X_tmp_a];
      X[dynsim_edo_B.X_tmp_a + 6 * (dynsim_edo_B.i1 + 3)] = 0.0;
    }
  }

  for (dynsim_edo_B.i1 = 0; dynsim_edo_B.i1 < 3; dynsim_edo_B.i1++) {
    X[6 * dynsim_edo_B.i1 + 3] = dynsim_edo_B.dv3[3 * dynsim_edo_B.i1];
    dynsim_edo_B.X_tmp_a = dynsim_edo_B.i1 << 2;
    dynsim_edo_B.X_tmp_k = 6 * (dynsim_edo_B.i1 + 3);
    X[dynsim_edo_B.X_tmp_k + 3] = T[dynsim_edo_B.X_tmp_a];
    X[6 * dynsim_edo_B.i1 + 4] = dynsim_edo_B.dv3[3 * dynsim_edo_B.i1 + 1];
    X[dynsim_edo_B.X_tmp_k + 4] = T[dynsim_edo_B.X_tmp_a + 1];
    X[6 * dynsim_edo_B.i1 + 5] = dynsim_edo_B.dv3[3 * dynsim_edo_B.i1 + 2];
    X[dynsim_edo_B.X_tmp_k + 5] = T[dynsim_edo_B.X_tmp_a + 2];
  }
}

static void dynsim_edo_emxFree_f_cell_wrap1(emxArray_f_cell_wrap_dynsim_h_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_cell_wrap_dynsim_h_T *)NULL) {
    if (((*pEmxArray)->data != (f_cell_wrap_dynsim_edo_h_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_cell_wrap_dynsim_h_T *)NULL;
  }
}

static void RigidBodyTreeDynamics_massMatri(n_robotics_manip_internal__ha_T
  *robot, const real_T q[3], emxArray_real_T_dynsim_edo_T *H,
  emxArray_real_T_dynsim_edo_T *lambda)
{
  emxArray_f_cell_wrap_dynsim_h_T *Ic;
  emxArray_f_cell_wrap_dynsim_h_T *X;
  emxArray_real_T_dynsim_edo_T *lambda_;
  emxArray_real_T_dynsim_edo_T *Si;
  emxArray_real_T_dynsim_edo_T *Fi;
  emxArray_real_T_dynsim_edo_T *Hji;
  emxArray_real_T_dynsim_edo_T *s;
  l_robotics_manip_internal__ha_T *obj;
  emxArray_char_T_dynsim_edo_T *a;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  boolean_T exitg1;
  int32_T exitg2;
  dynsim_edo_B.nb_i = robot->NumBodies;
  dynsim_edo_B.vNum = robot->VelocityNumber;
  dynsim_edo_B.f = H->size[0] * H->size[1];
  dynsim_edo_B.b_i_a = static_cast<int32_T>(dynsim_edo_B.vNum);
  H->size[0] = dynsim_edo_B.b_i_a;
  H->size[1] = dynsim_edo_B.b_i_a;
  dynsim_emxEnsureCapacity_real_T(H, dynsim_edo_B.f);
  dynsim_edo_B.loop_ub_n = dynsim_edo_B.b_i_a * dynsim_edo_B.b_i_a - 1;
  for (dynsim_edo_B.f = 0; dynsim_edo_B.f <= dynsim_edo_B.loop_ub_n;
       dynsim_edo_B.f++) {
    H->data[dynsim_edo_B.f] = 0.0;
  }

  dynsim_edo_emxInit_real_T(&lambda_, 2);
  dynsim_edo_B.f = lambda_->size[0] * lambda_->size[1];
  lambda_->size[0] = 1;
  dynsim_edo_B.nm1d2 = static_cast<int32_T>(dynsim_edo_B.nb_i);
  lambda_->size[1] = dynsim_edo_B.nm1d2;
  dynsim_emxEnsureCapacity_real_T(lambda_, dynsim_edo_B.f);
  dynsim_edo_B.idx = dynsim_edo_B.nm1d2 - 1;
  for (dynsim_edo_B.f = 0; dynsim_edo_B.f <= dynsim_edo_B.idx; dynsim_edo_B.f++)
  {
    lambda_->data[dynsim_edo_B.f] = 0.0;
  }

  dynsim_edo_B.f = lambda->size[0] * lambda->size[1];
  lambda->size[0] = 1;
  lambda->size[1] = dynsim_edo_B.b_i_a;
  dynsim_emxEnsureCapacity_real_T(lambda, dynsim_edo_B.f);
  dynsim_edo_B.loop_ub_n = dynsim_edo_B.b_i_a - 1;
  for (dynsim_edo_B.f = 0; dynsim_edo_B.f <= dynsim_edo_B.loop_ub_n;
       dynsim_edo_B.f++) {
    lambda->data[dynsim_edo_B.f] = 0.0;
  }

  dynsim_edo_emxInit_f_cell_wrap1(&Ic, 2);
  dynsim_edo_emxInit_f_cell_wrap1(&X, 2);
  dynsim_edo_B.f = Ic->size[0] * Ic->size[1];
  Ic->size[0] = 1;
  Ic->size[1] = dynsim_edo_B.nm1d2;
  emxEnsureCapacity_f_cell_wrap1(Ic, dynsim_edo_B.f);
  dynsim_edo_B.f = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = dynsim_edo_B.nm1d2;
  emxEnsureCapacity_f_cell_wrap1(X, dynsim_edo_B.f);
  for (dynsim_edo_B.b_i_a = 0; dynsim_edo_B.b_i_a <= dynsim_edo_B.idx;
       dynsim_edo_B.b_i_a++) {
    for (dynsim_edo_B.f = 0; dynsim_edo_B.f < 36; dynsim_edo_B.f++) {
      Ic->data[dynsim_edo_B.b_i_a].f1[dynsim_edo_B.f] = robot->
        Bodies[dynsim_edo_B.b_i_a]->SpatialInertia[dynsim_edo_B.f];
    }

    dynsim_edo_B.vNum = robot->PositionDoFMap[dynsim_edo_B.b_i_a];
    dynsim_edo_B.p_idx_1 = robot->PositionDoFMap[dynsim_edo_B.b_i_a + 6];
    if (dynsim_edo_B.p_idx_1 < dynsim_edo_B.vNum) {
      obj = robot->Bodies[dynsim_edo_B.b_i_a];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal, dynsim_edo_B.T_f);
    } else {
      if (dynsim_edo_B.vNum > dynsim_edo_B.p_idx_1) {
        dynsim_edo_B.nm1d2 = 0;
        dynsim_edo_B.f = -1;
      } else {
        dynsim_edo_B.nm1d2 = static_cast<int32_T>(dynsim_edo_B.vNum) - 1;
        dynsim_edo_B.f = static_cast<int32_T>(dynsim_edo_B.p_idx_1) - 1;
      }

      obj = robot->Bodies[dynsim_edo_B.b_i_a];
      dynsim_edo_B.loop_ub_n = dynsim_edo_B.f - dynsim_edo_B.nm1d2;
      dynsim_edo_B.q_size_l = dynsim_edo_B.loop_ub_n + 1;
      for (dynsim_edo_B.f = 0; dynsim_edo_B.f <= dynsim_edo_B.loop_ub_n;
           dynsim_edo_B.f++) {
        dynsim_edo_B.q_data_j[dynsim_edo_B.f] = q[dynsim_edo_B.nm1d2 +
          dynsim_edo_B.f];
      }

      rigidBodyJoint_transformBodyT_h(&obj->JointInternal, dynsim_edo_B.q_data_j,
        &dynsim_edo_B.q_size_l, dynsim_edo_B.T_f);
    }

    dynsim_edo_tforminv(dynsim_edo_B.T_f, dynsim_edo_B.dv);
    dynsim_edo_tformToSpatialXform(dynsim_edo_B.dv, X->data[dynsim_edo_B.b_i_a].
      f1);
  }

  dynsim_edo_B.nm1d2 = static_cast<int32_T>(((-1.0 - dynsim_edo_B.nb_i) + 1.0) /
    -1.0) - 1;
  dynsim_edo_emxInit_real_T(&Si, 2);
  dynsim_edo_emxInit_real_T(&Fi, 2);
  dynsim_edo_emxInit_real_T(&Hji, 2);
  dynsim_edo_emxInit_char_T(&a, 2);
  for (dynsim_edo_B.idx = 0; dynsim_edo_B.idx <= dynsim_edo_B.nm1d2;
       dynsim_edo_B.idx++) {
    dynsim_edo_B.n_pb = static_cast<int32_T>(dynsim_edo_B.nb_i +
      -static_cast<real_T>(dynsim_edo_B.idx));
    dynsim_edo_B.pid_tmp = dynsim_edo_B.n_pb - 1;
    dynsim_edo_B.pid = robot->Bodies[dynsim_edo_B.pid_tmp]->ParentIndex;
    dynsim_edo_B.vNum = robot->VelocityDoFMap[dynsim_edo_B.n_pb - 1];
    dynsim_edo_B.p_idx_1 = robot->VelocityDoFMap[dynsim_edo_B.n_pb + 5];
    if (dynsim_edo_B.pid > 0.0) {
      for (dynsim_edo_B.f = 0; dynsim_edo_B.f < 6; dynsim_edo_B.f++) {
        for (dynsim_edo_B.b_i_a = 0; dynsim_edo_B.b_i_a < 6; dynsim_edo_B.b_i_a
             ++) {
          dynsim_edo_B.X_tmp = dynsim_edo_B.f + 6 * dynsim_edo_B.b_i_a;
          dynsim_edo_B.X_m[dynsim_edo_B.X_tmp] = 0.0;
          for (dynsim_edo_B.loop_ub_n = 0; dynsim_edo_B.loop_ub_n < 6;
               dynsim_edo_B.loop_ub_n++) {
            dynsim_edo_B.X_m[dynsim_edo_B.X_tmp] += X->data[dynsim_edo_B.pid_tmp]
              .f1[6 * dynsim_edo_B.f + dynsim_edo_B.loop_ub_n] * Ic->
              data[dynsim_edo_B.pid_tmp].f1[6 * dynsim_edo_B.b_i_a +
              dynsim_edo_B.loop_ub_n];
          }
        }
      }

      for (dynsim_edo_B.f = 0; dynsim_edo_B.f < 6; dynsim_edo_B.f++) {
        for (dynsim_edo_B.b_i_a = 0; dynsim_edo_B.b_i_a < 6; dynsim_edo_B.b_i_a
             ++) {
          dynsim_edo_B.b_idx_0_g = 0.0;
          for (dynsim_edo_B.loop_ub_n = 0; dynsim_edo_B.loop_ub_n < 6;
               dynsim_edo_B.loop_ub_n++) {
            dynsim_edo_B.b_idx_0_g += dynsim_edo_B.X_m[6 *
              dynsim_edo_B.loop_ub_n + dynsim_edo_B.f] * X->
              data[dynsim_edo_B.pid_tmp].f1[6 * dynsim_edo_B.b_i_a +
              dynsim_edo_B.loop_ub_n];
          }

          dynsim_edo_B.loop_ub_n = 6 * dynsim_edo_B.b_i_a + dynsim_edo_B.f;
          Ic->data[static_cast<int32_T>(dynsim_edo_B.pid) - 1]
            .f1[dynsim_edo_B.loop_ub_n] += dynsim_edo_B.b_idx_0_g;
        }
      }

      lambda_->data[dynsim_edo_B.pid_tmp] = dynsim_edo_B.pid;
      exitg1 = false;
      while ((!exitg1) && (lambda_->data[dynsim_edo_B.pid_tmp] > 0.0)) {
        obj = robot->Bodies[static_cast<int32_T>(lambda_->
          data[dynsim_edo_B.pid_tmp]) - 1];
        dynsim_edo_B.f = a->size[0] * a->size[1];
        a->size[0] = 1;
        a->size[1] = obj->JointInternal.Type->size[1];
        dynsim_emxEnsureCapacity_char_T(a, dynsim_edo_B.f);
        dynsim_edo_B.loop_ub_n = obj->JointInternal.Type->size[0] *
          obj->JointInternal.Type->size[1] - 1;
        for (dynsim_edo_B.f = 0; dynsim_edo_B.f <= dynsim_edo_B.loop_ub_n;
             dynsim_edo_B.f++) {
          a->data[dynsim_edo_B.f] = obj->JointInternal.Type->data[dynsim_edo_B.f];
        }

        for (dynsim_edo_B.f = 0; dynsim_edo_B.f < 5; dynsim_edo_B.f++) {
          dynsim_edo_B.b_k[dynsim_edo_B.f] = tmp[dynsim_edo_B.f];
        }

        dynsim_edo_B.b_bool_f = false;
        if (a->size[1] == 5) {
          dynsim_edo_B.f = 1;
          do {
            exitg2 = 0;
            if (dynsim_edo_B.f - 1 < 5) {
              dynsim_edo_B.b_i_a = dynsim_edo_B.f - 1;
              if (a->data[dynsim_edo_B.b_i_a] !=
                  dynsim_edo_B.b_k[dynsim_edo_B.b_i_a]) {
                exitg2 = 1;
              } else {
                dynsim_edo_B.f++;
              }
            } else {
              dynsim_edo_B.b_bool_f = true;
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        }

        if (dynsim_edo_B.b_bool_f) {
          lambda_->data[dynsim_edo_B.pid_tmp] = robot->Bodies
            [static_cast<int32_T>(lambda_->data[dynsim_edo_B.pid_tmp]) - 1]
            ->ParentIndex;
        } else {
          exitg1 = true;
        }
      }
    }

    dynsim_edo_B.b_idx_0_g = robot->VelocityDoFMap[dynsim_edo_B.n_pb - 1];
    dynsim_edo_B.b_idx_1_c = robot->VelocityDoFMap[dynsim_edo_B.n_pb + 5];
    if (dynsim_edo_B.b_idx_0_g <= dynsim_edo_B.b_idx_1_c) {
      obj = robot->Bodies[dynsim_edo_B.pid_tmp];
      dynsim_edo_B.f = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
      dynsim_emxEnsureCapacity_real_T(Si, dynsim_edo_B.f);
      dynsim_edo_B.loop_ub_n = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (dynsim_edo_B.f = 0; dynsim_edo_B.f <= dynsim_edo_B.loop_ub_n;
           dynsim_edo_B.f++) {
        Si->data[dynsim_edo_B.f] = obj->JointInternal.MotionSubspace->
          data[dynsim_edo_B.f];
      }

      dynsim_edo_B.n_pb = Si->size[1] - 1;
      dynsim_edo_B.f = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = Si->size[1];
      dynsim_emxEnsureCapacity_real_T(Fi, dynsim_edo_B.f);
      for (dynsim_edo_B.loop_ub_n = 0; dynsim_edo_B.loop_ub_n <=
           dynsim_edo_B.n_pb; dynsim_edo_B.loop_ub_n++) {
        dynsim_edo_B.coffset_tmp_p = dynsim_edo_B.loop_ub_n * 6 - 1;
        for (dynsim_edo_B.b_i_a = 0; dynsim_edo_B.b_i_a < 6; dynsim_edo_B.b_i_a
             ++) {
          dynsim_edo_B.s_f = 0.0;
          for (dynsim_edo_B.f = 0; dynsim_edo_B.f < 6; dynsim_edo_B.f++) {
            dynsim_edo_B.s_f += Ic->data[dynsim_edo_B.pid_tmp].f1[dynsim_edo_B.f
              * 6 + dynsim_edo_B.b_i_a] * Si->data[(dynsim_edo_B.coffset_tmp_p +
              dynsim_edo_B.f) + 1];
          }

          Fi->data[(dynsim_edo_B.coffset_tmp_p + dynsim_edo_B.b_i_a) + 1] =
            dynsim_edo_B.s_f;
        }
      }

      if (dynsim_edo_B.vNum > dynsim_edo_B.p_idx_1) {
        dynsim_edo_B.coffset_tmp_p = 0;
        dynsim_edo_B.cb = 0;
      } else {
        dynsim_edo_B.coffset_tmp_p = static_cast<int32_T>(dynsim_edo_B.vNum) - 1;
        dynsim_edo_B.cb = static_cast<int32_T>(dynsim_edo_B.vNum) - 1;
      }

      dynsim_edo_B.m_m = Si->size[1];
      dynsim_edo_B.n_pb = Fi->size[1] - 1;
      dynsim_edo_B.f = Hji->size[0] * Hji->size[1];
      Hji->size[0] = Si->size[1];
      Hji->size[1] = Fi->size[1];
      dynsim_emxEnsureCapacity_real_T(Hji, dynsim_edo_B.f);
      for (dynsim_edo_B.loop_ub_n = 0; dynsim_edo_B.loop_ub_n <=
           dynsim_edo_B.n_pb; dynsim_edo_B.loop_ub_n++) {
        dynsim_edo_B.coffset = dynsim_edo_B.loop_ub_n * dynsim_edo_B.m_m - 1;
        dynsim_edo_B.boffset = dynsim_edo_B.loop_ub_n * 6 - 1;
        for (dynsim_edo_B.b_i_a = 0; dynsim_edo_B.b_i_a < dynsim_edo_B.m_m;
             dynsim_edo_B.b_i_a++) {
          dynsim_edo_B.aoffset_o = dynsim_edo_B.b_i_a * 6 - 1;
          dynsim_edo_B.s_f = 0.0;
          for (dynsim_edo_B.f = 0; dynsim_edo_B.f < 6; dynsim_edo_B.f++) {
            dynsim_edo_B.X_tmp = dynsim_edo_B.f + 1;
            dynsim_edo_B.s_f += Si->data[dynsim_edo_B.aoffset_o +
              dynsim_edo_B.X_tmp] * Fi->data[dynsim_edo_B.boffset +
              dynsim_edo_B.X_tmp];
          }

          Hji->data[(dynsim_edo_B.coffset + dynsim_edo_B.b_i_a) + 1] =
            dynsim_edo_B.s_f;
        }
      }

      dynsim_edo_B.loop_ub_n = Hji->size[1];
      for (dynsim_edo_B.f = 0; dynsim_edo_B.f < dynsim_edo_B.loop_ub_n;
           dynsim_edo_B.f++) {
        dynsim_edo_B.n_pb = Hji->size[0];
        for (dynsim_edo_B.b_i_a = 0; dynsim_edo_B.b_i_a < dynsim_edo_B.n_pb;
             dynsim_edo_B.b_i_a++) {
          H->data[(dynsim_edo_B.coffset_tmp_p + dynsim_edo_B.b_i_a) + H->size[0]
            * (dynsim_edo_B.cb + dynsim_edo_B.f)] = Hji->data[Hji->size[0] *
            dynsim_edo_B.f + dynsim_edo_B.b_i_a];
        }
      }

      dynsim_edo_B.n_pb = Fi->size[1];
      dynsim_edo_B.f = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = Fi->size[1];
      dynsim_emxEnsureCapacity_real_T(Si, dynsim_edo_B.f);
      dynsim_edo_B.loop_ub_n = Fi->size[0] * Fi->size[1] - 1;
      for (dynsim_edo_B.f = 0; dynsim_edo_B.f <= dynsim_edo_B.loop_ub_n;
           dynsim_edo_B.f++) {
        Si->data[dynsim_edo_B.f] = Fi->data[dynsim_edo_B.f];
      }

      dynsim_edo_B.f = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = dynsim_edo_B.n_pb;
      dynsim_emxEnsureCapacity_real_T(Fi, dynsim_edo_B.f);
      for (dynsim_edo_B.loop_ub_n = 0; dynsim_edo_B.loop_ub_n <
           dynsim_edo_B.n_pb; dynsim_edo_B.loop_ub_n++) {
        dynsim_edo_B.coffset_tmp_p = dynsim_edo_B.loop_ub_n * 6 - 1;
        for (dynsim_edo_B.b_i_a = 0; dynsim_edo_B.b_i_a < 6; dynsim_edo_B.b_i_a
             ++) {
          dynsim_edo_B.aoffset_o = dynsim_edo_B.b_i_a * 6 - 1;
          dynsim_edo_B.s_f = 0.0;
          for (dynsim_edo_B.f = 0; dynsim_edo_B.f < 6; dynsim_edo_B.f++) {
            dynsim_edo_B.X_tmp = dynsim_edo_B.f + 1;
            dynsim_edo_B.s_f += X->data[dynsim_edo_B.pid_tmp]
              .f1[dynsim_edo_B.aoffset_o + dynsim_edo_B.X_tmp] * Si->
              data[dynsim_edo_B.coffset_tmp_p + dynsim_edo_B.X_tmp];
          }

          Fi->data[(dynsim_edo_B.coffset_tmp_p + dynsim_edo_B.b_i_a) + 1] =
            dynsim_edo_B.s_f;
        }
      }

      while (dynsim_edo_B.pid > 0.0) {
        dynsim_edo_B.b_i_a = static_cast<int32_T>(dynsim_edo_B.pid);
        dynsim_edo_B.pid_tmp = dynsim_edo_B.b_i_a - 1;
        obj = robot->Bodies[dynsim_edo_B.pid_tmp];
        dynsim_edo_B.f = Si->size[0] * Si->size[1];
        Si->size[0] = 6;
        Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
        dynsim_emxEnsureCapacity_real_T(Si, dynsim_edo_B.f);
        dynsim_edo_B.loop_ub_n = obj->JointInternal.MotionSubspace->size[0] *
          obj->JointInternal.MotionSubspace->size[1] - 1;
        for (dynsim_edo_B.f = 0; dynsim_edo_B.f <= dynsim_edo_B.loop_ub_n;
             dynsim_edo_B.f++) {
          Si->data[dynsim_edo_B.f] = obj->JointInternal.MotionSubspace->
            data[dynsim_edo_B.f];
        }

        dynsim_edo_B.b_idx_0_g = robot->VelocityDoFMap[dynsim_edo_B.b_i_a - 1];
        dynsim_edo_B.b_idx_1_c = robot->VelocityDoFMap[dynsim_edo_B.b_i_a + 5];
        if (dynsim_edo_B.b_idx_0_g <= dynsim_edo_B.b_idx_1_c) {
          dynsim_edo_B.m_m = Si->size[1];
          dynsim_edo_B.n_pb = Fi->size[1] - 1;
          dynsim_edo_B.f = Hji->size[0] * Hji->size[1];
          Hji->size[0] = Si->size[1];
          Hji->size[1] = Fi->size[1];
          dynsim_emxEnsureCapacity_real_T(Hji, dynsim_edo_B.f);
          for (dynsim_edo_B.loop_ub_n = 0; dynsim_edo_B.loop_ub_n <=
               dynsim_edo_B.n_pb; dynsim_edo_B.loop_ub_n++) {
            dynsim_edo_B.coffset = dynsim_edo_B.loop_ub_n * dynsim_edo_B.m_m - 1;
            dynsim_edo_B.boffset = dynsim_edo_B.loop_ub_n * 6 - 1;
            for (dynsim_edo_B.b_i_a = 0; dynsim_edo_B.b_i_a < dynsim_edo_B.m_m;
                 dynsim_edo_B.b_i_a++) {
              dynsim_edo_B.aoffset_o = dynsim_edo_B.b_i_a * 6 - 1;
              dynsim_edo_B.s_f = 0.0;
              for (dynsim_edo_B.f = 0; dynsim_edo_B.f < 6; dynsim_edo_B.f++) {
                dynsim_edo_B.X_tmp = dynsim_edo_B.f + 1;
                dynsim_edo_B.s_f += Si->data[dynsim_edo_B.aoffset_o +
                  dynsim_edo_B.X_tmp] * Fi->data[dynsim_edo_B.boffset +
                  dynsim_edo_B.X_tmp];
              }

              Hji->data[(dynsim_edo_B.coffset + dynsim_edo_B.b_i_a) + 1] =
                dynsim_edo_B.s_f;
            }
          }

          if (dynsim_edo_B.b_idx_0_g > dynsim_edo_B.b_idx_1_c) {
            dynsim_edo_B.X_tmp = 0;
          } else {
            dynsim_edo_B.X_tmp = static_cast<int32_T>(dynsim_edo_B.b_idx_0_g) -
              1;
          }

          if (dynsim_edo_B.vNum > dynsim_edo_B.p_idx_1) {
            dynsim_edo_B.coffset_tmp_p = 0;
          } else {
            dynsim_edo_B.coffset_tmp_p = static_cast<int32_T>(dynsim_edo_B.vNum)
              - 1;
          }

          dynsim_edo_B.loop_ub_n = Hji->size[1];
          for (dynsim_edo_B.f = 0; dynsim_edo_B.f < dynsim_edo_B.loop_ub_n;
               dynsim_edo_B.f++) {
            dynsim_edo_B.n_pb = Hji->size[0];
            for (dynsim_edo_B.b_i_a = 0; dynsim_edo_B.b_i_a < dynsim_edo_B.n_pb;
                 dynsim_edo_B.b_i_a++) {
              H->data[(dynsim_edo_B.X_tmp + dynsim_edo_B.b_i_a) + H->size[0] *
                (dynsim_edo_B.coffset_tmp_p + dynsim_edo_B.f)] = Hji->data
                [Hji->size[0] * dynsim_edo_B.f + dynsim_edo_B.b_i_a];
            }
          }

          if (dynsim_edo_B.vNum > dynsim_edo_B.p_idx_1) {
            dynsim_edo_B.X_tmp = 0;
          } else {
            dynsim_edo_B.X_tmp = static_cast<int32_T>(dynsim_edo_B.vNum) - 1;
          }

          if (dynsim_edo_B.b_idx_0_g > dynsim_edo_B.b_idx_1_c) {
            dynsim_edo_B.coffset_tmp_p = 0;
          } else {
            dynsim_edo_B.coffset_tmp_p = static_cast<int32_T>
              (dynsim_edo_B.b_idx_0_g) - 1;
          }

          dynsim_edo_B.loop_ub_n = Hji->size[0];
          for (dynsim_edo_B.f = 0; dynsim_edo_B.f < dynsim_edo_B.loop_ub_n;
               dynsim_edo_B.f++) {
            dynsim_edo_B.n_pb = Hji->size[1];
            for (dynsim_edo_B.b_i_a = 0; dynsim_edo_B.b_i_a < dynsim_edo_B.n_pb;
                 dynsim_edo_B.b_i_a++) {
              H->data[(dynsim_edo_B.X_tmp + dynsim_edo_B.b_i_a) + H->size[0] *
                (dynsim_edo_B.coffset_tmp_p + dynsim_edo_B.f)] = Hji->data
                [Hji->size[0] * dynsim_edo_B.b_i_a + dynsim_edo_B.f];
            }
          }
        }

        dynsim_edo_B.n_pb = Fi->size[1];
        dynsim_edo_B.f = Si->size[0] * Si->size[1];
        Si->size[0] = 6;
        Si->size[1] = Fi->size[1];
        dynsim_emxEnsureCapacity_real_T(Si, dynsim_edo_B.f);
        dynsim_edo_B.loop_ub_n = Fi->size[0] * Fi->size[1] - 1;
        for (dynsim_edo_B.f = 0; dynsim_edo_B.f <= dynsim_edo_B.loop_ub_n;
             dynsim_edo_B.f++) {
          Si->data[dynsim_edo_B.f] = Fi->data[dynsim_edo_B.f];
        }

        dynsim_edo_B.f = Fi->size[0] * Fi->size[1];
        Fi->size[0] = 6;
        Fi->size[1] = dynsim_edo_B.n_pb;
        dynsim_emxEnsureCapacity_real_T(Fi, dynsim_edo_B.f);
        for (dynsim_edo_B.loop_ub_n = 0; dynsim_edo_B.loop_ub_n <
             dynsim_edo_B.n_pb; dynsim_edo_B.loop_ub_n++) {
          dynsim_edo_B.coffset_tmp_p = dynsim_edo_B.loop_ub_n * 6 - 1;
          for (dynsim_edo_B.b_i_a = 0; dynsim_edo_B.b_i_a < 6;
               dynsim_edo_B.b_i_a++) {
            dynsim_edo_B.aoffset_o = dynsim_edo_B.b_i_a * 6 - 1;
            dynsim_edo_B.s_f = 0.0;
            for (dynsim_edo_B.f = 0; dynsim_edo_B.f < 6; dynsim_edo_B.f++) {
              dynsim_edo_B.X_tmp = dynsim_edo_B.f + 1;
              dynsim_edo_B.s_f += X->data[dynsim_edo_B.pid_tmp]
                .f1[dynsim_edo_B.aoffset_o + dynsim_edo_B.X_tmp] * Si->
                data[dynsim_edo_B.coffset_tmp_p + dynsim_edo_B.X_tmp];
            }

            Fi->data[(dynsim_edo_B.coffset_tmp_p + dynsim_edo_B.b_i_a) + 1] =
              dynsim_edo_B.s_f;
          }
        }

        dynsim_edo_B.pid = robot->Bodies[dynsim_edo_B.pid_tmp]->ParentIndex;
      }
    }
  }

  dynsim_edo_emxFree_char_T(&a);
  dynsim_edo_emxFree_real_T(&Hji);
  dynsim_edo_emxFree_real_T(&Fi);
  dynsim_edo_emxFree_real_T(&Si);
  dynsim_edo_emxFree_f_cell_wrap1(&X);
  dynsim_edo_emxFree_f_cell_wrap1(&Ic);
  for (dynsim_edo_B.f = 0; dynsim_edo_B.f < 6; dynsim_edo_B.f++) {
    dynsim_edo_B.mask[dynsim_edo_B.f] = (robot->VelocityDoFMap[dynsim_edo_B.f] <=
      robot->VelocityDoFMap[dynsim_edo_B.f + 6]);
  }

  dynsim_edo_B.idx = 0;
  dynsim_edo_B.f = 1;
  exitg1 = false;
  while ((!exitg1) && (dynsim_edo_B.f - 1 < 6)) {
    if (dynsim_edo_B.mask[dynsim_edo_B.f - 1]) {
      dynsim_edo_B.idx++;
      dynsim_edo_B.ii_data[dynsim_edo_B.idx - 1] = dynsim_edo_B.f;
      if (dynsim_edo_B.idx >= 6) {
        exitg1 = true;
      } else {
        dynsim_edo_B.f++;
      }
    } else {
      dynsim_edo_B.f++;
    }
  }

  if (1 > dynsim_edo_B.idx) {
    dynsim_edo_B.idx = 0;
  }

  for (dynsim_edo_B.f = 0; dynsim_edo_B.f < dynsim_edo_B.idx; dynsim_edo_B.f++)
  {
    dynsim_edo_B.nonFixedIndices_data[dynsim_edo_B.f] =
      dynsim_edo_B.ii_data[dynsim_edo_B.f];
  }

  dynsim_edo_B.b_i_a = dynsim_edo_B.idx - 1;
  dynsim_edo_emxInit_real_T(&s, 2);
  for (dynsim_edo_B.idx = 0; dynsim_edo_B.idx <= dynsim_edo_B.b_i_a;
       dynsim_edo_B.idx++) {
    dynsim_edo_B.vNum = robot->
      VelocityDoFMap[dynsim_edo_B.nonFixedIndices_data[dynsim_edo_B.idx] - 1];
    dynsim_edo_B.p_idx_1 = robot->
      VelocityDoFMap[dynsim_edo_B.nonFixedIndices_data[dynsim_edo_B.idx] + 5];
    if (rtIsNaN(dynsim_edo_B.vNum) || rtIsNaN(dynsim_edo_B.p_idx_1)) {
      dynsim_edo_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      dynsim_emxEnsureCapacity_real_T(s, dynsim_edo_B.f);
      s->data[0] = (rtNaN);
    } else if (dynsim_edo_B.p_idx_1 < dynsim_edo_B.vNum) {
      s->size[0] = 1;
      s->size[1] = 0;
    } else if ((rtIsInf(dynsim_edo_B.vNum) || rtIsInf(dynsim_edo_B.p_idx_1)) &&
               (dynsim_edo_B.vNum == dynsim_edo_B.p_idx_1)) {
      dynsim_edo_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      dynsim_emxEnsureCapacity_real_T(s, dynsim_edo_B.f);
      s->data[0] = (rtNaN);
    } else if (floor(dynsim_edo_B.vNum) == dynsim_edo_B.vNum) {
      dynsim_edo_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      dynsim_edo_B.loop_ub_n = static_cast<int32_T>(floor(dynsim_edo_B.p_idx_1 -
        dynsim_edo_B.vNum));
      s->size[1] = dynsim_edo_B.loop_ub_n + 1;
      dynsim_emxEnsureCapacity_real_T(s, dynsim_edo_B.f);
      for (dynsim_edo_B.f = 0; dynsim_edo_B.f <= dynsim_edo_B.loop_ub_n;
           dynsim_edo_B.f++) {
        s->data[dynsim_edo_B.f] = dynsim_edo_B.vNum + static_cast<real_T>
          (dynsim_edo_B.f);
      }
    } else {
      dynsim_edo_B.nb_i = floor((dynsim_edo_B.p_idx_1 - dynsim_edo_B.vNum) + 0.5);
      dynsim_edo_B.pid = dynsim_edo_B.vNum + dynsim_edo_B.nb_i;
      dynsim_edo_B.b_idx_0_g = dynsim_edo_B.pid - dynsim_edo_B.p_idx_1;
      dynsim_edo_B.b_idx_1_c = fabs(dynsim_edo_B.vNum);
      dynsim_edo_B.s_f = fabs(dynsim_edo_B.p_idx_1);
      if ((dynsim_edo_B.b_idx_1_c > dynsim_edo_B.s_f) || rtIsNaN
          (dynsim_edo_B.s_f)) {
        dynsim_edo_B.s_f = dynsim_edo_B.b_idx_1_c;
      }

      if (fabs(dynsim_edo_B.b_idx_0_g) < 4.4408920985006262E-16 *
          dynsim_edo_B.s_f) {
        dynsim_edo_B.nb_i++;
        dynsim_edo_B.pid = dynsim_edo_B.p_idx_1;
      } else if (dynsim_edo_B.b_idx_0_g > 0.0) {
        dynsim_edo_B.pid = (dynsim_edo_B.nb_i - 1.0) + dynsim_edo_B.vNum;
      } else {
        dynsim_edo_B.nb_i++;
      }

      if (dynsim_edo_B.nb_i >= 0.0) {
        dynsim_edo_B.f = static_cast<int32_T>(dynsim_edo_B.nb_i);
      } else {
        dynsim_edo_B.f = 0;
      }

      dynsim_edo_B.n_pb = dynsim_edo_B.f - 1;
      dynsim_edo_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = dynsim_edo_B.n_pb + 1;
      dynsim_emxEnsureCapacity_real_T(s, dynsim_edo_B.f);
      if (dynsim_edo_B.n_pb + 1 > 0) {
        s->data[0] = dynsim_edo_B.vNum;
        if (dynsim_edo_B.n_pb + 1 > 1) {
          s->data[dynsim_edo_B.n_pb] = dynsim_edo_B.pid;
          dynsim_edo_B.nm1d2 = ((dynsim_edo_B.n_pb < 0) + dynsim_edo_B.n_pb) >>
            1;
          dynsim_edo_B.loop_ub_n = dynsim_edo_B.nm1d2 - 2;
          for (dynsim_edo_B.f = 0; dynsim_edo_B.f <= dynsim_edo_B.loop_ub_n;
               dynsim_edo_B.f++) {
            dynsim_edo_B.X_tmp = dynsim_edo_B.f + 1;
            s->data[dynsim_edo_B.X_tmp] = dynsim_edo_B.vNum + static_cast<real_T>
              (dynsim_edo_B.X_tmp);
            s->data[dynsim_edo_B.n_pb - dynsim_edo_B.X_tmp] = dynsim_edo_B.pid -
              static_cast<real_T>(dynsim_edo_B.X_tmp);
          }

          if (dynsim_edo_B.nm1d2 << 1 == dynsim_edo_B.n_pb) {
            s->data[dynsim_edo_B.nm1d2] = (dynsim_edo_B.vNum + dynsim_edo_B.pid)
              / 2.0;
          } else {
            s->data[dynsim_edo_B.nm1d2] = dynsim_edo_B.vNum + static_cast<real_T>
              (dynsim_edo_B.nm1d2);
            s->data[dynsim_edo_B.nm1d2 + 1] = dynsim_edo_B.pid -
              static_cast<real_T>(dynsim_edo_B.nm1d2);
          }
        }
      }
    }

    if (dynsim_edo_B.vNum > dynsim_edo_B.p_idx_1) {
      dynsim_edo_B.nm1d2 = 0;
    } else {
      dynsim_edo_B.nm1d2 = static_cast<int32_T>(dynsim_edo_B.vNum) - 1;
    }

    dynsim_edo_B.loop_ub_n = s->size[1];
    for (dynsim_edo_B.f = 0; dynsim_edo_B.f < dynsim_edo_B.loop_ub_n;
         dynsim_edo_B.f++) {
      lambda->data[dynsim_edo_B.nm1d2 + dynsim_edo_B.f] = s->data[dynsim_edo_B.f]
        - 1.0;
    }

    if (lambda_->data[dynsim_edo_B.nonFixedIndices_data[dynsim_edo_B.idx] - 1] ==
        0.0) {
      lambda->data[static_cast<int32_T>(dynsim_edo_B.vNum) - 1] = 0.0;
    } else {
      dynsim_edo_B.f = static_cast<int32_T>(lambda_->
        data[dynsim_edo_B.nonFixedIndices_data[dynsim_edo_B.idx] - 1]);
      dynsim_edo_B.b_idx_1_c = robot->VelocityDoFMap[dynsim_edo_B.f + 5];
      lambda->data[static_cast<int32_T>(dynsim_edo_B.vNum) - 1] =
        dynsim_edo_B.b_idx_1_c;
    }
  }

  dynsim_edo_emxFree_real_T(&s);
  dynsim_edo_emxFree_real_T(&lambda_);
}

static void RigidBodyTreeDynamics_inverseDy(n_robotics_manip_internal__ha_T
  *robot, const real_T q[3], const real_T qdot[3], const real_T fext[36], real_T
  tau[3])
{
  emxArray_f_cell_wrap_dynsim_h_T *X;
  emxArray_f_cell_wrap_dynsim_h_T *Xtree;
  emxArray_real_T_dynsim_edo_T *vJ;
  emxArray_real_T_dynsim_edo_T *vB;
  emxArray_real_T_dynsim_edo_T *aB;
  emxArray_real_T_dynsim_edo_T *f;
  emxArray_real_T_dynsim_edo_T *S;
  emxArray_real_T_dynsim_edo_T *taui;
  l_robotics_manip_internal__ha_T *obj;
  emxArray_char_T_dynsim_edo_T *a;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  dynsim_edo_B.a0[0] = 0.0;
  dynsim_edo_B.a0[1] = 0.0;
  dynsim_edo_B.a0[2] = 0.0;
  dynsim_edo_B.a0[3] = -robot->Gravity[0];
  dynsim_edo_B.a0[4] = -robot->Gravity[1];
  dynsim_edo_B.a0[5] = -robot->Gravity[2];
  dynsim_edo_emxInit_real_T(&vJ, 2);
  dynsim_edo_B.nb = robot->NumBodies;
  dynsim_edo_B.b_k_h = vJ->size[0] * vJ->size[1];
  vJ->size[0] = 6;
  dynsim_edo_B.unnamed_idx_1 = static_cast<int32_T>(dynsim_edo_B.nb);
  vJ->size[1] = dynsim_edo_B.unnamed_idx_1;
  dynsim_emxEnsureCapacity_real_T(vJ, dynsim_edo_B.b_k_h);
  dynsim_edo_B.loop_ub_tmp = 6 * dynsim_edo_B.unnamed_idx_1 - 1;
  for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h <= dynsim_edo_B.loop_ub_tmp;
       dynsim_edo_B.b_k_h++) {
    vJ->data[dynsim_edo_B.b_k_h] = 0.0;
  }

  dynsim_edo_emxInit_real_T(&vB, 2);
  dynsim_edo_B.b_k_h = vB->size[0] * vB->size[1];
  vB->size[0] = 6;
  vB->size[1] = dynsim_edo_B.unnamed_idx_1;
  dynsim_emxEnsureCapacity_real_T(vB, dynsim_edo_B.b_k_h);
  for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h <= dynsim_edo_B.loop_ub_tmp;
       dynsim_edo_B.b_k_h++) {
    vB->data[dynsim_edo_B.b_k_h] = 0.0;
  }

  dynsim_edo_emxInit_real_T(&aB, 2);
  dynsim_edo_B.b_k_h = aB->size[0] * aB->size[1];
  aB->size[0] = 6;
  aB->size[1] = dynsim_edo_B.unnamed_idx_1;
  dynsim_emxEnsureCapacity_real_T(aB, dynsim_edo_B.b_k_h);
  for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h <= dynsim_edo_B.loop_ub_tmp;
       dynsim_edo_B.b_k_h++) {
    aB->data[dynsim_edo_B.b_k_h] = 0.0;
  }

  tau[0] = 0.0;
  tau[1] = 0.0;
  tau[2] = 0.0;
  dynsim_edo_emxInit_f_cell_wrap1(&X, 2);
  dynsim_edo_emxInit_f_cell_wrap1(&Xtree, 2);
  dynsim_edo_B.loop_ub_tmp = dynsim_edo_B.unnamed_idx_1 - 1;
  dynsim_edo_B.b_k_h = Xtree->size[0] * Xtree->size[1];
  Xtree->size[0] = 1;
  Xtree->size[1] = dynsim_edo_B.unnamed_idx_1;
  emxEnsureCapacity_f_cell_wrap1(Xtree, dynsim_edo_B.b_k_h);
  dynsim_edo_B.b_k_h = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = dynsim_edo_B.unnamed_idx_1;
  emxEnsureCapacity_f_cell_wrap1(X, dynsim_edo_B.b_k_h);
  for (dynsim_edo_B.b_k_f = 0; dynsim_edo_B.b_k_f <= dynsim_edo_B.loop_ub_tmp;
       dynsim_edo_B.b_k_f++) {
    for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 36; dynsim_edo_B.b_k_h++)
    {
      Xtree->data[dynsim_edo_B.b_k_f].f1[dynsim_edo_B.b_k_h] = 0.0;
    }

    for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 6; dynsim_edo_B.b_k_h++) {
      Xtree->data[dynsim_edo_B.b_k_f].f1[dynsim_edo_B.b_k_h + 6 *
        dynsim_edo_B.b_k_h] = 1.0;
    }

    for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 36; dynsim_edo_B.b_k_h++)
    {
      X->data[dynsim_edo_B.b_k_f].f1[dynsim_edo_B.b_k_h] = 0.0;
    }

    for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 6; dynsim_edo_B.b_k_h++) {
      X->data[dynsim_edo_B.b_k_f].f1[dynsim_edo_B.b_k_h + 6 * dynsim_edo_B.b_k_h]
        = 1.0;
    }
  }

  dynsim_edo_emxInit_real_T(&f, 2);
  dynsim_edo_B.b_k_h = f->size[0] * f->size[1];
  f->size[0] = 6;
  f->size[1] = dynsim_edo_B.unnamed_idx_1;
  dynsim_emxEnsureCapacity_real_T(f, dynsim_edo_B.b_k_h);
  dynsim_edo_emxInit_real_T(&S, 2);
  if (0 <= dynsim_edo_B.loop_ub_tmp) {
    dynsim_edo_B.dv1[0] = 0.0;
    dynsim_edo_B.dv1[4] = 0.0;
    dynsim_edo_B.dv1[8] = 0.0;
  }

  for (dynsim_edo_B.unnamed_idx_1 = 0; dynsim_edo_B.unnamed_idx_1 <=
       dynsim_edo_B.loop_ub_tmp; dynsim_edo_B.unnamed_idx_1++) {
    obj = robot->Bodies[dynsim_edo_B.unnamed_idx_1];
    dynsim_edo_B.b_k_h = S->size[0] * S->size[1];
    S->size[0] = 6;
    S->size[1] = obj->JointInternal.MotionSubspace->size[1];
    dynsim_emxEnsureCapacity_real_T(S, dynsim_edo_B.b_k_h);
    dynsim_edo_B.b_k_f = obj->JointInternal.MotionSubspace->size[0] *
      obj->JointInternal.MotionSubspace->size[1] - 1;
    for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h <= dynsim_edo_B.b_k_f;
         dynsim_edo_B.b_k_h++) {
      S->data[dynsim_edo_B.b_k_h] = obj->JointInternal.MotionSubspace->
        data[dynsim_edo_B.b_k_h];
    }

    dynsim_edo_B.a_idx_0 = robot->PositionDoFMap[dynsim_edo_B.unnamed_idx_1];
    dynsim_edo_B.a_idx_1 = robot->PositionDoFMap[dynsim_edo_B.unnamed_idx_1 + 6];
    dynsim_edo_B.b_idx_0 = robot->VelocityDoFMap[dynsim_edo_B.unnamed_idx_1];
    dynsim_edo_B.b_idx_1 = robot->VelocityDoFMap[dynsim_edo_B.unnamed_idx_1 + 6];
    if (dynsim_edo_B.a_idx_1 < dynsim_edo_B.a_idx_0) {
      obj = robot->Bodies[dynsim_edo_B.unnamed_idx_1];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal, dynsim_edo_B.T_c);
      for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 6; dynsim_edo_B.b_k_h++)
      {
        vJ->data[dynsim_edo_B.b_k_h + 6 * dynsim_edo_B.unnamed_idx_1] = 0.0;
      }
    } else {
      if (dynsim_edo_B.a_idx_0 > dynsim_edo_B.a_idx_1) {
        dynsim_edo_B.b_k_f = 0;
        dynsim_edo_B.b_k_h = -1;
      } else {
        dynsim_edo_B.b_k_f = static_cast<int32_T>(dynsim_edo_B.a_idx_0) - 1;
        dynsim_edo_B.b_k_h = static_cast<int32_T>(dynsim_edo_B.a_idx_1) - 1;
      }

      if (dynsim_edo_B.b_idx_0 > dynsim_edo_B.b_idx_1) {
        dynsim_edo_B.p = -1;
      } else {
        dynsim_edo_B.p = static_cast<int32_T>(dynsim_edo_B.b_idx_0) - 2;
      }

      obj = robot->Bodies[dynsim_edo_B.unnamed_idx_1];
      dynsim_edo_B.q_size_tmp = dynsim_edo_B.b_k_h - dynsim_edo_B.b_k_f;
      dynsim_edo_B.q_size = dynsim_edo_B.q_size_tmp + 1;
      for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h <= dynsim_edo_B.q_size_tmp;
           dynsim_edo_B.b_k_h++) {
        dynsim_edo_B.q_data[dynsim_edo_B.b_k_h] = q[dynsim_edo_B.b_k_f +
          dynsim_edo_B.b_k_h];
      }

      rigidBodyJoint_transformBodyT_h(&obj->JointInternal, dynsim_edo_B.q_data,
        &dynsim_edo_B.q_size, dynsim_edo_B.T_c);
      dynsim_edo_B.inner = S->size[1] - 1;
      for (dynsim_edo_B.b_k_f = 0; dynsim_edo_B.b_k_f < 6; dynsim_edo_B.b_k_f++)
      {
        vJ->data[dynsim_edo_B.b_k_f + 6 * dynsim_edo_B.unnamed_idx_1] = 0.0;
      }

      for (dynsim_edo_B.b_k_f = 0; dynsim_edo_B.b_k_f <= dynsim_edo_B.inner;
           dynsim_edo_B.b_k_f++) {
        dynsim_edo_B.aoffset = dynsim_edo_B.b_k_f * 6 - 1;
        for (dynsim_edo_B.q_size_tmp = 0; dynsim_edo_B.q_size_tmp < 6;
             dynsim_edo_B.q_size_tmp++) {
          dynsim_edo_B.b_k_h = 6 * dynsim_edo_B.unnamed_idx_1 +
            dynsim_edo_B.q_size_tmp;
          vJ->data[dynsim_edo_B.b_k_h] += S->data[(dynsim_edo_B.aoffset +
            dynsim_edo_B.q_size_tmp) + 1] * qdot[(dynsim_edo_B.p +
            dynsim_edo_B.b_k_f) + 1];
        }
      }
    }

    for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 3; dynsim_edo_B.b_k_h++) {
      dynsim_edo_B.R_dy[3 * dynsim_edo_B.b_k_h] =
        dynsim_edo_B.T_c[dynsim_edo_B.b_k_h];
      dynsim_edo_B.R_dy[3 * dynsim_edo_B.b_k_h + 1] =
        dynsim_edo_B.T_c[dynsim_edo_B.b_k_h + 4];
      dynsim_edo_B.R_dy[3 * dynsim_edo_B.b_k_h + 2] =
        dynsim_edo_B.T_c[dynsim_edo_B.b_k_h + 8];
    }

    for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 9; dynsim_edo_B.b_k_h++) {
      dynsim_edo_B.R_lx[dynsim_edo_B.b_k_h] =
        -dynsim_edo_B.R_dy[dynsim_edo_B.b_k_h];
    }

    for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 3; dynsim_edo_B.b_k_h++) {
      dynsim_edo_B.b_k_f = dynsim_edo_B.b_k_h << 2;
      dynsim_edo_B.Tinv[dynsim_edo_B.b_k_f] = dynsim_edo_B.R_dy[3 *
        dynsim_edo_B.b_k_h];
      dynsim_edo_B.Tinv[dynsim_edo_B.b_k_f + 1] = dynsim_edo_B.R_dy[3 *
        dynsim_edo_B.b_k_h + 1];
      dynsim_edo_B.Tinv[dynsim_edo_B.b_k_f + 2] = dynsim_edo_B.R_dy[3 *
        dynsim_edo_B.b_k_h + 2];
      dynsim_edo_B.Tinv[dynsim_edo_B.b_k_h + 12] =
        dynsim_edo_B.R_lx[dynsim_edo_B.b_k_h + 6] * dynsim_edo_B.T_c[14] +
        (dynsim_edo_B.R_lx[dynsim_edo_B.b_k_h + 3] * dynsim_edo_B.T_c[13] +
         dynsim_edo_B.R_lx[dynsim_edo_B.b_k_h] * dynsim_edo_B.T_c[12]);
    }

    dynsim_edo_B.Tinv[3] = 0.0;
    dynsim_edo_B.Tinv[7] = 0.0;
    dynsim_edo_B.Tinv[11] = 0.0;
    dynsim_edo_B.Tinv[15] = 1.0;
    dynsim_edo_B.dv1[3] = -dynsim_edo_B.Tinv[14];
    dynsim_edo_B.dv1[6] = dynsim_edo_B.Tinv[13];
    dynsim_edo_B.dv1[1] = dynsim_edo_B.Tinv[14];
    dynsim_edo_B.dv1[7] = -dynsim_edo_B.Tinv[12];
    dynsim_edo_B.dv1[2] = -dynsim_edo_B.Tinv[13];
    dynsim_edo_B.dv1[5] = dynsim_edo_B.Tinv[12];
    for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 3; dynsim_edo_B.b_k_h++) {
      for (dynsim_edo_B.b_k_f = 0; dynsim_edo_B.b_k_f < 3; dynsim_edo_B.b_k_f++)
      {
        dynsim_edo_B.q_size_tmp = dynsim_edo_B.b_k_h + 3 * dynsim_edo_B.b_k_f;
        dynsim_edo_B.R_dy[dynsim_edo_B.q_size_tmp] = 0.0;
        dynsim_edo_B.p = dynsim_edo_B.b_k_f << 2;
        dynsim_edo_B.R_dy[dynsim_edo_B.q_size_tmp] +=
          dynsim_edo_B.Tinv[dynsim_edo_B.p] *
          dynsim_edo_B.dv1[dynsim_edo_B.b_k_h];
        dynsim_edo_B.R_dy[dynsim_edo_B.q_size_tmp] +=
          dynsim_edo_B.Tinv[dynsim_edo_B.p + 1] *
          dynsim_edo_B.dv1[dynsim_edo_B.b_k_h + 3];
        dynsim_edo_B.R_dy[dynsim_edo_B.q_size_tmp] +=
          dynsim_edo_B.Tinv[dynsim_edo_B.p + 2] *
          dynsim_edo_B.dv1[dynsim_edo_B.b_k_h + 6];
        X->data[dynsim_edo_B.unnamed_idx_1].f1[dynsim_edo_B.b_k_f + 6 *
          dynsim_edo_B.b_k_h] = dynsim_edo_B.Tinv[(dynsim_edo_B.b_k_h << 2) +
          dynsim_edo_B.b_k_f];
        X->data[dynsim_edo_B.unnamed_idx_1].f1[dynsim_edo_B.b_k_f + 6 *
          (dynsim_edo_B.b_k_h + 3)] = 0.0;
      }
    }

    for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 3; dynsim_edo_B.b_k_h++) {
      X->data[dynsim_edo_B.unnamed_idx_1].f1[6 * dynsim_edo_B.b_k_h + 3] =
        dynsim_edo_B.R_dy[3 * dynsim_edo_B.b_k_h];
      dynsim_edo_B.b_k_f = dynsim_edo_B.b_k_h << 2;
      dynsim_edo_B.q_size_tmp = 6 * (dynsim_edo_B.b_k_h + 3);
      X->data[dynsim_edo_B.unnamed_idx_1].f1[dynsim_edo_B.q_size_tmp + 3] =
        dynsim_edo_B.Tinv[dynsim_edo_B.b_k_f];
      X->data[dynsim_edo_B.unnamed_idx_1].f1[6 * dynsim_edo_B.b_k_h + 4] =
        dynsim_edo_B.R_dy[3 * dynsim_edo_B.b_k_h + 1];
      X->data[dynsim_edo_B.unnamed_idx_1].f1[dynsim_edo_B.q_size_tmp + 4] =
        dynsim_edo_B.Tinv[dynsim_edo_B.b_k_f + 1];
      X->data[dynsim_edo_B.unnamed_idx_1].f1[6 * dynsim_edo_B.b_k_h + 5] =
        dynsim_edo_B.R_dy[3 * dynsim_edo_B.b_k_h + 2];
      X->data[dynsim_edo_B.unnamed_idx_1].f1[dynsim_edo_B.q_size_tmp + 5] =
        dynsim_edo_B.Tinv[dynsim_edo_B.b_k_f + 2];
    }

    dynsim_edo_B.a_idx_0 = robot->Bodies[dynsim_edo_B.unnamed_idx_1]
      ->ParentIndex;
    if (dynsim_edo_B.a_idx_0 > 0.0) {
      dynsim_edo_B.m = static_cast<int32_T>(dynsim_edo_B.a_idx_0);
      for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 6; dynsim_edo_B.b_k_h++)
      {
        dynsim_edo_B.a_idx_1 = 0.0;
        for (dynsim_edo_B.b_k_f = 0; dynsim_edo_B.b_k_f < 6; dynsim_edo_B.b_k_f
             ++) {
          dynsim_edo_B.a_idx_1 += vB->data[(dynsim_edo_B.m - 1) * 6 +
            dynsim_edo_B.b_k_f] * X->data[dynsim_edo_B.unnamed_idx_1].f1[6 *
            dynsim_edo_B.b_k_f + dynsim_edo_B.b_k_h];
        }

        dynsim_edo_B.y[dynsim_edo_B.b_k_h] = vJ->data[6 *
          dynsim_edo_B.unnamed_idx_1 + dynsim_edo_B.b_k_h] +
          dynsim_edo_B.a_idx_1;
      }

      for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 6; dynsim_edo_B.b_k_h++)
      {
        vB->data[dynsim_edo_B.b_k_h + 6 * dynsim_edo_B.unnamed_idx_1] =
          dynsim_edo_B.y[dynsim_edo_B.b_k_h];
      }

      dynsim_edo_B.inner = S->size[1] - 1;
      for (dynsim_edo_B.b_k_f = 0; dynsim_edo_B.b_k_f < 6; dynsim_edo_B.b_k_f++)
      {
        dynsim_edo_B.y[dynsim_edo_B.b_k_f] = 0.0;
      }

      for (dynsim_edo_B.b_k_f = 0; dynsim_edo_B.b_k_f <= dynsim_edo_B.inner;
           dynsim_edo_B.b_k_f++) {
        dynsim_edo_B.aoffset = dynsim_edo_B.b_k_f * 6 - 1;
        for (dynsim_edo_B.q_size_tmp = 0; dynsim_edo_B.q_size_tmp < 6;
             dynsim_edo_B.q_size_tmp++) {
          dynsim_edo_B.a_idx_1 = S->data[(dynsim_edo_B.aoffset +
            dynsim_edo_B.q_size_tmp) + 1] * 0.0 +
            dynsim_edo_B.y[dynsim_edo_B.q_size_tmp];
          dynsim_edo_B.y[dynsim_edo_B.q_size_tmp] = dynsim_edo_B.a_idx_1;
        }
      }

      dynsim_edo_B.R_dy[0] = 0.0;
      dynsim_edo_B.b_k_f = 6 * dynsim_edo_B.unnamed_idx_1 + 2;
      dynsim_edo_B.R_dy[3] = -vB->data[dynsim_edo_B.b_k_f];
      dynsim_edo_B.b_k_h = 6 * dynsim_edo_B.unnamed_idx_1 + 1;
      dynsim_edo_B.R_dy[6] = vB->data[dynsim_edo_B.b_k_h];
      dynsim_edo_B.R_dy[1] = vB->data[dynsim_edo_B.b_k_f];
      dynsim_edo_B.R_dy[4] = 0.0;
      dynsim_edo_B.R_dy[7] = -vB->data[6 * dynsim_edo_B.unnamed_idx_1];
      dynsim_edo_B.R_dy[2] = -vB->data[dynsim_edo_B.b_k_h];
      dynsim_edo_B.R_dy[5] = vB->data[6 * dynsim_edo_B.unnamed_idx_1];
      dynsim_edo_B.R_dy[8] = 0.0;
      dynsim_edo_B.R[3] = 0.0;
      dynsim_edo_B.b_k_f = 6 * dynsim_edo_B.unnamed_idx_1 + 5;
      dynsim_edo_B.R[9] = -vB->data[dynsim_edo_B.b_k_f];
      dynsim_edo_B.b_k_h = 6 * dynsim_edo_B.unnamed_idx_1 + 4;
      dynsim_edo_B.R[15] = vB->data[dynsim_edo_B.b_k_h];
      dynsim_edo_B.R[4] = vB->data[dynsim_edo_B.b_k_f];
      dynsim_edo_B.R[10] = 0.0;
      dynsim_edo_B.b_k_f = 6 * dynsim_edo_B.unnamed_idx_1 + 3;
      dynsim_edo_B.R[16] = -vB->data[dynsim_edo_B.b_k_f];
      dynsim_edo_B.R[5] = -vB->data[dynsim_edo_B.b_k_h];
      dynsim_edo_B.R[11] = vB->data[dynsim_edo_B.b_k_f];
      dynsim_edo_B.R[17] = 0.0;
      for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 3; dynsim_edo_B.b_k_h++)
      {
        dynsim_edo_B.a_idx_1 = dynsim_edo_B.R_dy[3 * dynsim_edo_B.b_k_h];
        dynsim_edo_B.R[6 * dynsim_edo_B.b_k_h] = dynsim_edo_B.a_idx_1;
        dynsim_edo_B.b_k_f = 6 * (dynsim_edo_B.b_k_h + 3);
        dynsim_edo_B.R[dynsim_edo_B.b_k_f] = 0.0;
        dynsim_edo_B.R[dynsim_edo_B.b_k_f + 3] = dynsim_edo_B.a_idx_1;
        dynsim_edo_B.a_idx_1 = dynsim_edo_B.R_dy[3 * dynsim_edo_B.b_k_h + 1];
        dynsim_edo_B.R[6 * dynsim_edo_B.b_k_h + 1] = dynsim_edo_B.a_idx_1;
        dynsim_edo_B.R[dynsim_edo_B.b_k_f + 1] = 0.0;
        dynsim_edo_B.R[dynsim_edo_B.b_k_f + 4] = dynsim_edo_B.a_idx_1;
        dynsim_edo_B.a_idx_1 = dynsim_edo_B.R_dy[3 * dynsim_edo_B.b_k_h + 2];
        dynsim_edo_B.R[6 * dynsim_edo_B.b_k_h + 2] = dynsim_edo_B.a_idx_1;
        dynsim_edo_B.R[dynsim_edo_B.b_k_f + 2] = 0.0;
        dynsim_edo_B.R[dynsim_edo_B.b_k_f + 5] = dynsim_edo_B.a_idx_1;
      }

      for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 6; dynsim_edo_B.b_k_h++)
      {
        dynsim_edo_B.a_idx_1 = 0.0;
        for (dynsim_edo_B.b_k_f = 0; dynsim_edo_B.b_k_f < 6; dynsim_edo_B.b_k_f
             ++) {
          dynsim_edo_B.a_idx_1 += aB->data[(dynsim_edo_B.m - 1) * 6 +
            dynsim_edo_B.b_k_f] * X->data[dynsim_edo_B.unnamed_idx_1].f1[6 *
            dynsim_edo_B.b_k_f + dynsim_edo_B.b_k_h];
        }

        dynsim_edo_B.X_e[dynsim_edo_B.b_k_h] = dynsim_edo_B.a_idx_1 +
          dynsim_edo_B.y[dynsim_edo_B.b_k_h];
      }

      for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 6; dynsim_edo_B.b_k_h++)
      {
        dynsim_edo_B.y[dynsim_edo_B.b_k_h] = 0.0;
        for (dynsim_edo_B.b_k_f = 0; dynsim_edo_B.b_k_f < 6; dynsim_edo_B.b_k_f
             ++) {
          dynsim_edo_B.a_idx_1 = dynsim_edo_B.R[6 * dynsim_edo_B.b_k_f +
            dynsim_edo_B.b_k_h] * vJ->data[6 * dynsim_edo_B.unnamed_idx_1 +
            dynsim_edo_B.b_k_f] + dynsim_edo_B.y[dynsim_edo_B.b_k_h];
          dynsim_edo_B.y[dynsim_edo_B.b_k_h] = dynsim_edo_B.a_idx_1;
        }

        aB->data[dynsim_edo_B.b_k_h + 6 * dynsim_edo_B.unnamed_idx_1] =
          dynsim_edo_B.X_e[dynsim_edo_B.b_k_h] +
          dynsim_edo_B.y[dynsim_edo_B.b_k_h];
      }

      dynsim_edo_B.R_dy[0] = 0.0;
      dynsim_edo_B.R_dy[3] = -dynsim_edo_B.T_c[14];
      dynsim_edo_B.R_dy[6] = dynsim_edo_B.T_c[13];
      dynsim_edo_B.R_dy[1] = dynsim_edo_B.T_c[14];
      dynsim_edo_B.R_dy[4] = 0.0;
      dynsim_edo_B.R_dy[7] = -dynsim_edo_B.T_c[12];
      dynsim_edo_B.R_dy[2] = -dynsim_edo_B.T_c[13];
      dynsim_edo_B.R_dy[5] = dynsim_edo_B.T_c[12];
      dynsim_edo_B.R_dy[8] = 0.0;
      for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 3; dynsim_edo_B.b_k_h++)
      {
        for (dynsim_edo_B.b_k_f = 0; dynsim_edo_B.b_k_f < 3; dynsim_edo_B.b_k_f
             ++) {
          dynsim_edo_B.q_size_tmp = dynsim_edo_B.b_k_h + 3 * dynsim_edo_B.b_k_f;
          dynsim_edo_B.R_lx[dynsim_edo_B.q_size_tmp] = 0.0;
          dynsim_edo_B.p = dynsim_edo_B.b_k_f << 2;
          dynsim_edo_B.R_lx[dynsim_edo_B.q_size_tmp] +=
            dynsim_edo_B.T_c[dynsim_edo_B.p] *
            dynsim_edo_B.R_dy[dynsim_edo_B.b_k_h];
          dynsim_edo_B.R_lx[dynsim_edo_B.q_size_tmp] +=
            dynsim_edo_B.T_c[dynsim_edo_B.p + 1] *
            dynsim_edo_B.R_dy[dynsim_edo_B.b_k_h + 3];
          dynsim_edo_B.R_lx[dynsim_edo_B.q_size_tmp] +=
            dynsim_edo_B.T_c[dynsim_edo_B.p + 2] *
            dynsim_edo_B.R_dy[dynsim_edo_B.b_k_h + 6];
          dynsim_edo_B.R[dynsim_edo_B.b_k_f + 6 * dynsim_edo_B.b_k_h] =
            dynsim_edo_B.T_c[(dynsim_edo_B.b_k_h << 2) + dynsim_edo_B.b_k_f];
          dynsim_edo_B.R[dynsim_edo_B.b_k_f + 6 * (dynsim_edo_B.b_k_h + 3)] =
            0.0;
        }
      }

      for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 3; dynsim_edo_B.b_k_h++)
      {
        dynsim_edo_B.R[6 * dynsim_edo_B.b_k_h + 3] = dynsim_edo_B.R_lx[3 *
          dynsim_edo_B.b_k_h];
        dynsim_edo_B.b_k_f = dynsim_edo_B.b_k_h << 2;
        dynsim_edo_B.q_size_tmp = 6 * (dynsim_edo_B.b_k_h + 3);
        dynsim_edo_B.R[dynsim_edo_B.q_size_tmp + 3] =
          dynsim_edo_B.T_c[dynsim_edo_B.b_k_f];
        dynsim_edo_B.R[6 * dynsim_edo_B.b_k_h + 4] = dynsim_edo_B.R_lx[3 *
          dynsim_edo_B.b_k_h + 1];
        dynsim_edo_B.R[dynsim_edo_B.q_size_tmp + 4] =
          dynsim_edo_B.T_c[dynsim_edo_B.b_k_f + 1];
        dynsim_edo_B.R[6 * dynsim_edo_B.b_k_h + 5] = dynsim_edo_B.R_lx[3 *
          dynsim_edo_B.b_k_h + 2];
        dynsim_edo_B.R[dynsim_edo_B.q_size_tmp + 5] =
          dynsim_edo_B.T_c[dynsim_edo_B.b_k_f + 2];
      }

      for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 6; dynsim_edo_B.b_k_h++)
      {
        for (dynsim_edo_B.b_k_f = 0; dynsim_edo_B.b_k_f < 6; dynsim_edo_B.b_k_f
             ++) {
          dynsim_edo_B.p = dynsim_edo_B.b_k_h + 6 * dynsim_edo_B.b_k_f;
          dynsim_edo_B.b_I[dynsim_edo_B.p] = 0.0;
          for (dynsim_edo_B.q_size_tmp = 0; dynsim_edo_B.q_size_tmp < 6;
               dynsim_edo_B.q_size_tmp++) {
            dynsim_edo_B.b_I[dynsim_edo_B.p] += Xtree->data[static_cast<int32_T>
              (dynsim_edo_B.a_idx_0) - 1].f1[6 * dynsim_edo_B.q_size_tmp +
              dynsim_edo_B.b_k_h] * dynsim_edo_B.R[6 * dynsim_edo_B.b_k_f +
              dynsim_edo_B.q_size_tmp];
          }
        }
      }

      for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 36; dynsim_edo_B.b_k_h++)
      {
        Xtree->data[dynsim_edo_B.unnamed_idx_1].f1[dynsim_edo_B.b_k_h] =
          dynsim_edo_B.b_I[dynsim_edo_B.b_k_h];
      }
    } else {
      dynsim_edo_B.inner = S->size[1] - 1;
      for (dynsim_edo_B.b_k_f = 0; dynsim_edo_B.b_k_f < 6; dynsim_edo_B.b_k_f++)
      {
        dynsim_edo_B.b_k_h = 6 * dynsim_edo_B.unnamed_idx_1 + dynsim_edo_B.b_k_f;
        vB->data[dynsim_edo_B.b_k_h] = vJ->data[dynsim_edo_B.b_k_h];
        dynsim_edo_B.y[dynsim_edo_B.b_k_f] = 0.0;
      }

      for (dynsim_edo_B.b_k_f = 0; dynsim_edo_B.b_k_f <= dynsim_edo_B.inner;
           dynsim_edo_B.b_k_f++) {
        dynsim_edo_B.aoffset = dynsim_edo_B.b_k_f * 6 - 1;
        for (dynsim_edo_B.q_size_tmp = 0; dynsim_edo_B.q_size_tmp < 6;
             dynsim_edo_B.q_size_tmp++) {
          dynsim_edo_B.a_idx_1 = S->data[(dynsim_edo_B.aoffset +
            dynsim_edo_B.q_size_tmp) + 1] * 0.0 +
            dynsim_edo_B.y[dynsim_edo_B.q_size_tmp];
          dynsim_edo_B.y[dynsim_edo_B.q_size_tmp] = dynsim_edo_B.a_idx_1;
        }
      }

      for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 6; dynsim_edo_B.b_k_h++)
      {
        dynsim_edo_B.a_idx_1 = 0.0;
        for (dynsim_edo_B.b_k_f = 0; dynsim_edo_B.b_k_f < 6; dynsim_edo_B.b_k_f
             ++) {
          dynsim_edo_B.a_idx_1 += X->data[dynsim_edo_B.unnamed_idx_1].f1[6 *
            dynsim_edo_B.b_k_f + dynsim_edo_B.b_k_h] *
            dynsim_edo_B.a0[dynsim_edo_B.b_k_f];
        }

        aB->data[dynsim_edo_B.b_k_h + 6 * dynsim_edo_B.unnamed_idx_1] =
          dynsim_edo_B.a_idx_1 + dynsim_edo_B.y[dynsim_edo_B.b_k_h];
      }

      dynsim_edo_B.R_dy[0] = 0.0;
      dynsim_edo_B.R_dy[3] = -dynsim_edo_B.T_c[14];
      dynsim_edo_B.R_dy[6] = dynsim_edo_B.T_c[13];
      dynsim_edo_B.R_dy[1] = dynsim_edo_B.T_c[14];
      dynsim_edo_B.R_dy[4] = 0.0;
      dynsim_edo_B.R_dy[7] = -dynsim_edo_B.T_c[12];
      dynsim_edo_B.R_dy[2] = -dynsim_edo_B.T_c[13];
      dynsim_edo_B.R_dy[5] = dynsim_edo_B.T_c[12];
      dynsim_edo_B.R_dy[8] = 0.0;
      for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 3; dynsim_edo_B.b_k_h++)
      {
        for (dynsim_edo_B.b_k_f = 0; dynsim_edo_B.b_k_f < 3; dynsim_edo_B.b_k_f
             ++) {
          dynsim_edo_B.q_size_tmp = dynsim_edo_B.b_k_h + 3 * dynsim_edo_B.b_k_f;
          dynsim_edo_B.R_lx[dynsim_edo_B.q_size_tmp] = 0.0;
          dynsim_edo_B.p = dynsim_edo_B.b_k_f << 2;
          dynsim_edo_B.R_lx[dynsim_edo_B.q_size_tmp] +=
            dynsim_edo_B.T_c[dynsim_edo_B.p] *
            dynsim_edo_B.R_dy[dynsim_edo_B.b_k_h];
          dynsim_edo_B.R_lx[dynsim_edo_B.q_size_tmp] +=
            dynsim_edo_B.T_c[dynsim_edo_B.p + 1] *
            dynsim_edo_B.R_dy[dynsim_edo_B.b_k_h + 3];
          dynsim_edo_B.R_lx[dynsim_edo_B.q_size_tmp] +=
            dynsim_edo_B.T_c[dynsim_edo_B.p + 2] *
            dynsim_edo_B.R_dy[dynsim_edo_B.b_k_h + 6];
          Xtree->data[dynsim_edo_B.unnamed_idx_1].f1[dynsim_edo_B.b_k_f + 6 *
            dynsim_edo_B.b_k_h] = dynsim_edo_B.T_c[(dynsim_edo_B.b_k_h << 2) +
            dynsim_edo_B.b_k_f];
          Xtree->data[dynsim_edo_B.unnamed_idx_1].f1[dynsim_edo_B.b_k_f + 6 *
            (dynsim_edo_B.b_k_h + 3)] = 0.0;
        }
      }

      for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 3; dynsim_edo_B.b_k_h++)
      {
        Xtree->data[dynsim_edo_B.unnamed_idx_1].f1[6 * dynsim_edo_B.b_k_h + 3] =
          dynsim_edo_B.R_lx[3 * dynsim_edo_B.b_k_h];
        dynsim_edo_B.b_k_f = dynsim_edo_B.b_k_h << 2;
        dynsim_edo_B.q_size_tmp = 6 * (dynsim_edo_B.b_k_h + 3);
        Xtree->data[dynsim_edo_B.unnamed_idx_1].f1[dynsim_edo_B.q_size_tmp + 3] =
          dynsim_edo_B.T_c[dynsim_edo_B.b_k_f];
        Xtree->data[dynsim_edo_B.unnamed_idx_1].f1[6 * dynsim_edo_B.b_k_h + 4] =
          dynsim_edo_B.R_lx[3 * dynsim_edo_B.b_k_h + 1];
        Xtree->data[dynsim_edo_B.unnamed_idx_1].f1[dynsim_edo_B.q_size_tmp + 4] =
          dynsim_edo_B.T_c[dynsim_edo_B.b_k_f + 1];
        Xtree->data[dynsim_edo_B.unnamed_idx_1].f1[6 * dynsim_edo_B.b_k_h + 5] =
          dynsim_edo_B.R_lx[3 * dynsim_edo_B.b_k_h + 2];
        Xtree->data[dynsim_edo_B.unnamed_idx_1].f1[dynsim_edo_B.q_size_tmp + 5] =
          dynsim_edo_B.T_c[dynsim_edo_B.b_k_f + 2];
      }
    }

    for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 36; dynsim_edo_B.b_k_h++)
    {
      dynsim_edo_B.b_I[dynsim_edo_B.b_k_h] = robot->
        Bodies[dynsim_edo_B.unnamed_idx_1]->SpatialInertia[dynsim_edo_B.b_k_h];
    }

    dynsim_edo_B.R_dy[0] = 0.0;
    dynsim_edo_B.b_k_f = 6 * dynsim_edo_B.unnamed_idx_1 + 2;
    dynsim_edo_B.R_dy[3] = -vB->data[dynsim_edo_B.b_k_f];
    dynsim_edo_B.b_k_h = 6 * dynsim_edo_B.unnamed_idx_1 + 1;
    dynsim_edo_B.R_dy[6] = vB->data[dynsim_edo_B.b_k_h];
    dynsim_edo_B.R_dy[1] = vB->data[dynsim_edo_B.b_k_f];
    dynsim_edo_B.R_dy[4] = 0.0;
    dynsim_edo_B.R_dy[7] = -vB->data[6 * dynsim_edo_B.unnamed_idx_1];
    dynsim_edo_B.R_dy[2] = -vB->data[dynsim_edo_B.b_k_h];
    dynsim_edo_B.R_dy[5] = vB->data[6 * dynsim_edo_B.unnamed_idx_1];
    dynsim_edo_B.R_dy[8] = 0.0;
    dynsim_edo_B.R[18] = 0.0;
    dynsim_edo_B.b_k_f = 6 * dynsim_edo_B.unnamed_idx_1 + 5;
    dynsim_edo_B.R[24] = -vB->data[dynsim_edo_B.b_k_f];
    dynsim_edo_B.b_k_h = 6 * dynsim_edo_B.unnamed_idx_1 + 4;
    dynsim_edo_B.R[30] = vB->data[dynsim_edo_B.b_k_h];
    dynsim_edo_B.R[19] = vB->data[dynsim_edo_B.b_k_f];
    dynsim_edo_B.R[25] = 0.0;
    dynsim_edo_B.b_k_f = 6 * dynsim_edo_B.unnamed_idx_1 + 3;
    dynsim_edo_B.R[31] = -vB->data[dynsim_edo_B.b_k_f];
    dynsim_edo_B.R[20] = -vB->data[dynsim_edo_B.b_k_h];
    dynsim_edo_B.R[26] = vB->data[dynsim_edo_B.b_k_f];
    dynsim_edo_B.R[32] = 0.0;
    for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 3; dynsim_edo_B.b_k_h++) {
      dynsim_edo_B.a_idx_1 = dynsim_edo_B.R_dy[3 * dynsim_edo_B.b_k_h];
      dynsim_edo_B.R[6 * dynsim_edo_B.b_k_h] = dynsim_edo_B.a_idx_1;
      dynsim_edo_B.R[6 * dynsim_edo_B.b_k_h + 3] = 0.0;
      dynsim_edo_B.b_k_f = 6 * (dynsim_edo_B.b_k_h + 3);
      dynsim_edo_B.R[dynsim_edo_B.b_k_f + 3] = dynsim_edo_B.a_idx_1;
      dynsim_edo_B.a_idx_1 = dynsim_edo_B.R_dy[3 * dynsim_edo_B.b_k_h + 1];
      dynsim_edo_B.R[6 * dynsim_edo_B.b_k_h + 1] = dynsim_edo_B.a_idx_1;
      dynsim_edo_B.R[6 * dynsim_edo_B.b_k_h + 4] = 0.0;
      dynsim_edo_B.R[dynsim_edo_B.b_k_f + 4] = dynsim_edo_B.a_idx_1;
      dynsim_edo_B.a_idx_1 = dynsim_edo_B.R_dy[3 * dynsim_edo_B.b_k_h + 2];
      dynsim_edo_B.R[6 * dynsim_edo_B.b_k_h + 2] = dynsim_edo_B.a_idx_1;
      dynsim_edo_B.R[6 * dynsim_edo_B.b_k_h + 5] = 0.0;
      dynsim_edo_B.R[dynsim_edo_B.b_k_f + 5] = dynsim_edo_B.a_idx_1;
    }

    for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 6; dynsim_edo_B.b_k_h++) {
      dynsim_edo_B.X_e[dynsim_edo_B.b_k_h] = 0.0;
      dynsim_edo_B.b_I_b[dynsim_edo_B.b_k_h] = 0.0;
      for (dynsim_edo_B.b_k_f = 0; dynsim_edo_B.b_k_f < 6; dynsim_edo_B.b_k_f++)
      {
        dynsim_edo_B.a_idx_0 = dynsim_edo_B.b_I[6 * dynsim_edo_B.b_k_f +
          dynsim_edo_B.b_k_h];
        dynsim_edo_B.q_size_tmp = 6 * dynsim_edo_B.unnamed_idx_1 +
          dynsim_edo_B.b_k_f;
        dynsim_edo_B.a_idx_1 = vB->data[dynsim_edo_B.q_size_tmp] *
          dynsim_edo_B.a_idx_0 + dynsim_edo_B.X_e[dynsim_edo_B.b_k_h];
        dynsim_edo_B.a_idx_0 = aB->data[dynsim_edo_B.q_size_tmp] *
          dynsim_edo_B.a_idx_0 + dynsim_edo_B.b_I_b[dynsim_edo_B.b_k_h];
        dynsim_edo_B.X_e[dynsim_edo_B.b_k_h] = dynsim_edo_B.a_idx_1;
        dynsim_edo_B.b_I_b[dynsim_edo_B.b_k_h] = dynsim_edo_B.a_idx_0;
      }
    }

    for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 6; dynsim_edo_B.b_k_h++) {
      dynsim_edo_B.y[dynsim_edo_B.b_k_h] = 0.0;
      dynsim_edo_B.a_idx_1 = 0.0;
      for (dynsim_edo_B.b_k_f = 0; dynsim_edo_B.b_k_f < 6; dynsim_edo_B.b_k_f++)
      {
        dynsim_edo_B.a_idx_1 += Xtree->data[dynsim_edo_B.unnamed_idx_1].f1[6 *
          dynsim_edo_B.b_k_h + dynsim_edo_B.b_k_f] * fext[6 *
          dynsim_edo_B.unnamed_idx_1 + dynsim_edo_B.b_k_f];
        dynsim_edo_B.y[dynsim_edo_B.b_k_h] += dynsim_edo_B.R[6 *
          dynsim_edo_B.b_k_f + dynsim_edo_B.b_k_h] *
          dynsim_edo_B.X_e[dynsim_edo_B.b_k_f];
      }

      f->data[dynsim_edo_B.b_k_h + 6 * dynsim_edo_B.unnamed_idx_1] =
        (dynsim_edo_B.b_I_b[dynsim_edo_B.b_k_h] +
         dynsim_edo_B.y[dynsim_edo_B.b_k_h]) - dynsim_edo_B.a_idx_1;
    }
  }

  dynsim_edo_emxFree_real_T(&aB);
  dynsim_edo_emxFree_real_T(&vB);
  dynsim_edo_emxFree_real_T(&vJ);
  dynsim_edo_emxFree_f_cell_wrap1(&Xtree);
  dynsim_edo_B.q_size_tmp = static_cast<int32_T>(((-1.0 - dynsim_edo_B.nb) + 1.0)
    / -1.0) - 1;
  dynsim_edo_emxInit_real_T(&taui, 1);
  dynsim_edo_emxInit_char_T(&a, 2);
  if (0 <= dynsim_edo_B.q_size_tmp) {
    for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 5; dynsim_edo_B.b_k_h++) {
      dynsim_edo_B.b_h3[dynsim_edo_B.b_k_h] = tmp[dynsim_edo_B.b_k_h];
    }
  }

  for (dynsim_edo_B.loop_ub_tmp = 0; dynsim_edo_B.loop_ub_tmp <=
       dynsim_edo_B.q_size_tmp; dynsim_edo_B.loop_ub_tmp++) {
    dynsim_edo_B.a_idx_0 = dynsim_edo_B.nb + -static_cast<real_T>
      (dynsim_edo_B.loop_ub_tmp);
    dynsim_edo_B.p = static_cast<int32_T>(dynsim_edo_B.a_idx_0);
    dynsim_edo_B.inner = dynsim_edo_B.p - 1;
    obj = robot->Bodies[dynsim_edo_B.inner];
    dynsim_edo_B.b_k_h = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = obj->JointInternal.Type->size[1];
    dynsim_emxEnsureCapacity_char_T(a, dynsim_edo_B.b_k_h);
    dynsim_edo_B.b_k_f = obj->JointInternal.Type->size[0] *
      obj->JointInternal.Type->size[1] - 1;
    for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h <= dynsim_edo_B.b_k_f;
         dynsim_edo_B.b_k_h++) {
      a->data[dynsim_edo_B.b_k_h] = obj->JointInternal.Type->
        data[dynsim_edo_B.b_k_h];
    }

    dynsim_edo_B.b_bool_d = false;
    if (a->size[1] == 5) {
      dynsim_edo_B.b_k_h = 1;
      do {
        exitg1 = 0;
        if (dynsim_edo_B.b_k_h - 1 < 5) {
          dynsim_edo_B.unnamed_idx_1 = dynsim_edo_B.b_k_h - 1;
          if (a->data[dynsim_edo_B.unnamed_idx_1] !=
              dynsim_edo_B.b_h3[dynsim_edo_B.unnamed_idx_1]) {
            exitg1 = 1;
          } else {
            dynsim_edo_B.b_k_h++;
          }
        } else {
          dynsim_edo_B.b_bool_d = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (!dynsim_edo_B.b_bool_d) {
      obj = robot->Bodies[dynsim_edo_B.inner];
      dynsim_edo_B.b_k_h = S->size[0] * S->size[1];
      S->size[0] = 6;
      S->size[1] = obj->JointInternal.MotionSubspace->size[1];
      dynsim_emxEnsureCapacity_real_T(S, dynsim_edo_B.b_k_h);
      dynsim_edo_B.b_k_f = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h <= dynsim_edo_B.b_k_f;
           dynsim_edo_B.b_k_h++) {
        S->data[dynsim_edo_B.b_k_h] = obj->JointInternal.MotionSubspace->
          data[dynsim_edo_B.b_k_h];
      }

      dynsim_edo_B.m = S->size[1] - 1;
      dynsim_edo_B.b_k_h = taui->size[0];
      taui->size[0] = S->size[1];
      dynsim_emxEnsureCapacity_real_T(taui, dynsim_edo_B.b_k_h);
      for (dynsim_edo_B.unnamed_idx_1 = 0; dynsim_edo_B.unnamed_idx_1 <=
           dynsim_edo_B.m; dynsim_edo_B.unnamed_idx_1++) {
        dynsim_edo_B.aoffset = dynsim_edo_B.unnamed_idx_1 * 6 - 1;
        dynsim_edo_B.a_idx_1 = 0.0;
        for (dynsim_edo_B.b_k_f = 0; dynsim_edo_B.b_k_f < 6; dynsim_edo_B.b_k_f
             ++) {
          dynsim_edo_B.a_idx_1 += f->data[(static_cast<int32_T>
            (dynsim_edo_B.a_idx_0) - 1) * 6 + dynsim_edo_B.b_k_f] * S->data
            [(dynsim_edo_B.aoffset + dynsim_edo_B.b_k_f) + 1];
        }

        taui->data[dynsim_edo_B.unnamed_idx_1] = dynsim_edo_B.a_idx_1;
      }

      dynsim_edo_B.b_idx_0 = robot->VelocityDoFMap[dynsim_edo_B.p - 1];
      dynsim_edo_B.b_idx_1 = robot->VelocityDoFMap[dynsim_edo_B.p + 5];
      if (dynsim_edo_B.b_idx_0 > dynsim_edo_B.b_idx_1) {
        dynsim_edo_B.b_k_f = 0;
        dynsim_edo_B.b_k_h = 0;
      } else {
        dynsim_edo_B.b_k_f = static_cast<int32_T>(dynsim_edo_B.b_idx_0) - 1;
        dynsim_edo_B.b_k_h = static_cast<int32_T>(dynsim_edo_B.b_idx_1);
      }

      dynsim_edo_B.unnamed_idx_1 = dynsim_edo_B.b_k_h - dynsim_edo_B.b_k_f;
      for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h <
           dynsim_edo_B.unnamed_idx_1; dynsim_edo_B.b_k_h++) {
        tau[dynsim_edo_B.b_k_f + dynsim_edo_B.b_k_h] = taui->
          data[dynsim_edo_B.b_k_h];
      }
    }

    dynsim_edo_B.a_idx_0 = robot->Bodies[dynsim_edo_B.inner]->ParentIndex;
    if (dynsim_edo_B.a_idx_0 > 0.0) {
      dynsim_edo_B.m = static_cast<int32_T>(dynsim_edo_B.a_idx_0);
      for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 6; dynsim_edo_B.b_k_h++)
      {
        dynsim_edo_B.a_idx_1 = 0.0;
        for (dynsim_edo_B.b_k_f = 0; dynsim_edo_B.b_k_f < 6; dynsim_edo_B.b_k_f
             ++) {
          dynsim_edo_B.a_idx_1 += f->data[(dynsim_edo_B.p - 1) * 6 +
            dynsim_edo_B.b_k_f] * X->data[dynsim_edo_B.inner].f1[6 *
            dynsim_edo_B.b_k_h + dynsim_edo_B.b_k_f];
        }

        dynsim_edo_B.a0[dynsim_edo_B.b_k_h] = f->data[(dynsim_edo_B.m - 1) * 6 +
          dynsim_edo_B.b_k_h] + dynsim_edo_B.a_idx_1;
      }

      for (dynsim_edo_B.b_k_h = 0; dynsim_edo_B.b_k_h < 6; dynsim_edo_B.b_k_h++)
      {
        f->data[dynsim_edo_B.b_k_h + 6 * (dynsim_edo_B.m - 1)] =
          dynsim_edo_B.a0[dynsim_edo_B.b_k_h];
      }
    }
  }

  dynsim_edo_emxFree_char_T(&a);
  dynsim_edo_emxFree_real_T(&taui);
  dynsim_edo_emxFree_real_T(&S);
  dynsim_edo_emxFree_real_T(&f);
  dynsim_edo_emxFree_f_cell_wrap1(&X);
}

static void matlabCodegenHandle_matlab_hau4(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynsim_edo_T
  *pStruct)
{
  dynsim_edo_emxFree_char_T(&pStruct->Type);
  dynsim_edo_emxFree_real_T(&pStruct->MotionSubspace);
}

static void emxFreeStruct_m_robotics_manip_(m_robotics_manip_internal_Rig_T
  *pStruct)
{
  dynsim_edo_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxFreeStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_m_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxFreeStruct_n_robotics_manip_(&pStruct->TreeInternal);
}

static void emxFreeStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_dynsim_edo_h_T
  *pStruct)
{
  dynsim_edo_emxFree_char_T(&pStruct->Type);
}

static void emxFreeStruct_m_robotics_mani_h(m_robotics_manip_internal_R_h_T
  *pStruct)
{
  dynsim_edo_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxFreeStruct_n_robotics_mani_h(n_robotics_manip_internal_R_h_T
  *pStruct)
{
  emxFreeStruct_m_robotics_mani_h(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmani_h(robotics_slmanip_internal_b_h_T
  *pStruct)
{
  emxFreeStruct_n_robotics_mani_h(&pStruct->TreeInternal);
}

static void emxFreeStruct_l_robotics_mani_h(l_robotics_manip_internal_R_h_T
  *pStruct)
{
  emxFreeStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void matlabCodegenHandle_matlabC_hau(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxFreeStruct_c_rigidBodyJoint2(c_rigidBodyJoint_dynsim_ed_ha_T
  *pStruct)
{
  dynsim_edo_emxFree_char_T(&pStruct->Type);
  dynsim_edo_emxFree_real_T(&pStruct->MotionSubspace);
}

static void emxFreeStruct_m_robotics_man_ha(m_robotics_manip_internal__ha_T
  *pStruct)
{
  emxFreeStruct_c_rigidBodyJoint2(&pStruct->JointInternal);
}

static void emxFreeStruct_n_robotics_man_ha(n_robotics_manip_internal__ha_T
  *pStruct)
{
  emxFreeStruct_m_robotics_man_ha(&pStruct->Base);
}

static void emxFreeStruct_robotics_slman_ha(robotics_slmanip_internal__ha_T
  *pStruct)
{
  emxFreeStruct_n_robotics_man_ha(&pStruct->TreeInternal);
}

static void emxFreeStruct_l_robotics_man_ha(l_robotics_manip_internal__ha_T
  *pStruct)
{
  emxFreeStruct_c_rigidBodyJoint2(&pStruct->JointInternal);
}

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynsim_edo_T
  *pStruct)
{
  dynsim_edo_emxInit_char_T(&pStruct->Type, 2);
  dynsim_edo_emxInit_real_T(&pStruct->MotionSubspace, 2);
}

static void emxInitStruct_m_robotics_manip_(m_robotics_manip_internal_Rig_T
  *pStruct)
{
  dynsim_edo_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxInitStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_m_robotics_manip_(&pStruct->Base);
}

static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxInitStruct_n_robotics_manip_(&pStruct->TreeInternal);
}

static void emxInitStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static l_robotics_manip_internal_Rig_T *dynsim_edo_RigidBody_RigidBody
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_edo_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[6] = { 'l', 'i', 'n', 'k', '_', '1' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { -0.99999999997301514, 7.3464102066435871E-6,
    -6.9389E-16, 0.0, 2.6984177572320606E-11, 3.6732051032474579E-6,
    0.99999999999325373, 0.0, 7.3464102065940289E-6, 0.99999999996626887,
    -3.6732051033465739E-6, 0.0, 0.057188, 0.0059831, 0.13343, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->NameInternal[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 0.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *dynsim_ed_RigidBody_RigidBody_h
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_edo_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[6] = { 'l', 'i', 'n', 'k', '_', '2' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 0.88847119465357549, -0.10400474353252914,
    0.44699211356978236, 0.0, -0.29079372810919513, 0.62592701137950757,
    0.7236396783744472, 0.0, -0.35504639691623957, -0.77291551268444,
    0.52587419245342837, 0.0, 0.0, 0.18967, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { -0.88847134048214615, 0.29080043874549294,
    0.3550405356678123, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->NameInternal[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 1.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = -0.88847134048214615;
  obj->JointInternal.JointAxisInternal[1] = 0.29080043874549294;
  obj->JointInternal.JointAxisInternal[2] = 0.3550405356678123;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *dynsim_e_RigidBody_RigidBody_ha
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_edo_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[6] = { 'l', 'i', 'n', 'k', '_', '3' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { -0.88847685238438157, 0.29078066152542581,
    0.35504294058603364, 0.0, 0.10401076381482063, -0.62592577039760033,
    0.77291570754049777, 0.0, 0.44697946685256096, 0.72364600243144184,
    0.5258762396012906, 0.0, -0.024558, 0.12737, -0.16578, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->NameInternal[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 2.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *dynsim__RigidBody_RigidBody_hau
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_edo_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[6] = { 'l', 'i', 'n', 'k', '_', '4' };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -3.6732051033465739E-6, -0.99999999999325373, 0.0, -0.0, 0.99999999999325373,
    -3.6732051033465739E-6, 0.0, 0.0088, -0.1588, 0.0, 1.0 };

  static const real_T tmp_5[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->NameInternal[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 3.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_2[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_3[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_4[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_5[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *dynsim_RigidBody_RigidBody_hau4
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_edo_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[6] = { 'l', 'i', 'n', 'k', '_', '5' };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_4[16] = { -0.99999999997301514, -7.3464102066435871E-6,
    -1.1102E-14, 0.0, -7.3464102064452637E-6, 0.99999999994603028,
    -7.3464102066435871E-6, 0.0, 5.3980844924276475E-11, -7.3464102064452637E-6,
    -0.99999999997301514, 0.0, 0.0, 0.0, -0.1053, 1.0 };

  static const real_T tmp_5[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->NameInternal[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 4.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_2[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_3[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_4[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_5[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *dynsi_RigidBody_RigidBody_hau4i
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_edo_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[6] = { 'l', 'i', 'n', 'k', '_', '6' };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_4[16] = { 1.0, 0.0, -1.249E-14, 0.0,
    -1.2489999999915739E-14, -3.6732051033465739E-6, -0.99999999999325373, 0.0,
    -4.587833174079871E-20, 0.99999999999325373, -3.6732051033465739E-6, 0.0,
    -0.0039, 0.0, 0.1636, 1.0 };

  static const real_T tmp_5[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->NameInternal[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 5.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_2[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_3[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_4[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_5[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static m_robotics_manip_internal_Rig_T *dyns_RigidBody_RigidBody_hau4ih
  (m_robotics_manip_internal_Rig_T *obj)
{
  m_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_edo_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[9] = { 'b', 'a', 's', 'e', '_', 'l', 'i', 'n', 'k' };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 9;
  dynsim_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_2[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_3[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  return b_obj;
}

static n_robotics_manip_internal_Rig_T *dyn_RigidBodyTree_RigidBodyTree
  (n_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Rig_T *iobj_0,
   l_robotics_manip_internal_Rig_T *iobj_1, l_robotics_manip_internal_Rig_T
   *iobj_2, l_robotics_manip_internal_Rig_T *iobj_3,
   l_robotics_manip_internal_Rig_T *iobj_4, l_robotics_manip_internal_Rig_T
   *iobj_5)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int32_T i;
  static const int8_T tmp[12] = { 1, 2, 3, 0, 0, 0, 1, 2, 3, -1, -1, -1 };

  b_obj = obj;
  obj->Bodies[0] = dynsim_edo_RigidBody_RigidBody(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = dynsim_ed_RigidBody_RigidBody_h(iobj_5);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = dynsim_e_RigidBody_RigidBody_ha(iobj_1);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = dynsim__RigidBody_RigidBody_hau(iobj_2);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = dynsim_RigidBody_RigidBody_hau4(iobj_3);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = dynsi_RigidBody_RigidBody_hau4i(iobj_4);
  obj->Bodies[5]->Index = 6.0;
  obj->NumBodies = 6.0;
  obj->PositionNumber = 3.0;
  obj->VelocityNumber = 3.0;
  for (i = 0; i < 12; i++) {
    obj->PositionDoFMap[i] = tmp[i];
  }

  dyns_RigidBody_RigidBody_hau4ih(&obj->Base);
  return b_obj;
}

static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_dynsim_edo_h_T
  *pStruct)
{
  dynsim_edo_emxInit_char_T(&pStruct->Type, 2);
}

static void emxInitStruct_m_robotics_mani_h(m_robotics_manip_internal_R_h_T
  *pStruct)
{
  dynsim_edo_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxInitStruct_n_robotics_mani_h(n_robotics_manip_internal_R_h_T
  *pStruct)
{
  emxInitStruct_m_robotics_mani_h(&pStruct->Base);
}

static void emxInitStruct_robotics_slmani_h(robotics_slmanip_internal_b_h_T
  *pStruct)
{
  emxInitStruct_n_robotics_mani_h(&pStruct->TreeInternal);
}

static void emxInitStruct_l_robotics_mani_h(l_robotics_manip_internal_R_h_T
  *pStruct)
{
  emxInitStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static l_robotics_manip_internal_R_h_T *dyn_RigidBody_RigidBody_hau4ihh
  (l_robotics_manip_internal_R_h_T *obj)
{
  l_robotics_manip_internal_R_h_T *b_obj;
  emxArray_char_T_dynsim_edo_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[6] = { 'l', 'i', 'n', 'k', '_', '1' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { -0.99999999997301514, 7.3464102066435871E-6,
    -6.9389E-16, 0.0, 2.6984177572320606E-11, 3.6732051032474579E-6,
    0.99999999999325373, 0.0, 7.3464102065940289E-6, 0.99999999996626887,
    -3.6732051033465739E-6, 0.0, 0.057188, 0.0059831, 0.13343, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->NameInternal[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 0.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_0[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_1[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal_R_h_T *dy_RigidBody_RigidBody_hau4ihh0
  (l_robotics_manip_internal_R_h_T *obj)
{
  l_robotics_manip_internal_R_h_T *b_obj;
  emxArray_char_T_dynsim_edo_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[6] = { 'l', 'i', 'n', 'k', '_', '2' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { 0.88847119465357549, -0.10400474353252914,
    0.44699211356978236, 0.0, -0.29079372810919513, 0.62592701137950757,
    0.7236396783744472, 0.0, -0.35504639691623957, -0.77291551268444,
    0.52587419245342837, 0.0, 0.0, 0.18967, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->NameInternal[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 1.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_0[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_1[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = -0.88847134048214615;
  obj->JointInternal.JointAxisInternal[1] = 0.29080043874549294;
  obj->JointInternal.JointAxisInternal[2] = 0.3550405356678123;
  return b_obj;
}

static l_robotics_manip_internal_R_h_T *d_RigidBody_RigidBody_hau4ihh0a
  (l_robotics_manip_internal_R_h_T *obj)
{
  l_robotics_manip_internal_R_h_T *b_obj;
  emxArray_char_T_dynsim_edo_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[6] = { 'l', 'i', 'n', 'k', '_', '3' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { -0.88847685238438157, 0.29078066152542581,
    0.35504294058603364, 0.0, 0.10401076381482063, -0.62592577039760033,
    0.77291570754049777, 0.0, 0.44697946685256096, 0.72364600243144184,
    0.5258762396012906, 0.0, -0.024558, 0.12737, -0.16578, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->NameInternal[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 2.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_0[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_1[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static n_robotics_manip_internal_R_h_T *d_RigidBodyTree_RigidBodyTree_h
  (n_robotics_manip_internal_R_h_T *obj, l_robotics_manip_internal_R_h_T *iobj_0,
   l_robotics_manip_internal_R_h_T *iobj_1, l_robotics_manip_internal_R_h_T
   *iobj_2, l_robotics_manip_internal_R_h_T *iobj_3,
   l_robotics_manip_internal_R_h_T *iobj_4, l_robotics_manip_internal_R_h_T
   *iobj_5)
{
  n_robotics_manip_internal_R_h_T *b_obj;
  m_robotics_manip_internal_R_h_T *obj_0;
  emxArray_char_T_dynsim_edo_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[6] = { 'l', 'i', 'n', 'k', '_', '4' };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -3.6732051033465739E-6, -0.99999999999325373, 0.0, -0.0, 0.99999999999325373,
    -3.6732051033465739E-6, 0.0, 0.0088, -0.1588, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const char_T tmp_5[6] = { 'l', 'i', 'n', 'k', '_', '5' };

  static const real_T tmp_6[16] = { -0.99999999997301514, -7.3464102066435871E-6,
    -1.1102E-14, 0.0, -7.3464102064452637E-6, 0.99999999994603028,
    -7.3464102066435871E-6, 0.0, 5.3980844924276475E-11, -7.3464102064452637E-6,
    -0.99999999997301514, 0.0, 0.0, 0.0, -0.1053, 1.0 };

  static const char_T tmp_7[6] = { 'l', 'i', 'n', 'k', '_', '6' };

  static const real_T tmp_8[16] = { 1.0, 0.0, -1.249E-14, 0.0,
    -1.2489999999915739E-14, -3.6732051033465739E-6, -0.99999999999325373, 0.0,
    -4.587833174079871E-20, 0.99999999999325373, -3.6732051033465739E-6, 0.0,
    -0.0039, 0.0, 0.1636, 1.0 };

  static const char_T tmp_9[9] = { 'b', 'a', 's', 'e', '_', 'l', 'i', 'n', 'k' };

  int32_T exitg1;
  b_obj = obj;
  obj->Bodies[0] = dyn_RigidBody_RigidBody_hau4ihh(iobj_0);
  obj->Bodies[1] = dy_RigidBody_RigidBody_hau4ihh0(iobj_5);
  obj->Bodies[2] = d_RigidBody_RigidBody_hau4ihh0a(iobj_1);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_2->NameInternal[b_kstr] = tmp[b_kstr];
  }

  iobj_2->ParentIndex = 3.0;
  b_kstr = iobj_2->JointInternal.Type->size[0] * iobj_2->
    JointInternal.Type->size[1];
  iobj_2->JointInternal.Type->size[0] = 1;
  iobj_2->JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(iobj_2->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_2->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_2->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_2->JointInternal.Type->size[0] * iobj_2->
    JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_2->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  switch (b_kstr) {
   case 0:
    iobj_2->JointInternal.PositionNumber = 1.0;
    iobj_2->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_2->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_2->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    iobj_2->JointInternal.PositionNumber = 1.0;
    iobj_2->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_2->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_2->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    iobj_2->JointInternal.PositionNumber = 0.0;
    iobj_2->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_2->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_2->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_2->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_2->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  iobj_2->JointInternal.JointAxisInternal[0] = 0.0;
  iobj_2->JointInternal.JointAxisInternal[1] = 0.0;
  iobj_2->JointInternal.JointAxisInternal[2] = 0.0;
  obj->Bodies[3] = iobj_2;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_3->NameInternal[b_kstr] = tmp_5[b_kstr];
  }

  iobj_3->ParentIndex = 4.0;
  b_kstr = iobj_3->JointInternal.Type->size[0] * iobj_3->
    JointInternal.Type->size[1];
  iobj_3->JointInternal.Type->size[0] = 1;
  iobj_3->JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(iobj_3->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_3->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_3->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_3->JointInternal.Type->size[0] * iobj_3->
    JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_3->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  switch (b_kstr) {
   case 0:
    iobj_3->JointInternal.PositionNumber = 1.0;
    iobj_3->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_3->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_3->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    iobj_3->JointInternal.PositionNumber = 1.0;
    iobj_3->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_3->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_3->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    iobj_3->JointInternal.PositionNumber = 0.0;
    iobj_3->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_3->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_3->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_3->JointInternal.JointToParentTransform[b_kstr] = tmp_6[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_3->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  iobj_3->JointInternal.JointAxisInternal[0] = 0.0;
  iobj_3->JointInternal.JointAxisInternal[1] = 0.0;
  iobj_3->JointInternal.JointAxisInternal[2] = 0.0;
  obj->Bodies[4] = iobj_3;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_4->NameInternal[b_kstr] = tmp_7[b_kstr];
  }

  iobj_4->ParentIndex = 5.0;
  b_kstr = iobj_4->JointInternal.Type->size[0] * iobj_4->
    JointInternal.Type->size[1];
  iobj_4->JointInternal.Type->size[0] = 1;
  iobj_4->JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(iobj_4->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_4->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_4->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_4->JointInternal.Type->size[0] * iobj_4->
    JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_4->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  switch (b_kstr) {
   case 0:
    iobj_4->JointInternal.PositionNumber = 1.0;
    iobj_4->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_4->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_4->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    iobj_4->JointInternal.PositionNumber = 1.0;
    iobj_4->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_4->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_4->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    iobj_4->JointInternal.PositionNumber = 0.0;
    iobj_4->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_4->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_4->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_4->JointInternal.JointToParentTransform[b_kstr] = tmp_8[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_4->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  iobj_4->JointInternal.JointAxisInternal[0] = 0.0;
  iobj_4->JointInternal.JointAxisInternal[1] = 0.0;
  iobj_4->JointInternal.JointAxisInternal[2] = 0.0;
  obj->Bodies[5] = iobj_4;
  obj->NumBodies = 6.0;
  obj->PositionNumber = 3.0;
  obj_0 = &obj->Base;
  b_kstr = obj->Base.NameInternal->size[0] * obj->Base.NameInternal->size[1];
  obj->Base.NameInternal->size[0] = 1;
  obj->Base.NameInternal->size[1] = 9;
  dynsim_emxEnsureCapacity_char_T(obj->Base.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->Base.NameInternal->data[b_kstr] = tmp_9[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1];
  obj->Base.JointInternal.Type->size[0] = 1;
  obj->Base.JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(obj->Base.JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj_0->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Base.JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj_0->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    obj->Base.JointInternal.PositionNumber = 1.0;
    obj->Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->Base.JointInternal.PositionNumber = 1.0;
    obj->Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->Base.JointInternal.PositionNumber = 0.0;
    obj->Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  return b_obj;
}

static void emxInitStruct_c_rigidBodyJoint2(c_rigidBodyJoint_dynsim_ed_ha_T
  *pStruct)
{
  dynsim_edo_emxInit_char_T(&pStruct->Type, 2);
  dynsim_edo_emxInit_real_T(&pStruct->MotionSubspace, 2);
}

static void emxInitStruct_m_robotics_man_ha(m_robotics_manip_internal__ha_T
  *pStruct)
{
  emxInitStruct_c_rigidBodyJoint2(&pStruct->JointInternal);
}

static void emxInitStruct_n_robotics_man_ha(n_robotics_manip_internal__ha_T
  *pStruct)
{
  emxInitStruct_m_robotics_man_ha(&pStruct->Base);
}

static void emxInitStruct_robotics_slman_ha(robotics_slmanip_internal__ha_T
  *pStruct)
{
  emxInitStruct_n_robotics_man_ha(&pStruct->TreeInternal);
}

static void emxInitStruct_l_robotics_man_ha(l_robotics_manip_internal__ha_T
  *pStruct)
{
  emxInitStruct_c_rigidBodyJoint2(&pStruct->JointInternal);
}

static l_robotics_manip_internal__ha_T *RigidBody_RigidBody_hau4ihh0a3
  (l_robotics_manip_internal__ha_T *obj)
{
  l_robotics_manip_internal__ha_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_edo_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.0002, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0,
    6.29E-5, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0002, -0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0785942338762368, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0785942338762368,
    0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0785942338762368 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { -0.99999999997301514, 7.3464102066435871E-6,
    -6.9389E-16, 0.0, 2.6984177572320606E-11, 3.6732051032474579E-6,
    0.99999999999325373, 0.0, 7.3464102065940289E-6, 0.99999999996626887,
    -3.6732051033465739E-6, 0.0, 0.057188, 0.0059831, 0.13343, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 0.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal__ha_T *RigidBody_RigidBody_hau4ihh0a3c
  (l_robotics_manip_internal__ha_T *obj)
{
  l_robotics_manip_internal__ha_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_edo_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 9.15E-5, 3.51E-5, 4.25E-5, 0.0, -0.0, 0.0,
    3.51E-5, 0.0001836, -1.39E-5, 0.0, 0.0, -0.0, 4.25E-5, -1.39E-5, 0.0001784,
    -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0785942338762368, 0.0, 0.0, -0.0, 0.0, 0.0,
    0.0, 0.0785942338762368, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0785942338762368 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 0.88847119465357549, -0.10400474353252914,
    0.44699211356978236, 0.0, -0.29079372810919513, 0.62592701137950757,
    0.7236396783744472, 0.0, -0.35504639691623957, -0.77291551268444,
    0.52587419245342837, 0.0, 0.0, 0.18967, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { -0.88847134048214615, 0.29080043874549294,
    0.3550405356678123, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 1.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = -0.88847134048214615;
  obj->JointInternal.JointAxisInternal[1] = 0.29080043874549294;
  obj->JointInternal.JointAxisInternal[2] = 0.3550405356678123;
  return b_obj;
}

static l_robotics_manip_internal__ha_T *RigidBody_RigidBod_hau4ihh0a3ct
  (l_robotics_manip_internal__ha_T *obj)
{
  l_robotics_manip_internal__ha_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_edo_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 6.29E-5, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0,
    0.0002, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0002, -0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0785942338762368, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0785942338762368,
    0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0785942338762368 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { -0.88847685238438157, 0.29078066152542581,
    0.35504294058603364, 0.0, 0.10401076381482063, -0.62592577039760033,
    0.77291570754049777, 0.0, 0.44697946685256096, 0.72364600243144184,
    0.5258762396012906, 0.0, -0.024558, 0.12737, -0.16578, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 2.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal__ha_T *RigidBody_RigidBo_hau4ihh0a3ctk
  (l_robotics_manip_internal__ha_T *obj)
{
  l_robotics_manip_internal__ha_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_edo_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.0002, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0,
    0.0002, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 6.29E-5, -0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0785942338762368, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0785942338762368,
    0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0785942338762368 };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -3.6732051033465739E-6, -0.99999999999325373, 0.0, -0.0, 0.99999999999325373,
    -3.6732051033465739E-6, 0.0, 0.0088, -0.1588, 0.0, 1.0 };

  static const real_T tmp_5[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 3.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_2[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_3[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_4[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_5[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal__ha_T *RigidBody_RigidB_hau4ihh0a3ctk5
  (l_robotics_manip_internal__ha_T *obj)
{
  l_robotics_manip_internal__ha_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_edo_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 6.29E-5, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0,
    0.0002, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0002, -0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0785942338762368, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0785942338762368,
    0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0785942338762368 };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_4[16] = { -0.99999999997301514, -7.3464102066435871E-6,
    -1.1102E-14, 0.0, -7.3464102064452637E-6, 0.99999999994603028,
    -7.3464102066435871E-6, 0.0, 5.3980844924276475E-11, -7.3464102064452637E-6,
    -0.99999999997301514, 0.0, 0.0, 0.0, -0.1053, 1.0 };

  static const real_T tmp_5[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 4.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_2[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_3[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_4[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_5[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal__ha_T *RigidBody_Rigid_hau4ihh0a3ctk52
  (l_robotics_manip_internal__ha_T *obj)
{
  l_robotics_manip_internal__ha_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_edo_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 7.0E-5, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0,
    2.2E-5, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 7.0E-5, -0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0279702497322662, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0279702497322662,
    0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0279702497322662 };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_4[16] = { 1.0, 0.0, -1.249E-14, 0.0,
    -1.2489999999915739E-14, -3.6732051033465739E-6, -0.99999999999325373, 0.0,
    -4.587833174079871E-20, 0.99999999999325373, -3.6732051033465739E-6, 0.0,
    -0.0039, 0.0, 0.1636, 1.0 };

  static const real_T tmp_5[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 5.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_2[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_3[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_4[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_5[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static m_robotics_manip_internal__ha_T *d_RigidBody_Rigid_c
  (m_robotics_manip_internal__ha_T *obj)
{
  m_robotics_manip_internal__ha_T *b_obj;
  emxArray_char_T_dynsim_edo_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      dynsim_edo_B.b_md[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != dynsim_edo_B.b_md[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_edo_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      dynsim_edo_B.msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      dynsim_edo_B.msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      dynsim_edo_B.msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] =
      dynsim_edo_B.msubspace_data[b_kstr];
  }

  return b_obj;
}

static n_robotics_manip_internal__ha_T *RigidBodyTree_RigidBodyTree_ha
  (n_robotics_manip_internal__ha_T *obj, l_robotics_manip_internal__ha_T *iobj_0,
   l_robotics_manip_internal__ha_T *iobj_1, l_robotics_manip_internal__ha_T
   *iobj_2, l_robotics_manip_internal__ha_T *iobj_3,
   l_robotics_manip_internal__ha_T *iobj_4, l_robotics_manip_internal__ha_T
   *iobj_5)
{
  n_robotics_manip_internal__ha_T *b_obj;
  int32_T i;
  static const int8_T tmp[12] = { 1, 2, 3, 0, 0, 0, 1, 2, 3, -1, -1, -1 };

  b_obj = obj;
  obj->Bodies[0] = RigidBody_RigidBody_hau4ihh0a3(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = RigidBody_RigidBody_hau4ihh0a3c(iobj_5);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = RigidBody_RigidBod_hau4ihh0a3ct(iobj_1);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = RigidBody_RigidBo_hau4ihh0a3ctk(iobj_2);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = RigidBody_RigidB_hau4ihh0a3ctk5(iobj_3);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = RigidBody_Rigid_hau4ihh0a3ctk52(iobj_4);
  obj->Bodies[5]->Index = 6.0;
  obj->NumBodies = 6.0;
  obj->Gravity[0] = 0.0;
  obj->Gravity[1] = 0.0;
  obj->Gravity[2] = 0.0;
  obj->VelocityNumber = 3.0;
  for (i = 0; i < 12; i++) {
    obj->PositionDoFMap[i] = tmp[i];
  }

  for (i = 0; i < 12; i++) {
    obj->VelocityDoFMap[i] = tmp[i];
  }

  d_RigidBody_Rigid_c(&obj->Base);
  return b_obj;
}

// Model step function
void dynsim_edo_step(void)
{
  emxArray_real_T_dynsim_edo_T *b;
  robotics_slmanip_internal_b_h_T *obj;
  n_robotics_manip_internal_R_h_T *obj_0;
  emxArray_f_cell_wrap_dynsim_e_T *Ttree;
  emxArray_char_T_dynsim_edo_T *bname;
  l_robotics_manip_internal_R_h_T *obj_1;
  robotics_slmanip_internal__ha_T *obj_2;
  emxArray_real_T_dynsim_edo_T *L;
  emxArray_real_T_dynsim_edo_T *lambda;
  emxArray_real_T_dynsim_edo_T *H;
  emxArray_real_T_dynsim_edo_T *tmp;
  static const char_T tmp_0[6] = { 'l', 'i', 'n', 'k', '_', '4' };

  static const char_T e[7] = { 'x', '_', 'c', 'o', 'o', 'r', 'd' };

  static const char_T f[7] = { 'y', '_', 'c', 'o', 'o', 'r', 'd' };

  static const char_T g[7] = { 'z', '_', 'c', 'o', 'o', 'r', 'd' };

  static const char_T e_0[7] = { 'j', 'o', 'i', 'n', 't', '_', '1' };

  static const char_T f_0[7] = { 'j', 'o', 'i', 'n', 't', '_', '2' };

  static const char_T g_0[7] = { 'j', 'o', 'i', 'n', 't', '_', '3' };

  int32_T exitg1;
  boolean_T exitg2;
  if (rtmIsMajorTimeStep(dynsim_edo_M)) {
    // set solver stop time
    rtsiSetSolverStopTime(&dynsim_edo_M->solverInfo,
                          ((dynsim_edo_M->Timing.clockTick0+1)*
      dynsim_edo_M->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep(dynsim_edo_M)) {
    dynsim_edo_M->Timing.t[0] = rtsiGetT(&dynsim_edo_M->solverInfo);
  }

  // MATLABSystem: '<S16>/Get Parameter'
  ParamGet_dynsim_edo_112.get_parameter(&dynsim_edo_B.bid1);

  // MATLABSystem: '<S16>/Get Parameter1'
  ParamGet_dynsim_edo_113.get_parameter(&dynsim_edo_B.k);

  // MATLABSystem: '<S16>/Get Parameter4'
  ParamGet_dynsim_edo_125.get_parameter(&dynsim_edo_B.j);

  // Integrator: '<S13>/Position' incorporates:
  //   MATLABSystem: '<S16>/Get Parameter'
  //   MATLABSystem: '<S16>/Get Parameter1'
  //   MATLABSystem: '<S16>/Get Parameter4'

  if (dynsim_edo_DW.Position_IWORK != 0) {
    dynsim_edo_X.Position_CSTATE[0] = dynsim_edo_B.bid1;
    dynsim_edo_X.Position_CSTATE[1] = dynsim_edo_B.k;
    dynsim_edo_X.Position_CSTATE[2] = dynsim_edo_B.j;
  }

  dynsim_edo_emxInit_real_T(&b, 2);
  dynsim_edo_emxInit_f_cell_wrap(&Ttree, 2);
  dynsim_edo_emxInit_char_T(&bname, 2);

  // MATLABSystem: '<S4>/MATLAB System' incorporates:
  //   Integrator: '<S13>/Position'

  RigidBodyTree_geometricJacobian(&dynsim_edo_DW.obj_a.TreeInternal,
    dynsim_edo_X.Position_CSTATE, b);

  // MATLABSystem: '<S5>/MATLAB System' incorporates:
  //   Integrator: '<S13>/Position'

  obj = &dynsim_edo_DW.obj_o;
  obj_0 = &dynsim_edo_DW.obj_o.TreeInternal;
  RigidBodyTree_forwardKinemati_h(&obj->TreeInternal,
    dynsim_edo_X.Position_CSTATE, Ttree);
  dynsim_edo_B.bid1 = -1.0;
  dynsim_edo_B.ret = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj_0->Base.NameInternal->size[1];
  dynsim_emxEnsureCapacity_char_T(bname, dynsim_edo_B.ret);
  dynsim_edo_B.loop_ub = obj_0->Base.NameInternal->size[0] *
    obj_0->Base.NameInternal->size[1] - 1;
  for (dynsim_edo_B.ret = 0; dynsim_edo_B.ret <= dynsim_edo_B.loop_ub;
       dynsim_edo_B.ret++) {
    bname->data[dynsim_edo_B.ret] = obj_0->Base.NameInternal->
      data[dynsim_edo_B.ret];
  }

  for (dynsim_edo_B.ret = 0; dynsim_edo_B.ret < 6; dynsim_edo_B.ret++) {
    dynsim_edo_B.b_hh[dynsim_edo_B.ret] = tmp_0[dynsim_edo_B.ret];
  }

  dynsim_edo_B.b_bool = false;
  if (bname->size[1] == 6) {
    dynsim_edo_B.loop_ub = 1;
    do {
      exitg1 = 0;
      if (dynsim_edo_B.loop_ub - 1 < 6) {
        dynsim_edo_B.ret = dynsim_edo_B.loop_ub - 1;
        if (bname->data[dynsim_edo_B.ret] != dynsim_edo_B.b_hh[dynsim_edo_B.ret])
        {
          exitg1 = 1;
        } else {
          dynsim_edo_B.loop_ub++;
        }
      } else {
        dynsim_edo_B.b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  dynsim_edo_emxFree_char_T(&bname);

  // MATLABSystem: '<S5>/MATLAB System'
  if (dynsim_edo_B.b_bool) {
    dynsim_edo_B.bid1 = 0.0;
  } else {
    dynsim_edo_B.k = obj->TreeInternal.NumBodies;
    dynsim_edo_B.loop_ub = 0;
    exitg2 = false;
    while ((!exitg2) && (dynsim_edo_B.loop_ub <= static_cast<int32_T>
                         (dynsim_edo_B.k) - 1)) {
      obj_1 = obj_0->Bodies[dynsim_edo_B.loop_ub];
      for (dynsim_edo_B.ret = 0; dynsim_edo_B.ret < 6; dynsim_edo_B.ret++) {
        dynsim_edo_B.bname[dynsim_edo_B.ret] = obj_1->
          NameInternal[dynsim_edo_B.ret];
      }

      for (dynsim_edo_B.ret = 0; dynsim_edo_B.ret < 6; dynsim_edo_B.ret++) {
        dynsim_edo_B.b_hh[dynsim_edo_B.ret] = tmp_0[dynsim_edo_B.ret];
      }

      dynsim_edo_B.ret = memcmp(&dynsim_edo_B.bname[0], &dynsim_edo_B.b_hh[0], 6);
      if (dynsim_edo_B.ret == 0) {
        dynsim_edo_B.bid1 = static_cast<real_T>(dynsim_edo_B.loop_ub) + 1.0;
        exitg2 = true;
      } else {
        dynsim_edo_B.loop_ub++;
      }
    }
  }

  if (dynsim_edo_B.bid1 == 0.0) {
    memset(&dynsim_edo_B.T1[0], 0, sizeof(real_T) << 4U);
    dynsim_edo_B.T1[0] = 1.0;
    dynsim_edo_B.T1[5] = 1.0;
    dynsim_edo_B.T1[10] = 1.0;
    dynsim_edo_B.T1[15] = 1.0;
  } else {
    for (dynsim_edo_B.ret = 0; dynsim_edo_B.ret < 16; dynsim_edo_B.ret++) {
      dynsim_edo_B.T1[dynsim_edo_B.ret] = Ttree->data[static_cast<int32_T>
        (dynsim_edo_B.bid1) - 1].f1[dynsim_edo_B.ret];
    }
  }

  dynsim_edo_emxFree_f_cell_wrap(&Ttree);

  // MATLABSystem: '<S5>/MATLAB System'
  for (dynsim_edo_B.ret = 0; dynsim_edo_B.ret < 16; dynsim_edo_B.ret++) {
    dynsim_edo_B.T2_i[dynsim_edo_B.ret] = 0;
  }

  dynsim_edo_B.T2_i[0] = 1;
  dynsim_edo_B.T2_i[5] = 1;
  dynsim_edo_B.T2_i[10] = 1;
  dynsim_edo_B.T2_i[15] = 1;
  for (dynsim_edo_B.ret = 0; dynsim_edo_B.ret < 3; dynsim_edo_B.ret++) {
    dynsim_edo_B.R_j[3 * dynsim_edo_B.ret] = dynsim_edo_B.T2_i[dynsim_edo_B.ret];
    dynsim_edo_B.R_j[3 * dynsim_edo_B.ret + 1] =
      dynsim_edo_B.T2_i[dynsim_edo_B.ret + 4];
    dynsim_edo_B.R_j[3 * dynsim_edo_B.ret + 2] =
      dynsim_edo_B.T2_i[dynsim_edo_B.ret + 8];
  }

  for (dynsim_edo_B.ret = 0; dynsim_edo_B.ret < 9; dynsim_edo_B.ret++) {
    dynsim_edo_B.R_d[dynsim_edo_B.ret] = -dynsim_edo_B.R_j[dynsim_edo_B.ret];
  }

  for (dynsim_edo_B.ret = 0; dynsim_edo_B.ret < 3; dynsim_edo_B.ret++) {
    dynsim_edo_B.loop_ub = dynsim_edo_B.ret << 2;
    dynsim_edo_B.R_c[dynsim_edo_B.loop_ub] = dynsim_edo_B.R_j[3 *
      dynsim_edo_B.ret];
    dynsim_edo_B.R_c[dynsim_edo_B.loop_ub + 1] = dynsim_edo_B.R_j[3 *
      dynsim_edo_B.ret + 1];
    dynsim_edo_B.R_c[dynsim_edo_B.loop_ub + 2] = dynsim_edo_B.R_j[3 *
      dynsim_edo_B.ret + 2];
    dynsim_edo_B.R_c[dynsim_edo_B.ret + 12] = dynsim_edo_B.R_d[dynsim_edo_B.ret
      + 6] * static_cast<real_T>(dynsim_edo_B.T2_i[14]) +
      (dynsim_edo_B.R_d[dynsim_edo_B.ret + 3] * static_cast<real_T>
       (dynsim_edo_B.T2_i[13]) + dynsim_edo_B.R_d[dynsim_edo_B.ret] *
       static_cast<real_T>(dynsim_edo_B.T2_i[12]));
  }

  dynsim_edo_B.R_c[3] = 0.0;
  dynsim_edo_B.R_c[7] = 0.0;
  dynsim_edo_B.R_c[11] = 0.0;
  dynsim_edo_B.R_c[15] = 1.0;
  for (dynsim_edo_B.ret = 0; dynsim_edo_B.ret < 4; dynsim_edo_B.ret++) {
    for (dynsim_edo_B.n_p = 0; dynsim_edo_B.n_p < 4; dynsim_edo_B.n_p++) {
      dynsim_edo_B.loop_ub = dynsim_edo_B.ret << 2;
      dynsim_edo_B.rtb_MATLABSystem_tmp = dynsim_edo_B.n_p +
        dynsim_edo_B.loop_ub;
      dynsim_edo_B.MATLABSystem_k[dynsim_edo_B.rtb_MATLABSystem_tmp] = 0.0;
      dynsim_edo_B.MATLABSystem_k[dynsim_edo_B.rtb_MATLABSystem_tmp] +=
        dynsim_edo_B.T1[dynsim_edo_B.loop_ub] *
        dynsim_edo_B.R_c[dynsim_edo_B.n_p];
      dynsim_edo_B.MATLABSystem_k[dynsim_edo_B.rtb_MATLABSystem_tmp] +=
        dynsim_edo_B.T1[dynsim_edo_B.loop_ub + 1] *
        dynsim_edo_B.R_c[dynsim_edo_B.n_p + 4];
      dynsim_edo_B.MATLABSystem_k[dynsim_edo_B.rtb_MATLABSystem_tmp] +=
        dynsim_edo_B.T1[dynsim_edo_B.loop_ub + 2] *
        dynsim_edo_B.R_c[dynsim_edo_B.n_p + 8];
      dynsim_edo_B.MATLABSystem_k[dynsim_edo_B.rtb_MATLABSystem_tmp] +=
        dynsim_edo_B.T1[dynsim_edo_B.loop_ub + 3] *
        dynsim_edo_B.R_c[dynsim_edo_B.n_p + 12];
    }
  }

  // MATLABSystem: '<S16>/Get Parameter2'
  ParamGet_dynsim_edo_117.get_parameter(&dynsim_edo_B.bid1);

  // MATLABSystem: '<S16>/Get Parameter3'
  ParamGet_dynsim_edo_118.get_parameter(&dynsim_edo_B.k);

  // MATLABSystem: '<S16>/Get Parameter8'
  ParamGet_dynsim_edo_133.get_parameter(&dynsim_edo_B.j);

  // Integrator: '<S13>/Velocity' incorporates:
  //   MATLABSystem: '<S16>/Get Parameter2'
  //   MATLABSystem: '<S16>/Get Parameter3'
  //   MATLABSystem: '<S16>/Get Parameter8'

  if (dynsim_edo_DW.Velocity_IWORK != 0) {
    dynsim_edo_X.Velocity_CSTATE[0] = dynsim_edo_B.bid1;
    dynsim_edo_X.Velocity_CSTATE[1] = dynsim_edo_B.k;
    dynsim_edo_X.Velocity_CSTATE[2] = dynsim_edo_B.j;
  }

  dynsim_edo_B.Velocity[0] = dynsim_edo_X.Velocity_CSTATE[0];
  dynsim_edo_B.Velocity[1] = dynsim_edo_X.Velocity_CSTATE[1];
  dynsim_edo_B.Velocity[2] = dynsim_edo_X.Velocity_CSTATE[2];

  // End of Integrator: '<S13>/Velocity'

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   MATLABSystem: '<S4>/MATLAB System'

  for (dynsim_edo_B.ret = 0; dynsim_edo_B.ret < 4; dynsim_edo_B.ret++) {
    dynsim_edo_B.bid1 = b->data[dynsim_edo_B.ret + 2] * dynsim_edo_B.Velocity[0];
    dynsim_edo_B.bid1 += b->data[dynsim_edo_B.ret + 8] * dynsim_edo_B.Velocity[1];
    dynsim_edo_B.bid1 += b->data[dynsim_edo_B.ret + 14] * dynsim_edo_B.Velocity
      [2];
    dynsim_edo_B.cartVel[dynsim_edo_B.ret] = dynsim_edo_B.bid1;
  }

  dynsim_edo_emxFree_real_T(&b);
  if (rtmIsMajorTimeStep(dynsim_edo_M)) {
    // Outputs for Atomic SubSystem: '<Root>/Subscribe'
    // MATLABSystem: '<S12>/SourceBlock' incorporates:
    //   Inport: '<S14>/In1'

    dynsim_edo_SystemCore_step(&dynsim_edo_B.b_bool,
      dynsim_edo_B.b_varargout_2_Data,
      &dynsim_edo_B.b_varargout_2_Data_SL_Info_Curr,
      &dynsim_edo_B.b_varargout_2_Data_SL_Info_Rece,
      &dynsim_edo_B.b_varargout_2_Layout_DataOffset,
      dynsim_edo_B.b_varargout_2_Layout_Dim,
      &dynsim_edo_B.b_varargout_2_Layout_Dim_SL_Inf,
      &dynsim_edo_B.b_varargout_2_Layout_Dim_SL_I_f);

    // Outputs for Enabled SubSystem: '<S12>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S14>/Enable'

    if (dynsim_edo_B.b_bool) {
      memcpy(&dynsim_edo_B.In1.Data[0], &dynsim_edo_B.b_varargout_2_Data[0],
             sizeof(real_T) << 7U);
      dynsim_edo_B.In1.Data_SL_Info.CurrentLength =
        dynsim_edo_B.b_varargout_2_Data_SL_Info_Curr;
      dynsim_edo_B.In1.Data_SL_Info.ReceivedLength =
        dynsim_edo_B.b_varargout_2_Data_SL_Info_Rece;
      dynsim_edo_B.In1.Layout.DataOffset =
        dynsim_edo_B.b_varargout_2_Layout_DataOffset;
      memcpy(&dynsim_edo_B.In1.Layout.Dim[0],
             &dynsim_edo_B.b_varargout_2_Layout_Dim[0], sizeof
             (SL_Bus_dynsim_edo_std_msgs_MultiArrayDimension) << 4U);
      dynsim_edo_B.In1.Layout.Dim_SL_Info.CurrentLength =
        dynsim_edo_B.b_varargout_2_Layout_Dim_SL_Inf;
      dynsim_edo_B.In1.Layout.Dim_SL_Info.ReceivedLength =
        dynsim_edo_B.b_varargout_2_Layout_Dim_SL_I_f;
    }

    // End of MATLABSystem: '<S12>/SourceBlock'
    // End of Outputs for SubSystem: '<S12>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe'
  }

  dynsim_edo_emxInit_real_T(&L, 2);
  dynsim_edo_emxInit_real_T(&lambda, 2);
  dynsim_edo_emxInit_real_T(&tmp, 1);

  // MATLABSystem: '<S15>/MATLAB System' incorporates:
  //   Constant: '<S13>/Constant'
  //   Integrator: '<S13>/Position'

  obj_2 = &dynsim_edo_DW.obj;
  RigidBodyTreeDynamics_massMatri(&dynsim_edo_DW.obj.TreeInternal,
    dynsim_edo_X.Position_CSTATE, L, lambda);
  dynsim_edo_B.bid1 = obj_2->TreeInternal.VelocityNumber;
  dynsim_edo_B.rtb_MATLABSystem_tmp = static_cast<int32_T>(dynsim_edo_B.bid1);
  dynsim_edo_B.ret = tmp->size[0];
  tmp->size[0] = dynsim_edo_B.rtb_MATLABSystem_tmp;
  dynsim_emxEnsureCapacity_real_T(tmp, dynsim_edo_B.ret);
  for (dynsim_edo_B.ret = 0; dynsim_edo_B.ret <
       dynsim_edo_B.rtb_MATLABSystem_tmp; dynsim_edo_B.ret++) {
    tmp->data[dynsim_edo_B.ret] = 0.0;
  }

  RigidBodyTreeDynamics_inverseDy(&obj_2->TreeInternal,
    dynsim_edo_X.Position_CSTATE, dynsim_edo_B.Velocity,
    dynsim_edo_P.Constant_Value_f, dynsim_edo_B.MATLABSystem);
  dynsim_edo_emxFree_real_T(&tmp);

  // MATLABSystem: '<S15>/MATLAB System'
  dynsim_edo_B.MATLABSystem[0] = dynsim_edo_B.In1.Data[1] -
    dynsim_edo_B.MATLABSystem[0];
  dynsim_edo_B.MATLABSystem[1] = dynsim_edo_B.In1.Data[2] -
    dynsim_edo_B.MATLABSystem[1];
  dynsim_edo_B.MATLABSystem[2] = dynsim_edo_B.In1.Data[3] -
    dynsim_edo_B.MATLABSystem[2];
  if ((L->size[0] == 0) || (L->size[1] == 0)) {
    dynsim_edo_B.u1 = 0;
  } else {
    dynsim_edo_B.loop_ub = L->size[0];
    dynsim_edo_B.u1 = L->size[1];
    if (dynsim_edo_B.loop_ub > dynsim_edo_B.u1) {
      dynsim_edo_B.u1 = dynsim_edo_B.loop_ub;
    }
  }

  dynsim_edo_emxInit_real_T(&H, 2);

  // MATLABSystem: '<S15>/MATLAB System'
  dynsim_edo_B.ret = H->size[0] * H->size[1];
  H->size[0] = L->size[0];
  H->size[1] = L->size[1];
  dynsim_emxEnsureCapacity_real_T(H, dynsim_edo_B.ret);
  dynsim_edo_B.loop_ub = L->size[0] * L->size[1] - 1;
  for (dynsim_edo_B.ret = 0; dynsim_edo_B.ret <= dynsim_edo_B.loop_ub;
       dynsim_edo_B.ret++) {
    H->data[dynsim_edo_B.ret] = L->data[dynsim_edo_B.ret];
  }

  dynsim_edo_B.iend = static_cast<int32_T>(((-1.0 - static_cast<real_T>
    (dynsim_edo_B.u1)) + 1.0) / -1.0) - 1;
  for (dynsim_edo_B.loop_ub = 0; dynsim_edo_B.loop_ub <= dynsim_edo_B.iend;
       dynsim_edo_B.loop_ub++) {
    dynsim_edo_B.j = static_cast<real_T>(dynsim_edo_B.u1) + -static_cast<real_T>
      (dynsim_edo_B.loop_ub);
    dynsim_edo_B.ret = static_cast<int32_T>(dynsim_edo_B.j);
    dynsim_edo_B.n_p = dynsim_edo_B.ret - 1;
    H->data[(static_cast<int32_T>(dynsim_edo_B.j) + H->size[0] *
             (static_cast<int32_T>(dynsim_edo_B.j) - 1)) - 1] = sqrt(H->data
      [(dynsim_edo_B.n_p * H->size[0] + dynsim_edo_B.ret) - 1]);
    dynsim_edo_B.k = lambda->data[dynsim_edo_B.n_p];
    while (dynsim_edo_B.k > 0.0) {
      dynsim_edo_B.i = static_cast<int32_T>(dynsim_edo_B.k) - 1;
      H->data[(static_cast<int32_T>(dynsim_edo_B.j) + H->size[0] * (static_cast<
                int32_T>(dynsim_edo_B.k) - 1)) - 1] = H->data[(dynsim_edo_B.i *
        H->size[0] + dynsim_edo_B.ret) - 1] / H->data[((static_cast<int32_T>
        (dynsim_edo_B.j) - 1) * H->size[0] + static_cast<int32_T>(dynsim_edo_B.j))
        - 1];
      dynsim_edo_B.k = lambda->data[dynsim_edo_B.i];
    }

    dynsim_edo_B.k = lambda->data[dynsim_edo_B.n_p];
    while (dynsim_edo_B.k > 0.0) {
      dynsim_edo_B.j = dynsim_edo_B.k;
      while (dynsim_edo_B.j > 0.0) {
        dynsim_edo_B.n_p = static_cast<int32_T>(dynsim_edo_B.j) - 1;
        H->data[(static_cast<int32_T>(dynsim_edo_B.k) + H->size[0] * (
                  static_cast<int32_T>(dynsim_edo_B.j) - 1)) - 1] = H->data
          [(dynsim_edo_B.n_p * H->size[0] + static_cast<int32_T>(dynsim_edo_B.k))
          - 1] - H->data[((static_cast<int32_T>(dynsim_edo_B.k) - 1) * H->size[0]
                          + dynsim_edo_B.ret) - 1] * H->data
          [((static_cast<int32_T>(dynsim_edo_B.j) - 1) * H->size[0] +
            dynsim_edo_B.ret) - 1];
        dynsim_edo_B.j = lambda->data[dynsim_edo_B.n_p];
      }

      dynsim_edo_B.k = lambda->data[static_cast<int32_T>(dynsim_edo_B.k) - 1];
    }
  }

  dynsim_edo_B.ret = L->size[0] * L->size[1];
  L->size[0] = H->size[0];
  L->size[1] = H->size[1];
  dynsim_emxEnsureCapacity_real_T(L, dynsim_edo_B.ret);
  dynsim_edo_B.loop_ub = H->size[0] * H->size[1] - 1;
  for (dynsim_edo_B.ret = 0; dynsim_edo_B.ret <= dynsim_edo_B.loop_ub;
       dynsim_edo_B.ret++) {
    L->data[dynsim_edo_B.ret] = H->data[dynsim_edo_B.ret];
  }

  dynsim_edo_B.n_p = H->size[1];
  if ((H->size[0] == 0) || (H->size[1] == 0) || (1 >= H->size[1])) {
  } else {
    dynsim_edo_B.iend = 0;
    for (dynsim_edo_B.ret = 2; dynsim_edo_B.ret <= dynsim_edo_B.n_p;
         dynsim_edo_B.ret++) {
      for (dynsim_edo_B.loop_ub = 0; dynsim_edo_B.loop_ub <= dynsim_edo_B.iend;
           dynsim_edo_B.loop_ub++) {
        L->data[dynsim_edo_B.loop_ub + L->size[0] * (dynsim_edo_B.ret - 1)] =
          0.0;
      }

      if (dynsim_edo_B.iend + 1 < H->size[0]) {
        dynsim_edo_B.iend++;
      }
    }
  }

  dynsim_edo_emxFree_real_T(&H);

  // MATLABSystem: '<S15>/MATLAB System'
  dynsim_edo_B.iend = static_cast<int32_T>(((-1.0 - dynsim_edo_B.bid1) + 1.0) /
    -1.0) - 1;
  for (dynsim_edo_B.loop_ub = 0; dynsim_edo_B.loop_ub <= dynsim_edo_B.iend;
       dynsim_edo_B.loop_ub++) {
    dynsim_edo_B.n_p = static_cast<int32_T>(dynsim_edo_B.bid1 +
      -static_cast<real_T>(dynsim_edo_B.loop_ub));
    dynsim_edo_B.ret = dynsim_edo_B.n_p - 1;
    dynsim_edo_B.MATLABSystem[dynsim_edo_B.ret] /= L->data[(dynsim_edo_B.ret *
      L->size[0] + dynsim_edo_B.n_p) - 1];
    dynsim_edo_B.j = lambda->data[dynsim_edo_B.ret];
    while (dynsim_edo_B.j > 0.0) {
      dynsim_edo_B.u1 = static_cast<int32_T>(dynsim_edo_B.j) - 1;
      dynsim_edo_B.MATLABSystem[dynsim_edo_B.u1] -= L->data[(dynsim_edo_B.u1 *
        L->size[0] + dynsim_edo_B.n_p) - 1] *
        dynsim_edo_B.MATLABSystem[dynsim_edo_B.ret];
      dynsim_edo_B.j = lambda->data[dynsim_edo_B.u1];
    }
  }

  dynsim_edo_B.n_p = dynsim_edo_B.rtb_MATLABSystem_tmp - 1;
  for (dynsim_edo_B.loop_ub = 0; dynsim_edo_B.loop_ub <= dynsim_edo_B.n_p;
       dynsim_edo_B.loop_ub++) {
    dynsim_edo_B.j = lambda->data[dynsim_edo_B.loop_ub];
    while (dynsim_edo_B.j > 0.0) {
      dynsim_edo_B.ret = static_cast<int32_T>(dynsim_edo_B.j) - 1;
      dynsim_edo_B.MATLABSystem[dynsim_edo_B.loop_ub] -= L->
        data[dynsim_edo_B.ret * L->size[0] + dynsim_edo_B.loop_ub] *
        dynsim_edo_B.MATLABSystem[dynsim_edo_B.ret];
      dynsim_edo_B.j = lambda->data[dynsim_edo_B.ret];
    }

    dynsim_edo_B.MATLABSystem[dynsim_edo_B.loop_ub] /= L->data[L->size[0] *
      dynsim_edo_B.loop_ub + dynsim_edo_B.loop_ub];
  }

  dynsim_edo_emxFree_real_T(&lambda);
  dynsim_edo_emxFree_real_T(&L);

  // Clock: '<Root>/Clock1' incorporates:
  //   Clock: '<Root>/Clock'

  dynsim_edo_B.bid1 = dynsim_edo_M->Timing.t[0];

  // MATLAB Function: '<Root>/Assign to CartesianState msg' incorporates:
  //   Clock: '<Root>/Clock1'
  //   Constant: '<S6>/Constant'
  //   MATLAB Function: '<Root>/MATLAB Function'

  dynsim_edo_B.msg_g = dynsim_edo_P.Constant_Value;
  if (dynsim_edo_B.bid1 < 0.0) {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    dynsim_edo_B.k = ceil(dynsim_edo_B.bid1);
  } else {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    dynsim_edo_B.k = floor(dynsim_edo_B.bid1);
  }

  dynsim_edo_B.msg_g.Header.Stamp.Sec = dynsim_edo_B.k;
  dynsim_edo_B.j = (dynsim_edo_B.bid1 - dynsim_edo_B.k) * 1.0E+9;
  if (dynsim_edo_B.j < 0.0) {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    dynsim_edo_B.j = ceil(dynsim_edo_B.j);
  } else {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    dynsim_edo_B.j = floor(dynsim_edo_B.j);
  }

  dynsim_edo_B.msg_g.Header.Stamp.Nsec = dynsim_edo_B.j;
  dynsim_edo_B.msg_g.Name_SL_Info.CurrentLength = 3U;
  dynsim_edo_B.msg_g.Position_SL_Info.CurrentLength = 3U;
  dynsim_edo_B.msg_g.Velocity_SL_Info.CurrentLength = 3U;
  dynsim_edo_B.msg_g.Name[0].Data_SL_Info.CurrentLength = 7U;
  dynsim_edo_B.msg_g.Position[0] = dynsim_edo_B.MATLABSystem_k[12];
  dynsim_edo_B.msg_g.Velocity[0] = dynsim_edo_B.cartVel[0];
  dynsim_edo_B.msg_g.Name[1].Data_SL_Info.CurrentLength = 7U;
  dynsim_edo_B.msg_g.Position[1] = dynsim_edo_B.MATLABSystem_k[13];
  dynsim_edo_B.msg_g.Velocity[1] = dynsim_edo_B.cartVel[1];
  for (dynsim_edo_B.ret = 0; dynsim_edo_B.ret < 7; dynsim_edo_B.ret++) {
    dynsim_edo_B.b_o4.f1[dynsim_edo_B.ret] = e[dynsim_edo_B.ret];
    dynsim_edo_B.c.f1[dynsim_edo_B.ret] = f[dynsim_edo_B.ret];
    dynsim_edo_B.d.f1[dynsim_edo_B.ret] = g[dynsim_edo_B.ret];
    dynsim_edo_B.msg_g.Name[0].Data[dynsim_edo_B.ret] = static_cast<uint8_T>
      (dynsim_edo_B.b_o4.f1[dynsim_edo_B.ret]);
    dynsim_edo_B.msg_g.Name[1].Data[dynsim_edo_B.ret] = static_cast<uint8_T>
      (dynsim_edo_B.c.f1[dynsim_edo_B.ret]);
    dynsim_edo_B.msg_g.Name[2].Data[dynsim_edo_B.ret] = static_cast<uint8_T>
      (dynsim_edo_B.d.f1[dynsim_edo_B.ret]);
  }

  dynsim_edo_B.msg_g.Name[2].Data_SL_Info.CurrentLength = 7U;
  dynsim_edo_B.msg_g.Position[2] = dynsim_edo_B.MATLABSystem_k[14];
  dynsim_edo_B.msg_g.Velocity[2] = dynsim_edo_B.cartVel[2];

  // End of MATLAB Function: '<Root>/Assign to CartesianState msg'

  // Outputs for Atomic SubSystem: '<Root>/Publish2'
  // MATLABSystem: '<S11>/SinkBlock'
  Pub_dynsim_edo_124.publish(&dynsim_edo_B.msg_g);

  // End of Outputs for SubSystem: '<Root>/Publish2'

  // MATLAB Function: '<Root>/Assign to JointState msg' incorporates:
  //   Constant: '<S6>/Constant'
  //   Integrator: '<S13>/Position'

  dynsim_edo_B.msg_g = dynsim_edo_P.Constant_Value;
  dynsim_edo_B.msg_g.Header.Stamp.Sec = dynsim_edo_B.k;
  dynsim_edo_B.msg_g.Header.Stamp.Nsec = dynsim_edo_B.j;
  dynsim_edo_B.msg_g.Name_SL_Info.CurrentLength = 3U;
  dynsim_edo_B.msg_g.Position_SL_Info.CurrentLength = 3U;
  dynsim_edo_B.msg_g.Velocity_SL_Info.CurrentLength = 3U;
  dynsim_edo_B.msg_g.Name[0].Data_SL_Info.CurrentLength = 7U;
  dynsim_edo_B.msg_g.Position[0] = dynsim_edo_X.Position_CSTATE[0];
  dynsim_edo_B.msg_g.Velocity[0] = dynsim_edo_B.Velocity[0];
  dynsim_edo_B.msg_g.Name[1].Data_SL_Info.CurrentLength = 7U;
  dynsim_edo_B.msg_g.Position[1] = dynsim_edo_X.Position_CSTATE[1];
  dynsim_edo_B.msg_g.Velocity[1] = dynsim_edo_B.Velocity[1];
  for (dynsim_edo_B.ret = 0; dynsim_edo_B.ret < 7; dynsim_edo_B.ret++) {
    dynsim_edo_B.b_o4.f1[dynsim_edo_B.ret] = e_0[dynsim_edo_B.ret];
    dynsim_edo_B.c.f1[dynsim_edo_B.ret] = f_0[dynsim_edo_B.ret];
    dynsim_edo_B.d.f1[dynsim_edo_B.ret] = g_0[dynsim_edo_B.ret];
    dynsim_edo_B.msg_g.Name[0].Data[dynsim_edo_B.ret] = static_cast<uint8_T>
      (dynsim_edo_B.b_o4.f1[dynsim_edo_B.ret]);
    dynsim_edo_B.msg_g.Name[1].Data[dynsim_edo_B.ret] = static_cast<uint8_T>
      (dynsim_edo_B.c.f1[dynsim_edo_B.ret]);
    dynsim_edo_B.msg_g.Name[2].Data[dynsim_edo_B.ret] = static_cast<uint8_T>
      (dynsim_edo_B.d.f1[dynsim_edo_B.ret]);
  }

  dynsim_edo_B.msg_g.Name[2].Data_SL_Info.CurrentLength = 7U;
  dynsim_edo_B.msg_g.Position[2] = dynsim_edo_X.Position_CSTATE[2];
  dynsim_edo_B.msg_g.Velocity[2] = dynsim_edo_B.Velocity[2];

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S9>/SinkBlock'
  Pub_dynsim_edo_22.publish(&dynsim_edo_B.msg_g);

  // End of Outputs for SubSystem: '<Root>/Publish'

  // MATLAB Function: '<Root>/Assign to Time msg'
  if (dynsim_edo_B.bid1 < 0.0) {
    dynsim_edo_B.k = ceil(dynsim_edo_B.bid1);
  } else {
    dynsim_edo_B.k = floor(dynsim_edo_B.bid1);
  }

  dynsim_edo_B.msg_l.Clock_.Sec = dynsim_edo_B.k;
  dynsim_edo_B.j = (dynsim_edo_B.bid1 - dynsim_edo_B.k) * 1.0E+9;
  if (dynsim_edo_B.j < 0.0) {
    dynsim_edo_B.msg_l.Clock_.Nsec = ceil(dynsim_edo_B.j);
  } else {
    dynsim_edo_B.msg_l.Clock_.Nsec = floor(dynsim_edo_B.j);
  }

  // End of MATLAB Function: '<Root>/Assign to Time msg'

  // Outputs for Atomic SubSystem: '<Root>/Publish1'
  // MATLABSystem: '<S10>/SinkBlock'
  Pub_dynsim_edo_50.publish(&dynsim_edo_B.msg_l);

  // End of Outputs for SubSystem: '<Root>/Publish1'
  if (rtmIsMajorTimeStep(dynsim_edo_M)) {
    // Update for Integrator: '<S13>/Position'
    dynsim_edo_DW.Position_IWORK = 0;

    // Update for Integrator: '<S13>/Velocity'
    dynsim_edo_DW.Velocity_IWORK = 0;
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep(dynsim_edo_M)) {
    rt_ertODEUpdateContinuousStates(&dynsim_edo_M->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++dynsim_edo_M->Timing.clockTick0;
    dynsim_edo_M->Timing.t[0] = rtsiGetSolverStopTime(&dynsim_edo_M->solverInfo);

    {
      // Update absolute timer for sample time: [0.001s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.001, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      dynsim_edo_M->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void dynsim_edo_derivatives(void)
{
  XDot_dynsim_edo_T *_rtXdot;
  _rtXdot = ((XDot_dynsim_edo_T *) dynsim_edo_M->derivs);

  // Derivatives for Integrator: '<S13>/Position'
  _rtXdot->Position_CSTATE[0] = dynsim_edo_B.Velocity[0];

  // Derivatives for Integrator: '<S13>/Velocity'
  _rtXdot->Velocity_CSTATE[0] = dynsim_edo_B.MATLABSystem[0];

  // Derivatives for Integrator: '<S13>/Position'
  _rtXdot->Position_CSTATE[1] = dynsim_edo_B.Velocity[1];

  // Derivatives for Integrator: '<S13>/Velocity'
  _rtXdot->Velocity_CSTATE[1] = dynsim_edo_B.MATLABSystem[1];

  // Derivatives for Integrator: '<S13>/Position'
  _rtXdot->Position_CSTATE[2] = dynsim_edo_B.Velocity[2];

  // Derivatives for Integrator: '<S13>/Velocity'
  _rtXdot->Velocity_CSTATE[2] = dynsim_edo_B.MATLABSystem[2];
}

// Model initialize function
void dynsim_edo_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&dynsim_edo_M->solverInfo,
                          &dynsim_edo_M->Timing.simTimeStep);
    rtsiSetTPtr(&dynsim_edo_M->solverInfo, &rtmGetTPtr(dynsim_edo_M));
    rtsiSetStepSizePtr(&dynsim_edo_M->solverInfo,
                       &dynsim_edo_M->Timing.stepSize0);
    rtsiSetdXPtr(&dynsim_edo_M->solverInfo, &dynsim_edo_M->derivs);
    rtsiSetContStatesPtr(&dynsim_edo_M->solverInfo, (real_T **)
                         &dynsim_edo_M->contStates);
    rtsiSetNumContStatesPtr(&dynsim_edo_M->solverInfo,
      &dynsim_edo_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&dynsim_edo_M->solverInfo,
      &dynsim_edo_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&dynsim_edo_M->solverInfo,
      &dynsim_edo_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&dynsim_edo_M->solverInfo,
      &dynsim_edo_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&dynsim_edo_M->solverInfo, (&rtmGetErrorStatus
      (dynsim_edo_M)));
    rtsiSetRTModelPtr(&dynsim_edo_M->solverInfo, dynsim_edo_M);
  }

  rtsiSetSimTimeStep(&dynsim_edo_M->solverInfo, MAJOR_TIME_STEP);
  dynsim_edo_M->intgData.y = dynsim_edo_M->odeY;
  dynsim_edo_M->intgData.f[0] = dynsim_edo_M->odeF[0];
  dynsim_edo_M->intgData.f[1] = dynsim_edo_M->odeF[1];
  dynsim_edo_M->intgData.f[2] = dynsim_edo_M->odeF[2];
  dynsim_edo_M->contStates = ((X_dynsim_edo_T *) &dynsim_edo_X);
  rtsiSetSolverData(&dynsim_edo_M->solverInfo, static_cast<void *>
                    (&dynsim_edo_M->intgData));
  rtsiSetSolverName(&dynsim_edo_M->solverInfo,"ode3");
  rtmSetTPtr(dynsim_edo_M, &dynsim_edo_M->Timing.tArray[0]);
  dynsim_edo_M->Timing.stepSize0 = 0.001;
  rtmSetFirstInitCond(dynsim_edo_M, 1);

  {
    char_T tmp[7];
    int32_T i;
    static const char_T tmp_0[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 't',
      'o', 'r', 'q', 'u', 'e' };

    static const char_T tmp_1[17] = { '/', 'c', 'a', 'r', 't', 'e', 's', 'i',
      'a', 'n', '_', 's', 't', 'a', 't', 'e', 's' };

    static const char_T tmp_2[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 's',
      't', 'a', 't', 'e', 's' };

    static const char_T tmp_3[6] = { '/', 'c', 'l', 'o', 'c', 'k' };

    static const char_T tmp_4[22] = { '/', 'd', 'y', 'n', 's', 'i', 'm', '_',
      'e', 'd', 'o', '/', 'q', '1', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_5[22] = { '/', 'd', 'y', 'n', 's', 'i', 'm', '_',
      'e', 'd', 'o', '/', 'q', '2', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_6[22] = { '/', 'd', 'y', 'n', 's', 'i', 'm', '_',
      'e', 'd', 'o', '/', 'q', '3', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_7[23] = { '/', 'd', 'y', 'n', 's', 'i', 'm', '_',
      'e', 'd', 'o', '/', 'q', 'v', '1', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l'
    };

    static const char_T tmp_8[24] = { '/', 'd', 'y', 'n', 'n', 's', 'i', 'm',
      '_', 'e', 'd', 'o', '/', 'q', 'v', '2', '_', 'i', 'n', 'i', 't', 'i', 'a',
      'l' };

    static const char_T tmp_9[24] = { '/', 'd', 'y', 'n', 'n', 's', 'i', 'm',
      '_', 'e', 'd', 'o', '/', 'q', 'v', '3', '_', 'i', 'n', 'i', 't', 'i', 'a',
      'l' };

    // InitializeConditions for Integrator: '<S13>/Position' incorporates:
    //   Integrator: '<S13>/Velocity'

    if (rtmIsFirstInitCond(dynsim_edo_M)) {
      dynsim_edo_X.Position_CSTATE[0] = 0.0;
      dynsim_edo_X.Position_CSTATE[1] = 0.0;
      dynsim_edo_X.Position_CSTATE[2] = 0.0;
      dynsim_edo_X.Velocity_CSTATE[0] = 0.0;
      dynsim_edo_X.Velocity_CSTATE[1] = 0.0;
      dynsim_edo_X.Velocity_CSTATE[2] = 0.0;
    }

    dynsim_edo_DW.Position_IWORK = 1;

    // End of InitializeConditions for Integrator: '<S13>/Position'

    // InitializeConditions for Integrator: '<S13>/Velocity'
    dynsim_edo_DW.Velocity_IWORK = 1;

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S12>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S14>/Out1'
    dynsim_edo_B.In1 = dynsim_edo_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S12>/Enabled Subsystem'

    // Start for MATLABSystem: '<S12>/SourceBlock'
    dynsim_edo_DW.obj_pp.matlabCodegenIsDeleted = false;
    dynsim_edo_DW.obj_pp.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      dynsim_edo_B.cv4[i] = tmp_0[i];
    }

    dynsim_edo_B.cv4[13] = '\x00';
    Sub_dynsim_edo_16.createSubscriber(dynsim_edo_B.cv4, 1);
    dynsim_edo_DW.obj_pp.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish2'
    // Start for MATLABSystem: '<S11>/SinkBlock'
    dynsim_edo_DW.obj_ar.matlabCodegenIsDeleted = false;
    dynsim_edo_DW.obj_ar.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      dynsim_edo_B.cv3[i] = tmp_1[i];
    }

    dynsim_edo_B.cv3[17] = '\x00';
    Pub_dynsim_edo_124.createPublisher(dynsim_edo_B.cv3, 1);
    dynsim_edo_DW.obj_ar.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish2'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S9>/SinkBlock'
    dynsim_edo_DW.obj_nr.matlabCodegenIsDeleted = false;
    dynsim_edo_DW.obj_nr.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      dynsim_edo_B.cv4[i] = tmp_2[i];
    }

    dynsim_edo_B.cv4[13] = '\x00';
    Pub_dynsim_edo_22.createPublisher(dynsim_edo_B.cv4, 1);
    dynsim_edo_DW.obj_nr.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S10>/SinkBlock'
    dynsim_edo_DW.obj_f.matlabCodegenIsDeleted = false;
    dynsim_edo_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 6; i++) {
      tmp[i] = tmp_3[i];
    }

    tmp[6] = '\x00';
    Pub_dynsim_edo_50.createPublisher(tmp, 1);
    dynsim_edo_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish1'

    // Start for MATLABSystem: '<S16>/Get Parameter'
    dynsim_edo_DW.obj_p.matlabCodegenIsDeleted = false;
    dynsim_edo_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      dynsim_edo_B.cv2[i] = tmp_4[i];
    }

    dynsim_edo_B.cv2[22] = '\x00';
    ParamGet_dynsim_edo_112.initialize(dynsim_edo_B.cv2);
    ParamGet_dynsim_edo_112.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_edo_112.set_initial_value(0.78539816339744828);
    dynsim_edo_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S16>/Get Parameter'

    // Start for MATLABSystem: '<S16>/Get Parameter1'
    dynsim_edo_DW.obj_n.matlabCodegenIsDeleted = false;
    dynsim_edo_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      dynsim_edo_B.cv2[i] = tmp_5[i];
    }

    dynsim_edo_B.cv2[22] = '\x00';
    ParamGet_dynsim_edo_113.initialize(dynsim_edo_B.cv2);
    ParamGet_dynsim_edo_113.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_edo_113.set_initial_value(-1.5707963267948966);
    dynsim_edo_DW.obj_n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S16>/Get Parameter1'

    // Start for MATLABSystem: '<S16>/Get Parameter4'
    dynsim_edo_DW.obj_nf.matlabCodegenIsDeleted = false;
    dynsim_edo_DW.obj_nf.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      dynsim_edo_B.cv2[i] = tmp_6[i];
    }

    dynsim_edo_B.cv2[22] = '\x00';
    ParamGet_dynsim_edo_125.initialize(dynsim_edo_B.cv2);
    ParamGet_dynsim_edo_125.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_edo_125.set_initial_value(-1.5707963267948966);
    dynsim_edo_DW.obj_nf.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S16>/Get Parameter4'
    emxInitStruct_robotics_slmanip_(&dynsim_edo_DW.obj_a);
    emxInitStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_1_p);
    emxInitStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_12_m);
    emxInitStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_11_a);
    emxInitStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_10_h);
    emxInitStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_9_d);
    emxInitStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_8_l);
    emxInitStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_7_p);
    emxInitStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_6_n);
    emxInitStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_5_l);
    emxInitStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_4_b);
    emxInitStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_3_g);
    emxInitStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_2_a);

    // Start for MATLABSystem: '<S4>/MATLAB System'
    dynsim_edo_DW.obj_a.isInitialized = 0;
    dynsim_edo_DW.obj_a.isInitialized = 1;
    dyn_RigidBodyTree_RigidBodyTree(&dynsim_edo_DW.obj_a.TreeInternal,
      &dynsim_edo_DW.gobj_2_a, &dynsim_edo_DW.gobj_4_b, &dynsim_edo_DW.gobj_5_l,
      &dynsim_edo_DW.gobj_6_n, &dynsim_edo_DW.gobj_7_p, &dynsim_edo_DW.gobj_3_g);
    emxInitStruct_robotics_slmani_h(&dynsim_edo_DW.obj_o);
    emxInitStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_1_pg);
    emxInitStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_12_c);
    emxInitStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_11_d);
    emxInitStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_10_k);
    emxInitStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_9_b);
    emxInitStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_8_o);
    emxInitStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_7_h);
    emxInitStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_6_o);
    emxInitStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_5_o);
    emxInitStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_4_c);
    emxInitStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_3_m);
    emxInitStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_2_j);

    // Start for MATLABSystem: '<S5>/MATLAB System'
    dynsim_edo_DW.obj_o.isInitialized = 0;
    dynsim_edo_DW.obj_o.isInitialized = 1;
    d_RigidBodyTree_RigidBodyTree_h(&dynsim_edo_DW.obj_o.TreeInternal,
      &dynsim_edo_DW.gobj_2_j, &dynsim_edo_DW.gobj_4_c, &dynsim_edo_DW.gobj_5_o,
      &dynsim_edo_DW.gobj_6_o, &dynsim_edo_DW.gobj_7_h, &dynsim_edo_DW.gobj_3_m);

    // Start for MATLABSystem: '<S16>/Get Parameter2'
    dynsim_edo_DW.obj_h.matlabCodegenIsDeleted = false;
    dynsim_edo_DW.obj_h.isInitialized = 1;
    for (i = 0; i < 23; i++) {
      dynsim_edo_B.cv1[i] = tmp_7[i];
    }

    dynsim_edo_B.cv1[23] = '\x00';
    ParamGet_dynsim_edo_117.initialize(dynsim_edo_B.cv1);
    ParamGet_dynsim_edo_117.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_edo_117.set_initial_value(0.0);
    dynsim_edo_DW.obj_h.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S16>/Get Parameter2'

    // Start for MATLABSystem: '<S16>/Get Parameter3'
    dynsim_edo_DW.obj_b.matlabCodegenIsDeleted = false;
    dynsim_edo_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      dynsim_edo_B.cv[i] = tmp_8[i];
    }

    dynsim_edo_B.cv[24] = '\x00';
    ParamGet_dynsim_edo_118.initialize(dynsim_edo_B.cv);
    ParamGet_dynsim_edo_118.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_edo_118.set_initial_value(0.0);
    dynsim_edo_DW.obj_b.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S16>/Get Parameter3'

    // Start for MATLABSystem: '<S16>/Get Parameter8'
    dynsim_edo_DW.obj_nb.matlabCodegenIsDeleted = false;
    dynsim_edo_DW.obj_nb.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      dynsim_edo_B.cv[i] = tmp_9[i];
    }

    dynsim_edo_B.cv[24] = '\x00';
    ParamGet_dynsim_edo_133.initialize(dynsim_edo_B.cv);
    ParamGet_dynsim_edo_133.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_edo_133.set_initial_value(0.0);
    dynsim_edo_DW.obj_nb.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S16>/Get Parameter8'
    emxInitStruct_robotics_slman_ha(&dynsim_edo_DW.obj);
    emxInitStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_1);
    emxInitStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_12);
    emxInitStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_11);
    emxInitStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_10);
    emxInitStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_9);
    emxInitStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_8);
    emxInitStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_7);
    emxInitStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_6);
    emxInitStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_5);
    emxInitStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_4);
    emxInitStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_3);
    emxInitStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_2);

    // Start for MATLABSystem: '<S15>/MATLAB System'
    dynsim_edo_DW.obj.isInitialized = 0;
    dynsim_edo_DW.obj.isInitialized = 1;
    RigidBodyTree_RigidBodyTree_ha(&dynsim_edo_DW.obj.TreeInternal,
      &dynsim_edo_DW.gobj_2, &dynsim_edo_DW.gobj_4, &dynsim_edo_DW.gobj_5,
      &dynsim_edo_DW.gobj_6, &dynsim_edo_DW.gobj_7, &dynsim_edo_DW.gobj_3);
  }

  // set "at time zero" to false
  if (rtmIsFirstInitCond(dynsim_edo_M)) {
    rtmSetFirstInitCond(dynsim_edo_M, 0);
  }
}

// Model terminate function
void dynsim_edo_terminate(void)
{
  // Terminate for MATLABSystem: '<S16>/Get Parameter'
  matlabCodegenHandle_matlab_hau4(&dynsim_edo_DW.obj_p);

  // Terminate for MATLABSystem: '<S16>/Get Parameter1'
  matlabCodegenHandle_matlab_hau4(&dynsim_edo_DW.obj_n);

  // Terminate for MATLABSystem: '<S16>/Get Parameter4'
  matlabCodegenHandle_matlab_hau4(&dynsim_edo_DW.obj_nf);
  emxFreeStruct_robotics_slmanip_(&dynsim_edo_DW.obj_a);
  emxFreeStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_1_p);
  emxFreeStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_12_m);
  emxFreeStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_11_a);
  emxFreeStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_10_h);
  emxFreeStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_9_d);
  emxFreeStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_8_l);
  emxFreeStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_7_p);
  emxFreeStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_6_n);
  emxFreeStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_5_l);
  emxFreeStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_4_b);
  emxFreeStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_3_g);
  emxFreeStruct_l_robotics_manip_(&dynsim_edo_DW.gobj_2_a);
  emxFreeStruct_robotics_slmani_h(&dynsim_edo_DW.obj_o);
  emxFreeStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_1_pg);
  emxFreeStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_12_c);
  emxFreeStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_11_d);
  emxFreeStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_10_k);
  emxFreeStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_9_b);
  emxFreeStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_8_o);
  emxFreeStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_7_h);
  emxFreeStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_6_o);
  emxFreeStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_5_o);
  emxFreeStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_4_c);
  emxFreeStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_3_m);
  emxFreeStruct_l_robotics_mani_h(&dynsim_edo_DW.gobj_2_j);

  // Terminate for MATLABSystem: '<S16>/Get Parameter2'
  matlabCodegenHandle_matlab_hau4(&dynsim_edo_DW.obj_h);

  // Terminate for MATLABSystem: '<S16>/Get Parameter3'
  matlabCodegenHandle_matlab_hau4(&dynsim_edo_DW.obj_b);

  // Terminate for MATLABSystem: '<S16>/Get Parameter8'
  matlabCodegenHandle_matlab_hau4(&dynsim_edo_DW.obj_nb);

  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S12>/SourceBlock'
  matlabCodegenHandle_matlabC_hau(&dynsim_edo_DW.obj_pp);

  // End of Terminate for SubSystem: '<Root>/Subscribe'
  emxFreeStruct_robotics_slman_ha(&dynsim_edo_DW.obj);
  emxFreeStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_1);
  emxFreeStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_12);
  emxFreeStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_11);
  emxFreeStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_10);
  emxFreeStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_9);
  emxFreeStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_8);
  emxFreeStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_7);
  emxFreeStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_6);
  emxFreeStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_5);
  emxFreeStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_4);
  emxFreeStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_3);
  emxFreeStruct_l_robotics_man_ha(&dynsim_edo_DW.gobj_2);

  // Terminate for Atomic SubSystem: '<Root>/Publish2'
  // Terminate for MATLABSystem: '<S11>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&dynsim_edo_DW.obj_ar);

  // End of Terminate for SubSystem: '<Root>/Publish2'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S9>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&dynsim_edo_DW.obj_nr);

  // End of Terminate for SubSystem: '<Root>/Publish'

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S10>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&dynsim_edo_DW.obj_f);

  // End of Terminate for SubSystem: '<Root>/Publish1'
}

//
// File trailer for generated code.
//
// [EOF]
//
