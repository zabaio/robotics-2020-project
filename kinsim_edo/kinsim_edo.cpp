//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: kinsim_edo.cpp
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
static void kinsim_edo_emxInit_real_T(emxArray_real_T_kinsim_edo_T **pEmxArray,
  int32_T numDimensions);
static void kinsim_edo_emxInit_f_cell_wrap(emxArray_f_cell_wrap_kinsim_e_T
  **pEmxArray, int32_T numDimensions);
static void kinsim_edo_emxInit_char_T(emxArray_char_T_kinsim_edo_T **pEmxArray,
  int32_T numDimensions);
static void k_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_kinsim_e_T
  *emxArray, int32_T oldNumel);
static void kinsim_emxEnsureCapacity_char_T(emxArray_char_T_kinsim_edo_T
  *emxArray, int32_T oldNumel);
static void ki_rigidBodyJoint_get_JointAxis(const c_rigidBodyJoint_kinsim_edo_T *
  obj, real_T ax[3]);
static void kinsim_edo_emxFree_char_T(emxArray_char_T_kinsim_edo_T **pEmxArray);
static void RigidBodyTree_forwardKinematics(n_robotics_manip_internal_Rig_T *obj,
  const real_T qvec[3], emxArray_f_cell_wrap_kinsim_e_T *Ttree);
static void kinsim_emxEnsureCapacity_real_T(emxArray_real_T_kinsim_edo_T
  *emxArray, int32_T oldNumel);
static void kinsim_edo_emxFree_real_T(emxArray_real_T_kinsim_edo_T **pEmxArray);
static void kinsim_edo_emxFree_f_cell_wrap(emxArray_f_cell_wrap_kinsim_e_T
  **pEmxArray);
static void RigidBodyTree_geometricJacobian(n_robotics_manip_internal_Rig_T *obj,
  const real_T Q[3], emxArray_real_T_kinsim_edo_T *Jac);
static void rigidBodyJoint_get_JointAxis_p(const c_rigidBodyJoint_kinsim_edo_p_T
  *obj, real_T ax[3]);
static void RigidBodyTree_forwardKinemati_p(n_robotics_manip_internal_R_p_T *obj,
  const real_T qvec[3], emxArray_f_cell_wrap_kinsim_e_T *Ttree);
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
static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_kinsim_edo_T
  *pStruct);
static void emxFreeStruct_m_robotics_manip_(m_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxFreeStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_kinsim_edo_p_T
  *pStruct);
static void emxFreeStruct_m_robotics_mani_p(m_robotics_manip_internal_R_p_T
  *pStruct);
static void emxFreeStruct_n_robotics_mani_p(n_robotics_manip_internal_R_p_T
  *pStruct);
static void emxFreeStruct_robotics_slmani_p(robotics_slmanip_internal_b_p_T
  *pStruct);
static void emxFreeStruct_l_robotics_mani_p(l_robotics_manip_internal_R_p_T
  *pStruct);
static void matlabCodegenHandle_matlabC_pcd(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj);
static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_kinsim_edo_T
  *pStruct);
static void emxInitStruct_m_robotics_manip_(m_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxInitStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct);
static l_robotics_manip_internal_Rig_T *kinsim_edo_RigidBody_RigidBody
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *kinsim_ed_RigidBody_RigidBody_p
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *kinsim_e_RigidBody_RigidBody_pc
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *kinsim__RigidBody_RigidBody_pcd
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *kinsim_RigidBody_RigidBody_pcdk
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *kinsi_RigidBody_RigidBody_pcdkl
  (l_robotics_manip_internal_Rig_T *obj);
static m_robotics_manip_internal_Rig_T *kins_RigidBody_RigidBody_pcdklh
  (m_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *kin_RigidBodyTree_RigidBodyTree
  (n_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Rig_T *iobj_0,
   l_robotics_manip_internal_Rig_T *iobj_1, l_robotics_manip_internal_Rig_T
   *iobj_2, l_robotics_manip_internal_Rig_T *iobj_3,
   l_robotics_manip_internal_Rig_T *iobj_4, l_robotics_manip_internal_Rig_T
   *iobj_5);
static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_kinsim_edo_p_T
  *pStruct);
static void emxInitStruct_m_robotics_mani_p(m_robotics_manip_internal_R_p_T
  *pStruct);
static void emxInitStruct_n_robotics_mani_p(n_robotics_manip_internal_R_p_T
  *pStruct);
static void emxInitStruct_robotics_slmani_p(robotics_slmanip_internal_b_p_T
  *pStruct);
static void emxInitStruct_l_robotics_mani_p(l_robotics_manip_internal_R_p_T
  *pStruct);
static l_robotics_manip_internal_R_p_T *kin_RigidBody_RigidBody_pcdklhm
  (l_robotics_manip_internal_R_p_T *obj);
static l_robotics_manip_internal_R_p_T *ki_RigidBody_RigidBody_pcdklhm5
  (l_robotics_manip_internal_R_p_T *obj);
static l_robotics_manip_internal_R_p_T *k_RigidBody_RigidBody_pcdklhm5m
  (l_robotics_manip_internal_R_p_T *obj);
static n_robotics_manip_internal_R_p_T *k_RigidBodyTree_RigidBodyTree_p
  (n_robotics_manip_internal_R_p_T *obj, l_robotics_manip_internal_R_p_T *iobj_0,
   l_robotics_manip_internal_R_p_T *iobj_1, l_robotics_manip_internal_R_p_T
   *iobj_2, l_robotics_manip_internal_R_p_T *iobj_3,
   l_robotics_manip_internal_R_p_T *iobj_4, l_robotics_manip_internal_R_p_T
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
  int_T nXc = 3;
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

static void kinsim_edo_emxInit_real_T(emxArray_real_T_kinsim_edo_T **pEmxArray,
  int32_T numDimensions)
{
  emxArray_real_T_kinsim_edo_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real_T_kinsim_edo_T *)malloc(sizeof
    (emxArray_real_T_kinsim_edo_T));
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void kinsim_edo_emxInit_f_cell_wrap(emxArray_f_cell_wrap_kinsim_e_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_cell_wrap_kinsim_e_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_f_cell_wrap_kinsim_e_T *)malloc(sizeof
    (emxArray_f_cell_wrap_kinsim_e_T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_kinsim_edo_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void kinsim_edo_emxInit_char_T(emxArray_char_T_kinsim_edo_T **pEmxArray,
  int32_T numDimensions)
{
  emxArray_char_T_kinsim_edo_T *emxArray;
  *pEmxArray = (emxArray_char_T_kinsim_edo_T *)malloc(sizeof
    (emxArray_char_T_kinsim_edo_T));
  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (kinsim_edo_B.i_l = 0; kinsim_edo_B.i_l < numDimensions; kinsim_edo_B.i_l
       ++) {
    emxArray->size[kinsim_edo_B.i_l] = 0;
  }
}

static void k_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_kinsim_e_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  kinsim_edo_B.newNumel_c = 1;
  for (kinsim_edo_B.i_o = 0; kinsim_edo_B.i_o < emxArray->numDimensions;
       kinsim_edo_B.i_o++) {
    kinsim_edo_B.newNumel_c *= emxArray->size[kinsim_edo_B.i_o];
  }

  if (kinsim_edo_B.newNumel_c > emxArray->allocatedSize) {
    kinsim_edo_B.i_o = emxArray->allocatedSize;
    if (kinsim_edo_B.i_o < 16) {
      kinsim_edo_B.i_o = 16;
    }

    while (kinsim_edo_B.i_o < kinsim_edo_B.newNumel_c) {
      if (kinsim_edo_B.i_o > 1073741823) {
        kinsim_edo_B.i_o = MAX_int32_T;
      } else {
        kinsim_edo_B.i_o <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(kinsim_edo_B.i_o), sizeof
                     (f_cell_wrap_kinsim_edo_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_kinsim_edo_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_kinsim_edo_T *)newData;
    emxArray->allocatedSize = kinsim_edo_B.i_o;
    emxArray->canFreeData = true;
  }
}

static void kinsim_emxEnsureCapacity_char_T(emxArray_char_T_kinsim_edo_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  kinsim_edo_B.newNumel = 1;
  for (kinsim_edo_B.i = 0; kinsim_edo_B.i < emxArray->numDimensions;
       kinsim_edo_B.i++) {
    kinsim_edo_B.newNumel *= emxArray->size[kinsim_edo_B.i];
  }

  if (kinsim_edo_B.newNumel > emxArray->allocatedSize) {
    kinsim_edo_B.i = emxArray->allocatedSize;
    if (kinsim_edo_B.i < 16) {
      kinsim_edo_B.i = 16;
    }

    while (kinsim_edo_B.i < kinsim_edo_B.newNumel) {
      if (kinsim_edo_B.i > 1073741823) {
        kinsim_edo_B.i = MAX_int32_T;
      } else {
        kinsim_edo_B.i <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(kinsim_edo_B.i), sizeof(char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = kinsim_edo_B.i;
    emxArray->canFreeData = true;
  }
}

static void ki_rigidBodyJoint_get_JointAxis(const c_rigidBodyJoint_kinsim_edo_T *
  obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (kinsim_edo_B.b_kstr_i = 0; kinsim_edo_B.b_kstr_i < 8;
       kinsim_edo_B.b_kstr_i++) {
    kinsim_edo_B.b_n[kinsim_edo_B.b_kstr_i] = tmp[kinsim_edo_B.b_kstr_i];
  }

  kinsim_edo_B.b_bool_m = false;
  if (obj->Type->size[1] == 8) {
    kinsim_edo_B.b_kstr_i = 1;
    do {
      exitg1 = 0;
      if (kinsim_edo_B.b_kstr_i - 1 < 8) {
        kinsim_edo_B.kstr_a = kinsim_edo_B.b_kstr_i - 1;
        if (obj->Type->data[kinsim_edo_B.kstr_a] !=
            kinsim_edo_B.b_n[kinsim_edo_B.kstr_a]) {
          exitg1 = 1;
        } else {
          kinsim_edo_B.b_kstr_i++;
        }
      } else {
        kinsim_edo_B.b_bool_m = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (kinsim_edo_B.b_bool_m) {
    guard1 = true;
  } else {
    for (kinsim_edo_B.b_kstr_i = 0; kinsim_edo_B.b_kstr_i < 9;
         kinsim_edo_B.b_kstr_i++) {
      kinsim_edo_B.b_d[kinsim_edo_B.b_kstr_i] = tmp_0[kinsim_edo_B.b_kstr_i];
    }

    kinsim_edo_B.b_bool_m = false;
    if (obj->Type->size[1] == 9) {
      kinsim_edo_B.b_kstr_i = 1;
      do {
        exitg1 = 0;
        if (kinsim_edo_B.b_kstr_i - 1 < 9) {
          kinsim_edo_B.kstr_a = kinsim_edo_B.b_kstr_i - 1;
          if (obj->Type->data[kinsim_edo_B.kstr_a] !=
              kinsim_edo_B.b_d[kinsim_edo_B.kstr_a]) {
            exitg1 = 1;
          } else {
            kinsim_edo_B.b_kstr_i++;
          }
        } else {
          kinsim_edo_B.b_bool_m = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (kinsim_edo_B.b_bool_m) {
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

static void kinsim_edo_emxFree_char_T(emxArray_char_T_kinsim_edo_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T_kinsim_edo_T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_char_T_kinsim_edo_T *)NULL;
  }
}

static void RigidBodyTree_forwardKinematics(n_robotics_manip_internal_Rig_T *obj,
  const real_T qvec[3], emxArray_f_cell_wrap_kinsim_e_T *Ttree)
{
  l_robotics_manip_internal_Rig_T *body;
  emxArray_char_T_kinsim_edo_T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  kinsim_edo_B.n = obj->NumBodies;
  for (kinsim_edo_B.b_kstr = 0; kinsim_edo_B.b_kstr < 16; kinsim_edo_B.b_kstr++)
  {
    kinsim_edo_B.c_f1[kinsim_edo_B.b_kstr] = tmp[kinsim_edo_B.b_kstr];
  }

  kinsim_edo_B.b_kstr = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  kinsim_edo_B.e = static_cast<int32_T>(kinsim_edo_B.n);
  Ttree->size[1] = kinsim_edo_B.e;
  k_emxEnsureCapacity_f_cell_wrap(Ttree, kinsim_edo_B.b_kstr);
  if (kinsim_edo_B.e != 0) {
    kinsim_edo_B.ntilecols = kinsim_edo_B.e - 1;
    if (0 <= kinsim_edo_B.ntilecols) {
      memcpy(&kinsim_edo_B.expl_temp.f1[0], &kinsim_edo_B.c_f1[0], sizeof(real_T)
             << 4U);
    }

    for (kinsim_edo_B.b_jtilecol = 0; kinsim_edo_B.b_jtilecol <=
         kinsim_edo_B.ntilecols; kinsim_edo_B.b_jtilecol++) {
      Ttree->data[kinsim_edo_B.b_jtilecol] = kinsim_edo_B.expl_temp;
    }
  }

  kinsim_edo_B.k = 1.0;
  kinsim_edo_B.ntilecols = static_cast<int32_T>(kinsim_edo_B.n) - 1;
  kinsim_edo_emxInit_char_T(&switch_expression, 2);
  if (0 <= kinsim_edo_B.ntilecols) {
    for (kinsim_edo_B.b_kstr = 0; kinsim_edo_B.b_kstr < 5; kinsim_edo_B.b_kstr++)
    {
      kinsim_edo_B.b_j[kinsim_edo_B.b_kstr] = tmp_0[kinsim_edo_B.b_kstr];
    }
  }

  for (kinsim_edo_B.b_jtilecol = 0; kinsim_edo_B.b_jtilecol <=
       kinsim_edo_B.ntilecols; kinsim_edo_B.b_jtilecol++) {
    body = obj->Bodies[kinsim_edo_B.b_jtilecol];
    kinsim_edo_B.n = body->JointInternal.PositionNumber;
    kinsim_edo_B.n += kinsim_edo_B.k;
    if (kinsim_edo_B.k > kinsim_edo_B.n - 1.0) {
      kinsim_edo_B.e = 0;
      kinsim_edo_B.d_e = 0;
    } else {
      kinsim_edo_B.e = static_cast<int32_T>(kinsim_edo_B.k) - 1;
      kinsim_edo_B.d_e = static_cast<int32_T>(kinsim_edo_B.n - 1.0);
    }

    kinsim_edo_B.b_kstr = switch_expression->size[0] * switch_expression->size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    kinsim_emxEnsureCapacity_char_T(switch_expression, kinsim_edo_B.b_kstr);
    kinsim_edo_B.loop_ub_ax = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (kinsim_edo_B.b_kstr = 0; kinsim_edo_B.b_kstr <= kinsim_edo_B.loop_ub_ax;
         kinsim_edo_B.b_kstr++) {
      switch_expression->data[kinsim_edo_B.b_kstr] = body->
        JointInternal.Type->data[kinsim_edo_B.b_kstr];
    }

    kinsim_edo_B.b_bool_h = false;
    if (switch_expression->size[1] == 5) {
      kinsim_edo_B.b_kstr = 1;
      do {
        exitg1 = 0;
        if (kinsim_edo_B.b_kstr - 1 < 5) {
          kinsim_edo_B.loop_ub_ax = kinsim_edo_B.b_kstr - 1;
          if (switch_expression->data[kinsim_edo_B.loop_ub_ax] !=
              kinsim_edo_B.b_j[kinsim_edo_B.loop_ub_ax]) {
            exitg1 = 1;
          } else {
            kinsim_edo_B.b_kstr++;
          }
        } else {
          kinsim_edo_B.b_bool_h = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (kinsim_edo_B.b_bool_h) {
      kinsim_edo_B.b_kstr = 0;
    } else {
      for (kinsim_edo_B.b_kstr = 0; kinsim_edo_B.b_kstr < 8; kinsim_edo_B.b_kstr
           ++) {
        kinsim_edo_B.b_b[kinsim_edo_B.b_kstr] = tmp_1[kinsim_edo_B.b_kstr];
      }

      kinsim_edo_B.b_bool_h = false;
      if (switch_expression->size[1] == 8) {
        kinsim_edo_B.b_kstr = 1;
        do {
          exitg1 = 0;
          if (kinsim_edo_B.b_kstr - 1 < 8) {
            kinsim_edo_B.loop_ub_ax = kinsim_edo_B.b_kstr - 1;
            if (switch_expression->data[kinsim_edo_B.loop_ub_ax] !=
                kinsim_edo_B.b_b[kinsim_edo_B.loop_ub_ax]) {
              exitg1 = 1;
            } else {
              kinsim_edo_B.b_kstr++;
            }
          } else {
            kinsim_edo_B.b_bool_h = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (kinsim_edo_B.b_bool_h) {
        kinsim_edo_B.b_kstr = 1;
      } else {
        kinsim_edo_B.b_kstr = -1;
      }
    }

    switch (kinsim_edo_B.b_kstr) {
     case 0:
      memset(&kinsim_edo_B.c_f1[0], 0, sizeof(real_T) << 4U);
      kinsim_edo_B.c_f1[0] = 1.0;
      kinsim_edo_B.c_f1[5] = 1.0;
      kinsim_edo_B.c_f1[10] = 1.0;
      kinsim_edo_B.c_f1[15] = 1.0;
      break;

     case 1:
      ki_rigidBodyJoint_get_JointAxis(&body->JointInternal, kinsim_edo_B.v);
      kinsim_edo_B.d_e -= kinsim_edo_B.e;
      for (kinsim_edo_B.b_kstr = 0; kinsim_edo_B.b_kstr < kinsim_edo_B.d_e;
           kinsim_edo_B.b_kstr++) {
        kinsim_edo_B.e_data[kinsim_edo_B.b_kstr] = kinsim_edo_B.e +
          kinsim_edo_B.b_kstr;
      }

      kinsim_edo_B.result_data[0] = kinsim_edo_B.v[0];
      kinsim_edo_B.result_data[1] = kinsim_edo_B.v[1];
      kinsim_edo_B.result_data[2] = kinsim_edo_B.v[2];
      if (0 <= (kinsim_edo_B.d_e != 0) - 1) {
        kinsim_edo_B.result_data[3] = qvec[kinsim_edo_B.e_data[0]];
      }

      kinsim_edo_B.k = 1.0 / sqrt((kinsim_edo_B.result_data[0] *
        kinsim_edo_B.result_data[0] + kinsim_edo_B.result_data[1] *
        kinsim_edo_B.result_data[1]) + kinsim_edo_B.result_data[2] *
        kinsim_edo_B.result_data[2]);
      kinsim_edo_B.v[0] = kinsim_edo_B.result_data[0] * kinsim_edo_B.k;
      kinsim_edo_B.v[1] = kinsim_edo_B.result_data[1] * kinsim_edo_B.k;
      kinsim_edo_B.v[2] = kinsim_edo_B.result_data[2] * kinsim_edo_B.k;
      kinsim_edo_B.k = cos(kinsim_edo_B.result_data[3]);
      kinsim_edo_B.sth = sin(kinsim_edo_B.result_data[3]);
      kinsim_edo_B.tempR[0] = kinsim_edo_B.v[0] * kinsim_edo_B.v[0] * (1.0 -
        kinsim_edo_B.k) + kinsim_edo_B.k;
      kinsim_edo_B.tempR_tmp = kinsim_edo_B.v[1] * kinsim_edo_B.v[0] * (1.0 -
        kinsim_edo_B.k);
      kinsim_edo_B.tempR_tmp_b = kinsim_edo_B.v[2] * kinsim_edo_B.sth;
      kinsim_edo_B.tempR[1] = kinsim_edo_B.tempR_tmp - kinsim_edo_B.tempR_tmp_b;
      kinsim_edo_B.tempR_tmp_d = kinsim_edo_B.v[2] * kinsim_edo_B.v[0] * (1.0 -
        kinsim_edo_B.k);
      kinsim_edo_B.tempR_tmp_e = kinsim_edo_B.v[1] * kinsim_edo_B.sth;
      kinsim_edo_B.tempR[2] = kinsim_edo_B.tempR_tmp_d +
        kinsim_edo_B.tempR_tmp_e;
      kinsim_edo_B.tempR[3] = kinsim_edo_B.tempR_tmp + kinsim_edo_B.tempR_tmp_b;
      kinsim_edo_B.tempR[4] = kinsim_edo_B.v[1] * kinsim_edo_B.v[1] * (1.0 -
        kinsim_edo_B.k) + kinsim_edo_B.k;
      kinsim_edo_B.tempR_tmp = kinsim_edo_B.v[2] * kinsim_edo_B.v[1] * (1.0 -
        kinsim_edo_B.k);
      kinsim_edo_B.tempR_tmp_b = kinsim_edo_B.v[0] * kinsim_edo_B.sth;
      kinsim_edo_B.tempR[5] = kinsim_edo_B.tempR_tmp - kinsim_edo_B.tempR_tmp_b;
      kinsim_edo_B.tempR[6] = kinsim_edo_B.tempR_tmp_d -
        kinsim_edo_B.tempR_tmp_e;
      kinsim_edo_B.tempR[7] = kinsim_edo_B.tempR_tmp + kinsim_edo_B.tempR_tmp_b;
      kinsim_edo_B.tempR[8] = kinsim_edo_B.v[2] * kinsim_edo_B.v[2] * (1.0 -
        kinsim_edo_B.k) + kinsim_edo_B.k;
      for (kinsim_edo_B.b_kstr = 0; kinsim_edo_B.b_kstr < 3; kinsim_edo_B.b_kstr
           ++) {
        kinsim_edo_B.e = kinsim_edo_B.b_kstr + 1;
        kinsim_edo_B.R_n[kinsim_edo_B.e - 1] = kinsim_edo_B.tempR
          [(kinsim_edo_B.e - 1) * 3];
        kinsim_edo_B.e = kinsim_edo_B.b_kstr + 1;
        kinsim_edo_B.R_n[kinsim_edo_B.e + 2] = kinsim_edo_B.tempR
          [(kinsim_edo_B.e - 1) * 3 + 1];
        kinsim_edo_B.e = kinsim_edo_B.b_kstr + 1;
        kinsim_edo_B.R_n[kinsim_edo_B.e + 5] = kinsim_edo_B.tempR
          [(kinsim_edo_B.e - 1) * 3 + 2];
      }

      memset(&kinsim_edo_B.c_f1[0], 0, sizeof(real_T) << 4U);
      for (kinsim_edo_B.b_kstr = 0; kinsim_edo_B.b_kstr < 3; kinsim_edo_B.b_kstr
           ++) {
        kinsim_edo_B.d_e = kinsim_edo_B.b_kstr << 2;
        kinsim_edo_B.c_f1[kinsim_edo_B.d_e] = kinsim_edo_B.R_n[3 *
          kinsim_edo_B.b_kstr];
        kinsim_edo_B.c_f1[kinsim_edo_B.d_e + 1] = kinsim_edo_B.R_n[3 *
          kinsim_edo_B.b_kstr + 1];
        kinsim_edo_B.c_f1[kinsim_edo_B.d_e + 2] = kinsim_edo_B.R_n[3 *
          kinsim_edo_B.b_kstr + 2];
      }

      kinsim_edo_B.c_f1[15] = 1.0;
      break;

     default:
      ki_rigidBodyJoint_get_JointAxis(&body->JointInternal, kinsim_edo_B.v);
      memset(&kinsim_edo_B.tempR[0], 0, 9U * sizeof(real_T));
      kinsim_edo_B.tempR[0] = 1.0;
      kinsim_edo_B.tempR[4] = 1.0;
      kinsim_edo_B.tempR[8] = 1.0;
      for (kinsim_edo_B.b_kstr = 0; kinsim_edo_B.b_kstr < 3; kinsim_edo_B.b_kstr
           ++) {
        kinsim_edo_B.d_e = kinsim_edo_B.b_kstr << 2;
        kinsim_edo_B.c_f1[kinsim_edo_B.d_e] = kinsim_edo_B.tempR[3 *
          kinsim_edo_B.b_kstr];
        kinsim_edo_B.c_f1[kinsim_edo_B.d_e + 1] = kinsim_edo_B.tempR[3 *
          kinsim_edo_B.b_kstr + 1];
        kinsim_edo_B.c_f1[kinsim_edo_B.d_e + 2] = kinsim_edo_B.tempR[3 *
          kinsim_edo_B.b_kstr + 2];
        kinsim_edo_B.c_f1[kinsim_edo_B.b_kstr + 12] =
          kinsim_edo_B.v[kinsim_edo_B.b_kstr] * qvec[kinsim_edo_B.e];
      }

      kinsim_edo_B.c_f1[3] = 0.0;
      kinsim_edo_B.c_f1[7] = 0.0;
      kinsim_edo_B.c_f1[11] = 0.0;
      kinsim_edo_B.c_f1[15] = 1.0;
      break;
    }

    for (kinsim_edo_B.b_kstr = 0; kinsim_edo_B.b_kstr < 16; kinsim_edo_B.b_kstr
         ++) {
      kinsim_edo_B.a[kinsim_edo_B.b_kstr] =
        body->JointInternal.JointToParentTransform[kinsim_edo_B.b_kstr];
    }

    for (kinsim_edo_B.b_kstr = 0; kinsim_edo_B.b_kstr < 16; kinsim_edo_B.b_kstr
         ++) {
      kinsim_edo_B.b[kinsim_edo_B.b_kstr] =
        body->JointInternal.ChildToJointTransform[kinsim_edo_B.b_kstr];
    }

    for (kinsim_edo_B.b_kstr = 0; kinsim_edo_B.b_kstr < 4; kinsim_edo_B.b_kstr++)
    {
      for (kinsim_edo_B.e = 0; kinsim_edo_B.e < 4; kinsim_edo_B.e++) {
        kinsim_edo_B.d_e = kinsim_edo_B.e << 2;
        kinsim_edo_B.loop_ub_ax = kinsim_edo_B.b_kstr + kinsim_edo_B.d_e;
        kinsim_edo_B.a_k[kinsim_edo_B.loop_ub_ax] = 0.0;
        kinsim_edo_B.a_k[kinsim_edo_B.loop_ub_ax] +=
          kinsim_edo_B.c_f1[kinsim_edo_B.d_e] *
          kinsim_edo_B.a[kinsim_edo_B.b_kstr];
        kinsim_edo_B.a_k[kinsim_edo_B.loop_ub_ax] +=
          kinsim_edo_B.c_f1[kinsim_edo_B.d_e + 1] *
          kinsim_edo_B.a[kinsim_edo_B.b_kstr + 4];
        kinsim_edo_B.a_k[kinsim_edo_B.loop_ub_ax] +=
          kinsim_edo_B.c_f1[kinsim_edo_B.d_e + 2] *
          kinsim_edo_B.a[kinsim_edo_B.b_kstr + 8];
        kinsim_edo_B.a_k[kinsim_edo_B.loop_ub_ax] +=
          kinsim_edo_B.c_f1[kinsim_edo_B.d_e + 3] *
          kinsim_edo_B.a[kinsim_edo_B.b_kstr + 12];
      }

      for (kinsim_edo_B.e = 0; kinsim_edo_B.e < 4; kinsim_edo_B.e++) {
        kinsim_edo_B.d_e = kinsim_edo_B.e << 2;
        kinsim_edo_B.loop_ub_ax = kinsim_edo_B.b_kstr + kinsim_edo_B.d_e;
        Ttree->data[kinsim_edo_B.b_jtilecol].f1[kinsim_edo_B.loop_ub_ax] = 0.0;
        Ttree->data[kinsim_edo_B.b_jtilecol].f1[kinsim_edo_B.loop_ub_ax] +=
          kinsim_edo_B.b[kinsim_edo_B.d_e] *
          kinsim_edo_B.a_k[kinsim_edo_B.b_kstr];
        Ttree->data[kinsim_edo_B.b_jtilecol].f1[kinsim_edo_B.loop_ub_ax] +=
          kinsim_edo_B.b[kinsim_edo_B.d_e + 1] *
          kinsim_edo_B.a_k[kinsim_edo_B.b_kstr + 4];
        Ttree->data[kinsim_edo_B.b_jtilecol].f1[kinsim_edo_B.loop_ub_ax] +=
          kinsim_edo_B.b[kinsim_edo_B.d_e + 2] *
          kinsim_edo_B.a_k[kinsim_edo_B.b_kstr + 8];
        Ttree->data[kinsim_edo_B.b_jtilecol].f1[kinsim_edo_B.loop_ub_ax] +=
          kinsim_edo_B.b[kinsim_edo_B.d_e + 3] *
          kinsim_edo_B.a_k[kinsim_edo_B.b_kstr + 12];
      }
    }

    kinsim_edo_B.k = kinsim_edo_B.n;
    if (body->ParentIndex > 0.0) {
      for (kinsim_edo_B.b_kstr = 0; kinsim_edo_B.b_kstr < 16;
           kinsim_edo_B.b_kstr++) {
        kinsim_edo_B.a[kinsim_edo_B.b_kstr] = Ttree->data[static_cast<int32_T>
          (body->ParentIndex) - 1].f1[kinsim_edo_B.b_kstr];
      }

      for (kinsim_edo_B.b_kstr = 0; kinsim_edo_B.b_kstr < 4; kinsim_edo_B.b_kstr
           ++) {
        for (kinsim_edo_B.e = 0; kinsim_edo_B.e < 4; kinsim_edo_B.e++) {
          kinsim_edo_B.d_e = kinsim_edo_B.e << 2;
          kinsim_edo_B.loop_ub_ax = kinsim_edo_B.b_kstr + kinsim_edo_B.d_e;
          kinsim_edo_B.a_k[kinsim_edo_B.loop_ub_ax] = 0.0;
          kinsim_edo_B.a_k[kinsim_edo_B.loop_ub_ax] += Ttree->
            data[kinsim_edo_B.b_jtilecol].f1[kinsim_edo_B.d_e] *
            kinsim_edo_B.a[kinsim_edo_B.b_kstr];
          kinsim_edo_B.a_k[kinsim_edo_B.loop_ub_ax] += Ttree->
            data[kinsim_edo_B.b_jtilecol].f1[kinsim_edo_B.d_e + 1] *
            kinsim_edo_B.a[kinsim_edo_B.b_kstr + 4];
          kinsim_edo_B.a_k[kinsim_edo_B.loop_ub_ax] += Ttree->
            data[kinsim_edo_B.b_jtilecol].f1[kinsim_edo_B.d_e + 2] *
            kinsim_edo_B.a[kinsim_edo_B.b_kstr + 8];
          kinsim_edo_B.a_k[kinsim_edo_B.loop_ub_ax] += Ttree->
            data[kinsim_edo_B.b_jtilecol].f1[kinsim_edo_B.d_e + 3] *
            kinsim_edo_B.a[kinsim_edo_B.b_kstr + 12];
        }
      }

      for (kinsim_edo_B.b_kstr = 0; kinsim_edo_B.b_kstr < 16;
           kinsim_edo_B.b_kstr++) {
        Ttree->data[kinsim_edo_B.b_jtilecol].f1[kinsim_edo_B.b_kstr] =
          kinsim_edo_B.a_k[kinsim_edo_B.b_kstr];
      }
    }
  }

  kinsim_edo_emxFree_char_T(&switch_expression);
}

static void kinsim_emxEnsureCapacity_real_T(emxArray_real_T_kinsim_edo_T
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

    newData = calloc(static_cast<uint32_T>(i), sizeof(real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void kinsim_edo_emxFree_real_T(emxArray_real_T_kinsim_edo_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T_kinsim_edo_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T_kinsim_edo_T *)NULL;
  }
}

static void kinsim_edo_emxFree_f_cell_wrap(emxArray_f_cell_wrap_kinsim_e_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_cell_wrap_kinsim_e_T *)NULL) {
    if (((*pEmxArray)->data != (f_cell_wrap_kinsim_edo_T *)NULL) && (*pEmxArray
        )->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_cell_wrap_kinsim_e_T *)NULL;
  }
}

static void RigidBodyTree_geometricJacobian(n_robotics_manip_internal_Rig_T *obj,
  const real_T Q[3], emxArray_real_T_kinsim_edo_T *Jac)
{
  emxArray_f_cell_wrap_kinsim_e_T *Ttree;
  l_robotics_manip_internal_Rig_T *body;
  emxArray_real_T_kinsim_edo_T *JacSlice;
  emxArray_char_T_kinsim_edo_T *bname;
  emxArray_real_T_kinsim_edo_T *b;
  static const char_T tmp[6] = { 'l', 'i', 'n', 'k', '_', '4' };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  boolean_T exitg2;
  kinsim_edo_emxInit_f_cell_wrap(&Ttree, 2);
  RigidBodyTree_forwardKinematics(obj, Q, Ttree);
  kinsim_edo_B.ret_p = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = static_cast<int32_T>(obj->VelocityNumber);
  kinsim_emxEnsureCapacity_real_T(Jac, kinsim_edo_B.ret_p);
  kinsim_edo_B.loop_ub_a = 6 * static_cast<int32_T>(obj->VelocityNumber) - 1;
  for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p <= kinsim_edo_B.loop_ub_a;
       kinsim_edo_B.ret_p++) {
    Jac->data[kinsim_edo_B.ret_p] = 0.0;
  }

  for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 6; kinsim_edo_B.ret_p++) {
    kinsim_edo_B.chainmask[kinsim_edo_B.ret_p] = 0;
  }

  kinsim_edo_emxInit_char_T(&bname, 2);
  kinsim_edo_B.ret_p = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  kinsim_emxEnsureCapacity_char_T(bname, kinsim_edo_B.ret_p);
  kinsim_edo_B.loop_ub_a = obj->Base.NameInternal->size[0] *
    obj->Base.NameInternal->size[1] - 1;
  for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p <= kinsim_edo_B.loop_ub_a;
       kinsim_edo_B.ret_p++) {
    bname->data[kinsim_edo_B.ret_p] = obj->Base.NameInternal->
      data[kinsim_edo_B.ret_p];
  }

  for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 6; kinsim_edo_B.ret_p++) {
    kinsim_edo_B.a_cz[kinsim_edo_B.ret_p] = tmp[kinsim_edo_B.ret_p];
  }

  kinsim_edo_B.b_bool_l = false;
  if (6 == bname->size[1]) {
    kinsim_edo_B.ret_p = 1;
    do {
      exitg1 = 0;
      if (kinsim_edo_B.ret_p - 1 < 6) {
        kinsim_edo_B.kstr = kinsim_edo_B.ret_p - 1;
        if (kinsim_edo_B.a_cz[kinsim_edo_B.kstr] != bname->
            data[kinsim_edo_B.kstr]) {
          exitg1 = 1;
        } else {
          kinsim_edo_B.ret_p++;
        }
      } else {
        kinsim_edo_B.b_bool_l = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (kinsim_edo_B.b_bool_l) {
    memset(&kinsim_edo_B.T2inv[0], 0, sizeof(real_T) << 4U);
    kinsim_edo_B.T2inv[0] = 1.0;
    kinsim_edo_B.T2inv[5] = 1.0;
    kinsim_edo_B.T2inv[10] = 1.0;
    kinsim_edo_B.T2inv[15] = 1.0;
    memset(&kinsim_edo_B.T2[0], 0, sizeof(real_T) << 4U);
    kinsim_edo_B.T2[0] = 1.0;
    kinsim_edo_B.T2[5] = 1.0;
    kinsim_edo_B.T2[10] = 1.0;
    kinsim_edo_B.T2[15] = 1.0;
  } else {
    kinsim_edo_B.endeffectorIndex = -1.0;
    kinsim_edo_B.ret_p = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = obj->Base.NameInternal->size[1];
    kinsim_emxEnsureCapacity_char_T(bname, kinsim_edo_B.ret_p);
    kinsim_edo_B.loop_ub_a = obj->Base.NameInternal->size[0] *
      obj->Base.NameInternal->size[1] - 1;
    for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p <= kinsim_edo_B.loop_ub_a;
         kinsim_edo_B.ret_p++) {
      bname->data[kinsim_edo_B.ret_p] = obj->Base.NameInternal->
        data[kinsim_edo_B.ret_p];
    }

    for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 6; kinsim_edo_B.ret_p++) {
      kinsim_edo_B.a_cz[kinsim_edo_B.ret_p] = tmp[kinsim_edo_B.ret_p];
    }

    kinsim_edo_B.b_bool_l = false;
    if (bname->size[1] == 6) {
      kinsim_edo_B.ret_p = 1;
      do {
        exitg1 = 0;
        if (kinsim_edo_B.ret_p - 1 < 6) {
          kinsim_edo_B.kstr = kinsim_edo_B.ret_p - 1;
          if (bname->data[kinsim_edo_B.kstr] !=
              kinsim_edo_B.a_cz[kinsim_edo_B.kstr]) {
            exitg1 = 1;
          } else {
            kinsim_edo_B.ret_p++;
          }
        } else {
          kinsim_edo_B.b_bool_l = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (kinsim_edo_B.b_bool_l) {
      kinsim_edo_B.endeffectorIndex = 0.0;
    } else {
      kinsim_edo_B.idx_idx_1 = obj->NumBodies;
      kinsim_edo_B.b_i_c = 0;
      exitg2 = false;
      while ((!exitg2) && (kinsim_edo_B.b_i_c <= static_cast<int32_T>
                           (kinsim_edo_B.idx_idx_1) - 1)) {
        body = obj->Bodies[kinsim_edo_B.b_i_c];
        for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 6; kinsim_edo_B.ret_p
             ++) {
          kinsim_edo_B.bname_m[kinsim_edo_B.ret_p] = body->
            NameInternal[kinsim_edo_B.ret_p];
        }

        for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 6; kinsim_edo_B.ret_p
             ++) {
          kinsim_edo_B.a_cz[kinsim_edo_B.ret_p] = tmp[kinsim_edo_B.ret_p];
        }

        kinsim_edo_B.ret_p = memcmp(&kinsim_edo_B.bname_m[0],
          &kinsim_edo_B.a_cz[0], 6);
        if (kinsim_edo_B.ret_p == 0) {
          kinsim_edo_B.endeffectorIndex = static_cast<real_T>(kinsim_edo_B.b_i_c)
            + 1.0;
          exitg2 = true;
        } else {
          kinsim_edo_B.b_i_c++;
        }
      }
    }

    kinsim_edo_B.b_i_c = static_cast<int32_T>(kinsim_edo_B.endeffectorIndex) - 1;
    body = obj->Bodies[kinsim_edo_B.b_i_c];
    for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 16; kinsim_edo_B.ret_p++)
    {
      kinsim_edo_B.T2[kinsim_edo_B.ret_p] = Ttree->data[kinsim_edo_B.b_i_c]
        .f1[kinsim_edo_B.ret_p];
    }

    for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 3; kinsim_edo_B.ret_p++) {
      kinsim_edo_B.R_g1[3 * kinsim_edo_B.ret_p] = Ttree->data[kinsim_edo_B.b_i_c]
        .f1[kinsim_edo_B.ret_p];
      kinsim_edo_B.R_g1[3 * kinsim_edo_B.ret_p + 1] = Ttree->
        data[kinsim_edo_B.b_i_c].f1[kinsim_edo_B.ret_p + 4];
      kinsim_edo_B.R_g1[3 * kinsim_edo_B.ret_p + 2] = Ttree->
        data[kinsim_edo_B.b_i_c].f1[kinsim_edo_B.ret_p + 8];
    }

    for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 9; kinsim_edo_B.ret_p++) {
      kinsim_edo_B.R_m[kinsim_edo_B.ret_p] =
        -kinsim_edo_B.R_g1[kinsim_edo_B.ret_p];
    }

    for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 3; kinsim_edo_B.ret_p++) {
      kinsim_edo_B.endeffectorIndex = Ttree->data[kinsim_edo_B.b_i_c].f1[12] *
        kinsim_edo_B.R_m[kinsim_edo_B.ret_p];
      kinsim_edo_B.loop_ub_a = kinsim_edo_B.ret_p << 2;
      kinsim_edo_B.T2inv[kinsim_edo_B.loop_ub_a] = kinsim_edo_B.R_g1[3 *
        kinsim_edo_B.ret_p];
      kinsim_edo_B.endeffectorIndex += kinsim_edo_B.R_m[kinsim_edo_B.ret_p + 3] *
        Ttree->data[kinsim_edo_B.b_i_c].f1[13];
      kinsim_edo_B.T2inv[kinsim_edo_B.loop_ub_a + 1] = kinsim_edo_B.R_g1[3 *
        kinsim_edo_B.ret_p + 1];
      kinsim_edo_B.endeffectorIndex += kinsim_edo_B.R_m[kinsim_edo_B.ret_p + 6] *
        Ttree->data[kinsim_edo_B.b_i_c].f1[14];
      kinsim_edo_B.T2inv[kinsim_edo_B.loop_ub_a + 2] = kinsim_edo_B.R_g1[3 *
        kinsim_edo_B.ret_p + 2];
      kinsim_edo_B.T2inv[kinsim_edo_B.ret_p + 12] =
        kinsim_edo_B.endeffectorIndex;
    }

    kinsim_edo_B.T2inv[3] = 0.0;
    kinsim_edo_B.T2inv[7] = 0.0;
    kinsim_edo_B.T2inv[11] = 0.0;
    kinsim_edo_B.T2inv[15] = 1.0;
    kinsim_edo_B.chainmask[kinsim_edo_B.b_i_c] = 1;
    while (body->ParentIndex > 0.0) {
      body = obj->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
      kinsim_edo_B.chainmask[static_cast<int32_T>(body->Index) - 1] = 1;
    }
  }

  kinsim_edo_B.idx_idx_1 = obj->NumBodies;
  kinsim_edo_B.c_c = static_cast<int32_T>(kinsim_edo_B.idx_idx_1) - 1;
  kinsim_edo_emxInit_real_T(&JacSlice, 2);
  kinsim_edo_emxInit_real_T(&b, 2);
  if (0 <= kinsim_edo_B.c_c) {
    for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 5; kinsim_edo_B.ret_p++) {
      kinsim_edo_B.b_m3[kinsim_edo_B.ret_p] = tmp_0[kinsim_edo_B.ret_p];
    }
  }

  for (kinsim_edo_B.b_i_c = 0; kinsim_edo_B.b_i_c <= kinsim_edo_B.c_c;
       kinsim_edo_B.b_i_c++) {
    body = obj->Bodies[kinsim_edo_B.b_i_c];
    kinsim_edo_B.ret_p = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = body->JointInternal.Type->size[1];
    kinsim_emxEnsureCapacity_char_T(bname, kinsim_edo_B.ret_p);
    kinsim_edo_B.loop_ub_a = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p <= kinsim_edo_B.loop_ub_a;
         kinsim_edo_B.ret_p++) {
      bname->data[kinsim_edo_B.ret_p] = body->JointInternal.Type->
        data[kinsim_edo_B.ret_p];
    }

    kinsim_edo_B.b_bool_l = false;
    if (bname->size[1] == 5) {
      kinsim_edo_B.ret_p = 1;
      do {
        exitg1 = 0;
        if (kinsim_edo_B.ret_p - 1 < 5) {
          kinsim_edo_B.kstr = kinsim_edo_B.ret_p - 1;
          if (bname->data[kinsim_edo_B.kstr] !=
              kinsim_edo_B.b_m3[kinsim_edo_B.kstr]) {
            exitg1 = 1;
          } else {
            kinsim_edo_B.ret_p++;
          }
        } else {
          kinsim_edo_B.b_bool_l = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if ((!kinsim_edo_B.b_bool_l) && (kinsim_edo_B.chainmask[kinsim_edo_B.b_i_c]
         != 0)) {
      for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 16; kinsim_edo_B.ret_p++)
      {
        kinsim_edo_B.T1_c[kinsim_edo_B.ret_p] = Ttree->data[static_cast<int32_T>
          (body->Index) - 1].f1[kinsim_edo_B.ret_p];
      }

      for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 16; kinsim_edo_B.ret_p++)
      {
        kinsim_edo_B.Tdh[kinsim_edo_B.ret_p] =
          body->JointInternal.ChildToJointTransform[kinsim_edo_B.ret_p];
      }

      for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 3; kinsim_edo_B.ret_p++)
      {
        kinsim_edo_B.R_g1[3 * kinsim_edo_B.ret_p] =
          kinsim_edo_B.Tdh[kinsim_edo_B.ret_p];
        kinsim_edo_B.R_g1[3 * kinsim_edo_B.ret_p + 1] =
          kinsim_edo_B.Tdh[kinsim_edo_B.ret_p + 4];
        kinsim_edo_B.R_g1[3 * kinsim_edo_B.ret_p + 2] =
          kinsim_edo_B.Tdh[kinsim_edo_B.ret_p + 8];
      }

      for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 9; kinsim_edo_B.ret_p++)
      {
        kinsim_edo_B.R_m[kinsim_edo_B.ret_p] =
          -kinsim_edo_B.R_g1[kinsim_edo_B.ret_p];
      }

      for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 3; kinsim_edo_B.ret_p++)
      {
        kinsim_edo_B.R_d[kinsim_edo_B.ret_p] =
          kinsim_edo_B.R_m[kinsim_edo_B.ret_p + 6] * kinsim_edo_B.Tdh[14] +
          (kinsim_edo_B.R_m[kinsim_edo_B.ret_p + 3] * kinsim_edo_B.Tdh[13] +
           kinsim_edo_B.R_m[kinsim_edo_B.ret_p] * kinsim_edo_B.Tdh[12]);
      }

      for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 4; kinsim_edo_B.ret_p++)
      {
        for (kinsim_edo_B.kstr = 0; kinsim_edo_B.kstr < 4; kinsim_edo_B.kstr++)
        {
          kinsim_edo_B.n_p = kinsim_edo_B.kstr << 2;
          kinsim_edo_B.loop_ub_a = kinsim_edo_B.ret_p + kinsim_edo_B.n_p;
          kinsim_edo_B.Tdh[kinsim_edo_B.loop_ub_a] = 0.0;
          kinsim_edo_B.Tdh[kinsim_edo_B.loop_ub_a] +=
            kinsim_edo_B.T1_c[kinsim_edo_B.n_p] *
            kinsim_edo_B.T2inv[kinsim_edo_B.ret_p];
          kinsim_edo_B.Tdh[kinsim_edo_B.loop_ub_a] +=
            kinsim_edo_B.T1_c[kinsim_edo_B.n_p + 1] *
            kinsim_edo_B.T2inv[kinsim_edo_B.ret_p + 4];
          kinsim_edo_B.Tdh[kinsim_edo_B.loop_ub_a] +=
            kinsim_edo_B.T1_c[kinsim_edo_B.n_p + 2] *
            kinsim_edo_B.T2inv[kinsim_edo_B.ret_p + 8];
          kinsim_edo_B.Tdh[kinsim_edo_B.loop_ub_a] +=
            kinsim_edo_B.T1_c[kinsim_edo_B.n_p + 3] *
            kinsim_edo_B.T2inv[kinsim_edo_B.ret_p + 12];
        }
      }

      for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 3; kinsim_edo_B.ret_p++)
      {
        kinsim_edo_B.kstr = kinsim_edo_B.ret_p << 2;
        kinsim_edo_B.T1_c[kinsim_edo_B.kstr] = kinsim_edo_B.R_g1[3 *
          kinsim_edo_B.ret_p];
        kinsim_edo_B.T1_c[kinsim_edo_B.kstr + 1] = kinsim_edo_B.R_g1[3 *
          kinsim_edo_B.ret_p + 1];
        kinsim_edo_B.T1_c[kinsim_edo_B.kstr + 2] = kinsim_edo_B.R_g1[3 *
          kinsim_edo_B.ret_p + 2];
        kinsim_edo_B.T1_c[kinsim_edo_B.ret_p + 12] =
          kinsim_edo_B.R_d[kinsim_edo_B.ret_p];
      }

      kinsim_edo_B.T1_c[3] = 0.0;
      kinsim_edo_B.T1_c[7] = 0.0;
      kinsim_edo_B.T1_c[11] = 0.0;
      kinsim_edo_B.T1_c[15] = 1.0;
      for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 4; kinsim_edo_B.ret_p++)
      {
        for (kinsim_edo_B.kstr = 0; kinsim_edo_B.kstr < 4; kinsim_edo_B.kstr++)
        {
          kinsim_edo_B.loop_ub_a = kinsim_edo_B.kstr << 2;
          kinsim_edo_B.n_p = kinsim_edo_B.ret_p + kinsim_edo_B.loop_ub_a;
          kinsim_edo_B.T[kinsim_edo_B.n_p] = 0.0;
          kinsim_edo_B.T[kinsim_edo_B.n_p] +=
            kinsim_edo_B.T1_c[kinsim_edo_B.loop_ub_a] *
            kinsim_edo_B.Tdh[kinsim_edo_B.ret_p];
          kinsim_edo_B.T[kinsim_edo_B.n_p] +=
            kinsim_edo_B.T1_c[kinsim_edo_B.loop_ub_a + 1] *
            kinsim_edo_B.Tdh[kinsim_edo_B.ret_p + 4];
          kinsim_edo_B.T[kinsim_edo_B.n_p] +=
            kinsim_edo_B.T1_c[kinsim_edo_B.loop_ub_a + 2] *
            kinsim_edo_B.Tdh[kinsim_edo_B.ret_p + 8];
          kinsim_edo_B.T[kinsim_edo_B.n_p] +=
            kinsim_edo_B.T1_c[kinsim_edo_B.loop_ub_a + 3] *
            kinsim_edo_B.Tdh[kinsim_edo_B.ret_p + 12];
        }
      }

      kinsim_edo_B.endeffectorIndex = obj->PositionDoFMap[kinsim_edo_B.b_i_c];
      kinsim_edo_B.idx_idx_1 = obj->PositionDoFMap[kinsim_edo_B.b_i_c + 6];
      kinsim_edo_B.R_g1[0] = 0.0;
      kinsim_edo_B.R_g1[3] = -kinsim_edo_B.T[14];
      kinsim_edo_B.R_g1[6] = kinsim_edo_B.T[13];
      kinsim_edo_B.R_g1[1] = kinsim_edo_B.T[14];
      kinsim_edo_B.R_g1[4] = 0.0;
      kinsim_edo_B.R_g1[7] = -kinsim_edo_B.T[12];
      kinsim_edo_B.R_g1[2] = -kinsim_edo_B.T[13];
      kinsim_edo_B.R_g1[5] = kinsim_edo_B.T[12];
      kinsim_edo_B.R_g1[8] = 0.0;
      for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 3; kinsim_edo_B.ret_p++)
      {
        for (kinsim_edo_B.kstr = 0; kinsim_edo_B.kstr < 3; kinsim_edo_B.kstr++)
        {
          kinsim_edo_B.loop_ub_a = kinsim_edo_B.ret_p + 3 * kinsim_edo_B.kstr;
          kinsim_edo_B.R_m[kinsim_edo_B.loop_ub_a] = 0.0;
          kinsim_edo_B.n_p = kinsim_edo_B.kstr << 2;
          kinsim_edo_B.R_m[kinsim_edo_B.loop_ub_a] +=
            kinsim_edo_B.T[kinsim_edo_B.n_p] *
            kinsim_edo_B.R_g1[kinsim_edo_B.ret_p];
          kinsim_edo_B.R_m[kinsim_edo_B.loop_ub_a] +=
            kinsim_edo_B.T[kinsim_edo_B.n_p + 1] *
            kinsim_edo_B.R_g1[kinsim_edo_B.ret_p + 3];
          kinsim_edo_B.R_m[kinsim_edo_B.loop_ub_a] +=
            kinsim_edo_B.T[kinsim_edo_B.n_p + 2] *
            kinsim_edo_B.R_g1[kinsim_edo_B.ret_p + 6];
          kinsim_edo_B.X[kinsim_edo_B.kstr + 6 * kinsim_edo_B.ret_p] =
            kinsim_edo_B.T[(kinsim_edo_B.ret_p << 2) + kinsim_edo_B.kstr];
          kinsim_edo_B.X[kinsim_edo_B.kstr + 6 * (kinsim_edo_B.ret_p + 3)] = 0.0;
        }
      }

      for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 3; kinsim_edo_B.ret_p++)
      {
        kinsim_edo_B.X[6 * kinsim_edo_B.ret_p + 3] = kinsim_edo_B.R_m[3 *
          kinsim_edo_B.ret_p];
        kinsim_edo_B.kstr = kinsim_edo_B.ret_p << 2;
        kinsim_edo_B.loop_ub_a = 6 * (kinsim_edo_B.ret_p + 3);
        kinsim_edo_B.X[kinsim_edo_B.loop_ub_a + 3] =
          kinsim_edo_B.T[kinsim_edo_B.kstr];
        kinsim_edo_B.X[6 * kinsim_edo_B.ret_p + 4] = kinsim_edo_B.R_m[3 *
          kinsim_edo_B.ret_p + 1];
        kinsim_edo_B.X[kinsim_edo_B.loop_ub_a + 4] =
          kinsim_edo_B.T[kinsim_edo_B.kstr + 1];
        kinsim_edo_B.X[6 * kinsim_edo_B.ret_p + 5] = kinsim_edo_B.R_m[3 *
          kinsim_edo_B.ret_p + 2];
        kinsim_edo_B.X[kinsim_edo_B.loop_ub_a + 5] =
          kinsim_edo_B.T[kinsim_edo_B.kstr + 2];
      }

      kinsim_edo_B.ret_p = b->size[0] * b->size[1];
      b->size[0] = 6;
      b->size[1] = body->JointInternal.MotionSubspace->size[1];
      kinsim_emxEnsureCapacity_real_T(b, kinsim_edo_B.ret_p);
      kinsim_edo_B.loop_ub_a = body->JointInternal.MotionSubspace->size[0] *
        body->JointInternal.MotionSubspace->size[1] - 1;
      for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p <= kinsim_edo_B.loop_ub_a;
           kinsim_edo_B.ret_p++) {
        b->data[kinsim_edo_B.ret_p] = body->JointInternal.MotionSubspace->
          data[kinsim_edo_B.ret_p];
      }

      kinsim_edo_B.n_p = b->size[1] - 1;
      kinsim_edo_B.ret_p = JacSlice->size[0] * JacSlice->size[1];
      JacSlice->size[0] = 6;
      JacSlice->size[1] = b->size[1];
      kinsim_emxEnsureCapacity_real_T(JacSlice, kinsim_edo_B.ret_p);
      for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p <= kinsim_edo_B.n_p;
           kinsim_edo_B.ret_p++) {
        kinsim_edo_B.coffset_tmp = kinsim_edo_B.ret_p * 6 - 1;
        for (kinsim_edo_B.kstr = 0; kinsim_edo_B.kstr < 6; kinsim_edo_B.kstr++)
        {
          kinsim_edo_B.s = 0.0;
          for (kinsim_edo_B.loop_ub_a = 0; kinsim_edo_B.loop_ub_a < 6;
               kinsim_edo_B.loop_ub_a++) {
            kinsim_edo_B.s += kinsim_edo_B.X[kinsim_edo_B.loop_ub_a * 6 +
              kinsim_edo_B.kstr] * b->data[(kinsim_edo_B.coffset_tmp +
              kinsim_edo_B.loop_ub_a) + 1];
          }

          JacSlice->data[(kinsim_edo_B.coffset_tmp + kinsim_edo_B.kstr) + 1] =
            kinsim_edo_B.s;
        }
      }

      if (kinsim_edo_B.endeffectorIndex > kinsim_edo_B.idx_idx_1) {
        kinsim_edo_B.n_p = 0;
      } else {
        kinsim_edo_B.n_p = static_cast<int32_T>(kinsim_edo_B.endeffectorIndex) -
          1;
      }

      kinsim_edo_B.loop_ub_a = JacSlice->size[1];
      for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < kinsim_edo_B.loop_ub_a;
           kinsim_edo_B.ret_p++) {
        for (kinsim_edo_B.kstr = 0; kinsim_edo_B.kstr < 6; kinsim_edo_B.kstr++)
        {
          Jac->data[kinsim_edo_B.kstr + 6 * (kinsim_edo_B.n_p +
            kinsim_edo_B.ret_p)] = JacSlice->data[6 * kinsim_edo_B.ret_p +
            kinsim_edo_B.kstr];
        }
      }
    }
  }

  kinsim_edo_emxFree_char_T(&bname);
  kinsim_edo_emxFree_real_T(&JacSlice);
  kinsim_edo_emxFree_f_cell_wrap(&Ttree);
  for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < 3; kinsim_edo_B.ret_p++) {
    kinsim_edo_B.b_i_c = kinsim_edo_B.ret_p << 2;
    kinsim_edo_B.X[6 * kinsim_edo_B.ret_p] = kinsim_edo_B.T2[kinsim_edo_B.b_i_c];
    kinsim_edo_B.kstr = 6 * (kinsim_edo_B.ret_p + 3);
    kinsim_edo_B.X[kinsim_edo_B.kstr] = 0.0;
    kinsim_edo_B.X[6 * kinsim_edo_B.ret_p + 3] = 0.0;
    kinsim_edo_B.X[kinsim_edo_B.kstr + 3] = kinsim_edo_B.T2[kinsim_edo_B.b_i_c];
    kinsim_edo_B.endeffectorIndex = kinsim_edo_B.T2[kinsim_edo_B.b_i_c + 1];
    kinsim_edo_B.X[6 * kinsim_edo_B.ret_p + 1] = kinsim_edo_B.endeffectorIndex;
    kinsim_edo_B.X[kinsim_edo_B.kstr + 1] = 0.0;
    kinsim_edo_B.X[6 * kinsim_edo_B.ret_p + 4] = 0.0;
    kinsim_edo_B.X[kinsim_edo_B.kstr + 4] = kinsim_edo_B.endeffectorIndex;
    kinsim_edo_B.endeffectorIndex = kinsim_edo_B.T2[kinsim_edo_B.b_i_c + 2];
    kinsim_edo_B.X[6 * kinsim_edo_B.ret_p + 2] = kinsim_edo_B.endeffectorIndex;
    kinsim_edo_B.X[kinsim_edo_B.kstr + 2] = 0.0;
    kinsim_edo_B.X[6 * kinsim_edo_B.ret_p + 5] = 0.0;
    kinsim_edo_B.X[kinsim_edo_B.kstr + 5] = kinsim_edo_B.endeffectorIndex;
  }

  kinsim_edo_B.n_p = Jac->size[1];
  kinsim_edo_B.ret_p = b->size[0] * b->size[1];
  b->size[0] = 6;
  b->size[1] = Jac->size[1];
  kinsim_emxEnsureCapacity_real_T(b, kinsim_edo_B.ret_p);
  kinsim_edo_B.loop_ub_a = Jac->size[0] * Jac->size[1] - 1;
  for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p <= kinsim_edo_B.loop_ub_a;
       kinsim_edo_B.ret_p++) {
    b->data[kinsim_edo_B.ret_p] = Jac->data[kinsim_edo_B.ret_p];
  }

  kinsim_edo_B.ret_p = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = kinsim_edo_B.n_p;
  kinsim_emxEnsureCapacity_real_T(Jac, kinsim_edo_B.ret_p);
  for (kinsim_edo_B.ret_p = 0; kinsim_edo_B.ret_p < kinsim_edo_B.n_p;
       kinsim_edo_B.ret_p++) {
    kinsim_edo_B.coffset_tmp = kinsim_edo_B.ret_p * 6 - 1;
    for (kinsim_edo_B.b_i_c = 0; kinsim_edo_B.b_i_c < 6; kinsim_edo_B.b_i_c++) {
      kinsim_edo_B.s = 0.0;
      for (kinsim_edo_B.loop_ub_a = 0; kinsim_edo_B.loop_ub_a < 6;
           kinsim_edo_B.loop_ub_a++) {
        kinsim_edo_B.s += kinsim_edo_B.X[kinsim_edo_B.loop_ub_a * 6 +
          kinsim_edo_B.b_i_c] * b->data[(kinsim_edo_B.coffset_tmp +
          kinsim_edo_B.loop_ub_a) + 1];
      }

      Jac->data[(kinsim_edo_B.coffset_tmp + kinsim_edo_B.b_i_c) + 1] =
        kinsim_edo_B.s;
    }
  }

  kinsim_edo_emxFree_real_T(&b);
}

static void rigidBodyJoint_get_JointAxis_p(const c_rigidBodyJoint_kinsim_edo_p_T
  *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (kinsim_edo_B.b_kstr_g = 0; kinsim_edo_B.b_kstr_g < 8;
       kinsim_edo_B.b_kstr_g++) {
    kinsim_edo_B.b_ln[kinsim_edo_B.b_kstr_g] = tmp[kinsim_edo_B.b_kstr_g];
  }

  kinsim_edo_B.b_bool_h3 = false;
  if (obj->Type->size[1] == 8) {
    kinsim_edo_B.b_kstr_g = 1;
    do {
      exitg1 = 0;
      if (kinsim_edo_B.b_kstr_g - 1 < 8) {
        kinsim_edo_B.kstr_f = kinsim_edo_B.b_kstr_g - 1;
        if (obj->Type->data[kinsim_edo_B.kstr_f] !=
            kinsim_edo_B.b_ln[kinsim_edo_B.kstr_f]) {
          exitg1 = 1;
        } else {
          kinsim_edo_B.b_kstr_g++;
        }
      } else {
        kinsim_edo_B.b_bool_h3 = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (kinsim_edo_B.b_bool_h3) {
    guard1 = true;
  } else {
    for (kinsim_edo_B.b_kstr_g = 0; kinsim_edo_B.b_kstr_g < 9;
         kinsim_edo_B.b_kstr_g++) {
      kinsim_edo_B.b_l[kinsim_edo_B.b_kstr_g] = tmp_0[kinsim_edo_B.b_kstr_g];
    }

    kinsim_edo_B.b_bool_h3 = false;
    if (obj->Type->size[1] == 9) {
      kinsim_edo_B.b_kstr_g = 1;
      do {
        exitg1 = 0;
        if (kinsim_edo_B.b_kstr_g - 1 < 9) {
          kinsim_edo_B.kstr_f = kinsim_edo_B.b_kstr_g - 1;
          if (obj->Type->data[kinsim_edo_B.kstr_f] !=
              kinsim_edo_B.b_l[kinsim_edo_B.kstr_f]) {
            exitg1 = 1;
          } else {
            kinsim_edo_B.b_kstr_g++;
          }
        } else {
          kinsim_edo_B.b_bool_h3 = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (kinsim_edo_B.b_bool_h3) {
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

static void RigidBodyTree_forwardKinemati_p(n_robotics_manip_internal_R_p_T *obj,
  const real_T qvec[3], emxArray_f_cell_wrap_kinsim_e_T *Ttree)
{
  l_robotics_manip_internal_R_p_T *body;
  emxArray_char_T_kinsim_edo_T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  kinsim_edo_B.n_b = obj->NumBodies;
  for (kinsim_edo_B.b_kstr_f = 0; kinsim_edo_B.b_kstr_f < 16;
       kinsim_edo_B.b_kstr_f++) {
    kinsim_edo_B.c_f1_c[kinsim_edo_B.b_kstr_f] = tmp[kinsim_edo_B.b_kstr_f];
  }

  kinsim_edo_B.b_kstr_f = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  kinsim_edo_B.e_o = static_cast<int32_T>(kinsim_edo_B.n_b);
  Ttree->size[1] = kinsim_edo_B.e_o;
  k_emxEnsureCapacity_f_cell_wrap(Ttree, kinsim_edo_B.b_kstr_f);
  if (kinsim_edo_B.e_o != 0) {
    kinsim_edo_B.ntilecols_o = kinsim_edo_B.e_o - 1;
    if (0 <= kinsim_edo_B.ntilecols_o) {
      memcpy(&kinsim_edo_B.expl_temp_m.f1[0], &kinsim_edo_B.c_f1_c[0], sizeof
             (real_T) << 4U);
    }

    for (kinsim_edo_B.b_jtilecol_i = 0; kinsim_edo_B.b_jtilecol_i <=
         kinsim_edo_B.ntilecols_o; kinsim_edo_B.b_jtilecol_i++) {
      Ttree->data[kinsim_edo_B.b_jtilecol_i] = kinsim_edo_B.expl_temp_m;
    }
  }

  kinsim_edo_B.k_j = 1.0;
  kinsim_edo_B.ntilecols_o = static_cast<int32_T>(kinsim_edo_B.n_b) - 1;
  kinsim_edo_emxInit_char_T(&switch_expression, 2);
  if (0 <= kinsim_edo_B.ntilecols_o) {
    for (kinsim_edo_B.b_kstr_f = 0; kinsim_edo_B.b_kstr_f < 5;
         kinsim_edo_B.b_kstr_f++) {
      kinsim_edo_B.b_h[kinsim_edo_B.b_kstr_f] = tmp_0[kinsim_edo_B.b_kstr_f];
    }
  }

  for (kinsim_edo_B.b_jtilecol_i = 0; kinsim_edo_B.b_jtilecol_i <=
       kinsim_edo_B.ntilecols_o; kinsim_edo_B.b_jtilecol_i++) {
    body = obj->Bodies[kinsim_edo_B.b_jtilecol_i];
    kinsim_edo_B.n_b = body->JointInternal.PositionNumber;
    kinsim_edo_B.n_b += kinsim_edo_B.k_j;
    if (kinsim_edo_B.k_j > kinsim_edo_B.n_b - 1.0) {
      kinsim_edo_B.e_o = 0;
      kinsim_edo_B.d_l = 0;
    } else {
      kinsim_edo_B.e_o = static_cast<int32_T>(kinsim_edo_B.k_j) - 1;
      kinsim_edo_B.d_l = static_cast<int32_T>(kinsim_edo_B.n_b - 1.0);
    }

    kinsim_edo_B.b_kstr_f = switch_expression->size[0] * switch_expression->
      size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    kinsim_emxEnsureCapacity_char_T(switch_expression, kinsim_edo_B.b_kstr_f);
    kinsim_edo_B.loop_ub_i = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (kinsim_edo_B.b_kstr_f = 0; kinsim_edo_B.b_kstr_f <=
         kinsim_edo_B.loop_ub_i; kinsim_edo_B.b_kstr_f++) {
      switch_expression->data[kinsim_edo_B.b_kstr_f] = body->
        JointInternal.Type->data[kinsim_edo_B.b_kstr_f];
    }

    kinsim_edo_B.b_bool_mc = false;
    if (switch_expression->size[1] == 5) {
      kinsim_edo_B.b_kstr_f = 1;
      do {
        exitg1 = 0;
        if (kinsim_edo_B.b_kstr_f - 1 < 5) {
          kinsim_edo_B.loop_ub_i = kinsim_edo_B.b_kstr_f - 1;
          if (switch_expression->data[kinsim_edo_B.loop_ub_i] !=
              kinsim_edo_B.b_h[kinsim_edo_B.loop_ub_i]) {
            exitg1 = 1;
          } else {
            kinsim_edo_B.b_kstr_f++;
          }
        } else {
          kinsim_edo_B.b_bool_mc = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (kinsim_edo_B.b_bool_mc) {
      kinsim_edo_B.b_kstr_f = 0;
    } else {
      for (kinsim_edo_B.b_kstr_f = 0; kinsim_edo_B.b_kstr_f < 8;
           kinsim_edo_B.b_kstr_f++) {
        kinsim_edo_B.b_bs[kinsim_edo_B.b_kstr_f] = tmp_1[kinsim_edo_B.b_kstr_f];
      }

      kinsim_edo_B.b_bool_mc = false;
      if (switch_expression->size[1] == 8) {
        kinsim_edo_B.b_kstr_f = 1;
        do {
          exitg1 = 0;
          if (kinsim_edo_B.b_kstr_f - 1 < 8) {
            kinsim_edo_B.loop_ub_i = kinsim_edo_B.b_kstr_f - 1;
            if (switch_expression->data[kinsim_edo_B.loop_ub_i] !=
                kinsim_edo_B.b_bs[kinsim_edo_B.loop_ub_i]) {
              exitg1 = 1;
            } else {
              kinsim_edo_B.b_kstr_f++;
            }
          } else {
            kinsim_edo_B.b_bool_mc = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (kinsim_edo_B.b_bool_mc) {
        kinsim_edo_B.b_kstr_f = 1;
      } else {
        kinsim_edo_B.b_kstr_f = -1;
      }
    }

    switch (kinsim_edo_B.b_kstr_f) {
     case 0:
      memset(&kinsim_edo_B.c_f1_c[0], 0, sizeof(real_T) << 4U);
      kinsim_edo_B.c_f1_c[0] = 1.0;
      kinsim_edo_B.c_f1_c[5] = 1.0;
      kinsim_edo_B.c_f1_c[10] = 1.0;
      kinsim_edo_B.c_f1_c[15] = 1.0;
      break;

     case 1:
      rigidBodyJoint_get_JointAxis_p(&body->JointInternal, kinsim_edo_B.v_g);
      kinsim_edo_B.d_l -= kinsim_edo_B.e_o;
      for (kinsim_edo_B.b_kstr_f = 0; kinsim_edo_B.b_kstr_f < kinsim_edo_B.d_l;
           kinsim_edo_B.b_kstr_f++) {
        kinsim_edo_B.e_data_d[kinsim_edo_B.b_kstr_f] = kinsim_edo_B.e_o +
          kinsim_edo_B.b_kstr_f;
      }

      kinsim_edo_B.result_data_j[0] = kinsim_edo_B.v_g[0];
      kinsim_edo_B.result_data_j[1] = kinsim_edo_B.v_g[1];
      kinsim_edo_B.result_data_j[2] = kinsim_edo_B.v_g[2];
      if (0 <= (kinsim_edo_B.d_l != 0) - 1) {
        kinsim_edo_B.result_data_j[3] = qvec[kinsim_edo_B.e_data_d[0]];
      }

      kinsim_edo_B.k_j = 1.0 / sqrt((kinsim_edo_B.result_data_j[0] *
        kinsim_edo_B.result_data_j[0] + kinsim_edo_B.result_data_j[1] *
        kinsim_edo_B.result_data_j[1]) + kinsim_edo_B.result_data_j[2] *
        kinsim_edo_B.result_data_j[2]);
      kinsim_edo_B.v_g[0] = kinsim_edo_B.result_data_j[0] * kinsim_edo_B.k_j;
      kinsim_edo_B.v_g[1] = kinsim_edo_B.result_data_j[1] * kinsim_edo_B.k_j;
      kinsim_edo_B.v_g[2] = kinsim_edo_B.result_data_j[2] * kinsim_edo_B.k_j;
      kinsim_edo_B.k_j = cos(kinsim_edo_B.result_data_j[3]);
      kinsim_edo_B.sth_f = sin(kinsim_edo_B.result_data_j[3]);
      kinsim_edo_B.tempR_l[0] = kinsim_edo_B.v_g[0] * kinsim_edo_B.v_g[0] * (1.0
        - kinsim_edo_B.k_j) + kinsim_edo_B.k_j;
      kinsim_edo_B.tempR_tmp_a = kinsim_edo_B.v_g[1] * kinsim_edo_B.v_g[0] *
        (1.0 - kinsim_edo_B.k_j);
      kinsim_edo_B.tempR_tmp_j = kinsim_edo_B.v_g[2] * kinsim_edo_B.sth_f;
      kinsim_edo_B.tempR_l[1] = kinsim_edo_B.tempR_tmp_a -
        kinsim_edo_B.tempR_tmp_j;
      kinsim_edo_B.tempR_tmp_jz = kinsim_edo_B.v_g[2] * kinsim_edo_B.v_g[0] *
        (1.0 - kinsim_edo_B.k_j);
      kinsim_edo_B.tempR_tmp_o = kinsim_edo_B.v_g[1] * kinsim_edo_B.sth_f;
      kinsim_edo_B.tempR_l[2] = kinsim_edo_B.tempR_tmp_jz +
        kinsim_edo_B.tempR_tmp_o;
      kinsim_edo_B.tempR_l[3] = kinsim_edo_B.tempR_tmp_a +
        kinsim_edo_B.tempR_tmp_j;
      kinsim_edo_B.tempR_l[4] = kinsim_edo_B.v_g[1] * kinsim_edo_B.v_g[1] * (1.0
        - kinsim_edo_B.k_j) + kinsim_edo_B.k_j;
      kinsim_edo_B.tempR_tmp_a = kinsim_edo_B.v_g[2] * kinsim_edo_B.v_g[1] *
        (1.0 - kinsim_edo_B.k_j);
      kinsim_edo_B.tempR_tmp_j = kinsim_edo_B.v_g[0] * kinsim_edo_B.sth_f;
      kinsim_edo_B.tempR_l[5] = kinsim_edo_B.tempR_tmp_a -
        kinsim_edo_B.tempR_tmp_j;
      kinsim_edo_B.tempR_l[6] = kinsim_edo_B.tempR_tmp_jz -
        kinsim_edo_B.tempR_tmp_o;
      kinsim_edo_B.tempR_l[7] = kinsim_edo_B.tempR_tmp_a +
        kinsim_edo_B.tempR_tmp_j;
      kinsim_edo_B.tempR_l[8] = kinsim_edo_B.v_g[2] * kinsim_edo_B.v_g[2] * (1.0
        - kinsim_edo_B.k_j) + kinsim_edo_B.k_j;
      for (kinsim_edo_B.b_kstr_f = 0; kinsim_edo_B.b_kstr_f < 3;
           kinsim_edo_B.b_kstr_f++) {
        kinsim_edo_B.e_o = kinsim_edo_B.b_kstr_f + 1;
        kinsim_edo_B.R_p[kinsim_edo_B.e_o - 1] = kinsim_edo_B.tempR_l
          [(kinsim_edo_B.e_o - 1) * 3];
        kinsim_edo_B.e_o = kinsim_edo_B.b_kstr_f + 1;
        kinsim_edo_B.R_p[kinsim_edo_B.e_o + 2] = kinsim_edo_B.tempR_l
          [(kinsim_edo_B.e_o - 1) * 3 + 1];
        kinsim_edo_B.e_o = kinsim_edo_B.b_kstr_f + 1;
        kinsim_edo_B.R_p[kinsim_edo_B.e_o + 5] = kinsim_edo_B.tempR_l
          [(kinsim_edo_B.e_o - 1) * 3 + 2];
      }

      memset(&kinsim_edo_B.c_f1_c[0], 0, sizeof(real_T) << 4U);
      for (kinsim_edo_B.b_kstr_f = 0; kinsim_edo_B.b_kstr_f < 3;
           kinsim_edo_B.b_kstr_f++) {
        kinsim_edo_B.d_l = kinsim_edo_B.b_kstr_f << 2;
        kinsim_edo_B.c_f1_c[kinsim_edo_B.d_l] = kinsim_edo_B.R_p[3 *
          kinsim_edo_B.b_kstr_f];
        kinsim_edo_B.c_f1_c[kinsim_edo_B.d_l + 1] = kinsim_edo_B.R_p[3 *
          kinsim_edo_B.b_kstr_f + 1];
        kinsim_edo_B.c_f1_c[kinsim_edo_B.d_l + 2] = kinsim_edo_B.R_p[3 *
          kinsim_edo_B.b_kstr_f + 2];
      }

      kinsim_edo_B.c_f1_c[15] = 1.0;
      break;

     default:
      rigidBodyJoint_get_JointAxis_p(&body->JointInternal, kinsim_edo_B.v_g);
      memset(&kinsim_edo_B.tempR_l[0], 0, 9U * sizeof(real_T));
      kinsim_edo_B.tempR_l[0] = 1.0;
      kinsim_edo_B.tempR_l[4] = 1.0;
      kinsim_edo_B.tempR_l[8] = 1.0;
      for (kinsim_edo_B.b_kstr_f = 0; kinsim_edo_B.b_kstr_f < 3;
           kinsim_edo_B.b_kstr_f++) {
        kinsim_edo_B.d_l = kinsim_edo_B.b_kstr_f << 2;
        kinsim_edo_B.c_f1_c[kinsim_edo_B.d_l] = kinsim_edo_B.tempR_l[3 *
          kinsim_edo_B.b_kstr_f];
        kinsim_edo_B.c_f1_c[kinsim_edo_B.d_l + 1] = kinsim_edo_B.tempR_l[3 *
          kinsim_edo_B.b_kstr_f + 1];
        kinsim_edo_B.c_f1_c[kinsim_edo_B.d_l + 2] = kinsim_edo_B.tempR_l[3 *
          kinsim_edo_B.b_kstr_f + 2];
        kinsim_edo_B.c_f1_c[kinsim_edo_B.b_kstr_f + 12] =
          kinsim_edo_B.v_g[kinsim_edo_B.b_kstr_f] * qvec[kinsim_edo_B.e_o];
      }

      kinsim_edo_B.c_f1_c[3] = 0.0;
      kinsim_edo_B.c_f1_c[7] = 0.0;
      kinsim_edo_B.c_f1_c[11] = 0.0;
      kinsim_edo_B.c_f1_c[15] = 1.0;
      break;
    }

    for (kinsim_edo_B.b_kstr_f = 0; kinsim_edo_B.b_kstr_f < 16;
         kinsim_edo_B.b_kstr_f++) {
      kinsim_edo_B.a_b[kinsim_edo_B.b_kstr_f] =
        body->JointInternal.JointToParentTransform[kinsim_edo_B.b_kstr_f];
    }

    for (kinsim_edo_B.b_kstr_f = 0; kinsim_edo_B.b_kstr_f < 16;
         kinsim_edo_B.b_kstr_f++) {
      kinsim_edo_B.b_p[kinsim_edo_B.b_kstr_f] =
        body->JointInternal.ChildToJointTransform[kinsim_edo_B.b_kstr_f];
    }

    for (kinsim_edo_B.b_kstr_f = 0; kinsim_edo_B.b_kstr_f < 4;
         kinsim_edo_B.b_kstr_f++) {
      for (kinsim_edo_B.e_o = 0; kinsim_edo_B.e_o < 4; kinsim_edo_B.e_o++) {
        kinsim_edo_B.d_l = kinsim_edo_B.e_o << 2;
        kinsim_edo_B.loop_ub_i = kinsim_edo_B.b_kstr_f + kinsim_edo_B.d_l;
        kinsim_edo_B.a_c[kinsim_edo_B.loop_ub_i] = 0.0;
        kinsim_edo_B.a_c[kinsim_edo_B.loop_ub_i] +=
          kinsim_edo_B.c_f1_c[kinsim_edo_B.d_l] *
          kinsim_edo_B.a_b[kinsim_edo_B.b_kstr_f];
        kinsim_edo_B.a_c[kinsim_edo_B.loop_ub_i] +=
          kinsim_edo_B.c_f1_c[kinsim_edo_B.d_l + 1] *
          kinsim_edo_B.a_b[kinsim_edo_B.b_kstr_f + 4];
        kinsim_edo_B.a_c[kinsim_edo_B.loop_ub_i] +=
          kinsim_edo_B.c_f1_c[kinsim_edo_B.d_l + 2] *
          kinsim_edo_B.a_b[kinsim_edo_B.b_kstr_f + 8];
        kinsim_edo_B.a_c[kinsim_edo_B.loop_ub_i] +=
          kinsim_edo_B.c_f1_c[kinsim_edo_B.d_l + 3] *
          kinsim_edo_B.a_b[kinsim_edo_B.b_kstr_f + 12];
      }

      for (kinsim_edo_B.e_o = 0; kinsim_edo_B.e_o < 4; kinsim_edo_B.e_o++) {
        kinsim_edo_B.d_l = kinsim_edo_B.e_o << 2;
        kinsim_edo_B.loop_ub_i = kinsim_edo_B.b_kstr_f + kinsim_edo_B.d_l;
        Ttree->data[kinsim_edo_B.b_jtilecol_i].f1[kinsim_edo_B.loop_ub_i] = 0.0;
        Ttree->data[kinsim_edo_B.b_jtilecol_i].f1[kinsim_edo_B.loop_ub_i] +=
          kinsim_edo_B.b_p[kinsim_edo_B.d_l] *
          kinsim_edo_B.a_c[kinsim_edo_B.b_kstr_f];
        Ttree->data[kinsim_edo_B.b_jtilecol_i].f1[kinsim_edo_B.loop_ub_i] +=
          kinsim_edo_B.b_p[kinsim_edo_B.d_l + 1] *
          kinsim_edo_B.a_c[kinsim_edo_B.b_kstr_f + 4];
        Ttree->data[kinsim_edo_B.b_jtilecol_i].f1[kinsim_edo_B.loop_ub_i] +=
          kinsim_edo_B.b_p[kinsim_edo_B.d_l + 2] *
          kinsim_edo_B.a_c[kinsim_edo_B.b_kstr_f + 8];
        Ttree->data[kinsim_edo_B.b_jtilecol_i].f1[kinsim_edo_B.loop_ub_i] +=
          kinsim_edo_B.b_p[kinsim_edo_B.d_l + 3] *
          kinsim_edo_B.a_c[kinsim_edo_B.b_kstr_f + 12];
      }
    }

    kinsim_edo_B.k_j = kinsim_edo_B.n_b;
    if (body->ParentIndex > 0.0) {
      for (kinsim_edo_B.b_kstr_f = 0; kinsim_edo_B.b_kstr_f < 16;
           kinsim_edo_B.b_kstr_f++) {
        kinsim_edo_B.a_b[kinsim_edo_B.b_kstr_f] = Ttree->data
          [static_cast<int32_T>(body->ParentIndex) - 1].f1[kinsim_edo_B.b_kstr_f];
      }

      for (kinsim_edo_B.b_kstr_f = 0; kinsim_edo_B.b_kstr_f < 4;
           kinsim_edo_B.b_kstr_f++) {
        for (kinsim_edo_B.e_o = 0; kinsim_edo_B.e_o < 4; kinsim_edo_B.e_o++) {
          kinsim_edo_B.d_l = kinsim_edo_B.e_o << 2;
          kinsim_edo_B.loop_ub_i = kinsim_edo_B.b_kstr_f + kinsim_edo_B.d_l;
          kinsim_edo_B.a_c[kinsim_edo_B.loop_ub_i] = 0.0;
          kinsim_edo_B.a_c[kinsim_edo_B.loop_ub_i] += Ttree->
            data[kinsim_edo_B.b_jtilecol_i].f1[kinsim_edo_B.d_l] *
            kinsim_edo_B.a_b[kinsim_edo_B.b_kstr_f];
          kinsim_edo_B.a_c[kinsim_edo_B.loop_ub_i] += Ttree->
            data[kinsim_edo_B.b_jtilecol_i].f1[kinsim_edo_B.d_l + 1] *
            kinsim_edo_B.a_b[kinsim_edo_B.b_kstr_f + 4];
          kinsim_edo_B.a_c[kinsim_edo_B.loop_ub_i] += Ttree->
            data[kinsim_edo_B.b_jtilecol_i].f1[kinsim_edo_B.d_l + 2] *
            kinsim_edo_B.a_b[kinsim_edo_B.b_kstr_f + 8];
          kinsim_edo_B.a_c[kinsim_edo_B.loop_ub_i] += Ttree->
            data[kinsim_edo_B.b_jtilecol_i].f1[kinsim_edo_B.d_l + 3] *
            kinsim_edo_B.a_b[kinsim_edo_B.b_kstr_f + 12];
        }
      }

      for (kinsim_edo_B.b_kstr_f = 0; kinsim_edo_B.b_kstr_f < 16;
           kinsim_edo_B.b_kstr_f++) {
        Ttree->data[kinsim_edo_B.b_jtilecol_i].f1[kinsim_edo_B.b_kstr_f] =
          kinsim_edo_B.a_c[kinsim_edo_B.b_kstr_f];
      }
    }
  }

  kinsim_edo_emxFree_char_T(&switch_expression);
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

static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_kinsim_edo_T
  *pStruct)
{
  kinsim_edo_emxFree_char_T(&pStruct->Type);
  kinsim_edo_emxFree_real_T(&pStruct->MotionSubspace);
}

static void emxFreeStruct_m_robotics_manip_(m_robotics_manip_internal_Rig_T
  *pStruct)
{
  kinsim_edo_emxFree_char_T(&pStruct->NameInternal);
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

static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_kinsim_edo_p_T
  *pStruct)
{
  kinsim_edo_emxFree_char_T(&pStruct->Type);
}

static void emxFreeStruct_m_robotics_mani_p(m_robotics_manip_internal_R_p_T
  *pStruct)
{
  kinsim_edo_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxFreeStruct_n_robotics_mani_p(n_robotics_manip_internal_R_p_T
  *pStruct)
{
  emxFreeStruct_m_robotics_mani_p(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmani_p(robotics_slmanip_internal_b_p_T
  *pStruct)
{
  emxFreeStruct_n_robotics_mani_p(&pStruct->TreeInternal);
}

static void emxFreeStruct_l_robotics_mani_p(l_robotics_manip_internal_R_p_T
  *pStruct)
{
  emxFreeStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
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

static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_kinsim_edo_T
  *pStruct)
{
  kinsim_edo_emxInit_char_T(&pStruct->Type, 2);
  kinsim_edo_emxInit_real_T(&pStruct->MotionSubspace, 2);
}

static void emxInitStruct_m_robotics_manip_(m_robotics_manip_internal_Rig_T
  *pStruct)
{
  kinsim_edo_emxInit_char_T(&pStruct->NameInternal, 2);
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

static l_robotics_manip_internal_Rig_T *kinsim_edo_RigidBody_RigidBody
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinsim_edo_T *switch_expression;
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
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinsim_edo_emxFree_char_T(&switch_expression);
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
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *kinsim_ed_RigidBody_RigidBody_p
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinsim_edo_T *switch_expression;
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
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinsim_edo_emxFree_char_T(&switch_expression);
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
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = -0.88847134048214615;
  obj->JointInternal.JointAxisInternal[1] = 0.29080043874549294;
  obj->JointInternal.JointAxisInternal[2] = 0.3550405356678123;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *kinsim_e_RigidBody_RigidBody_pc
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinsim_edo_T *switch_expression;
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
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinsim_edo_emxFree_char_T(&switch_expression);
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
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *kinsim__RigidBody_RigidBody_pcd
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinsim_edo_T *switch_expression;
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
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinsim_edo_emxFree_char_T(&switch_expression);
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
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *kinsim_RigidBody_RigidBody_pcdk
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinsim_edo_T *switch_expression;
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
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinsim_edo_emxFree_char_T(&switch_expression);
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
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *kinsi_RigidBody_RigidBody_pcdkl
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinsim_edo_T *switch_expression;
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
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinsim_edo_emxFree_char_T(&switch_expression);
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
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static m_robotics_manip_internal_Rig_T *kins_RigidBody_RigidBody_pcdklh
  (m_robotics_manip_internal_Rig_T *obj)
{
  m_robotics_manip_internal_Rig_T *b_obj;
  emxArray_char_T_kinsim_edo_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
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
  kinsim_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
      kinsim_edo_B.b_o[b_kstr] = tmp_3[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != kinsim_edo_B.b_o[loop_ub]) {
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

  kinsim_edo_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      kinsim_edo_B.msubspace_data[b_kstr] = tmp[b_kstr];
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
      kinsim_edo_B.msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      kinsim_edo_B.msubspace_data[b_kstr] = 0;
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
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] =
      kinsim_edo_B.msubspace_data[b_kstr];
  }

  return b_obj;
}

static n_robotics_manip_internal_Rig_T *kin_RigidBodyTree_RigidBodyTree
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
  obj->Bodies[0] = kinsim_edo_RigidBody_RigidBody(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = kinsim_ed_RigidBody_RigidBody_p(iobj_5);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = kinsim_e_RigidBody_RigidBody_pc(iobj_1);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = kinsim__RigidBody_RigidBody_pcd(iobj_2);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = kinsim_RigidBody_RigidBody_pcdk(iobj_3);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = kinsi_RigidBody_RigidBody_pcdkl(iobj_4);
  obj->Bodies[5]->Index = 6.0;
  obj->NumBodies = 6.0;
  obj->PositionNumber = 3.0;
  obj->VelocityNumber = 3.0;
  for (i = 0; i < 12; i++) {
    obj->PositionDoFMap[i] = tmp[i];
  }

  kins_RigidBody_RigidBody_pcdklh(&obj->Base);
  return b_obj;
}

static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_kinsim_edo_p_T
  *pStruct)
{
  kinsim_edo_emxInit_char_T(&pStruct->Type, 2);
}

static void emxInitStruct_m_robotics_mani_p(m_robotics_manip_internal_R_p_T
  *pStruct)
{
  kinsim_edo_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxInitStruct_n_robotics_mani_p(n_robotics_manip_internal_R_p_T
  *pStruct)
{
  emxInitStruct_m_robotics_mani_p(&pStruct->Base);
}

static void emxInitStruct_robotics_slmani_p(robotics_slmanip_internal_b_p_T
  *pStruct)
{
  emxInitStruct_n_robotics_mani_p(&pStruct->TreeInternal);
}

static void emxInitStruct_l_robotics_mani_p(l_robotics_manip_internal_R_p_T
  *pStruct)
{
  emxInitStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static l_robotics_manip_internal_R_p_T *kin_RigidBody_RigidBody_pcdklhm
  (l_robotics_manip_internal_R_p_T *obj)
{
  l_robotics_manip_internal_R_p_T *b_obj;
  emxArray_char_T_kinsim_edo_T *switch_expression;
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
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  kinsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinsim_edo_emxFree_char_T(&switch_expression);
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

static l_robotics_manip_internal_R_p_T *ki_RigidBody_RigidBody_pcdklhm5
  (l_robotics_manip_internal_R_p_T *obj)
{
  l_robotics_manip_internal_R_p_T *b_obj;
  emxArray_char_T_kinsim_edo_T *switch_expression;
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
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  kinsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinsim_edo_emxFree_char_T(&switch_expression);
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

static l_robotics_manip_internal_R_p_T *k_RigidBody_RigidBody_pcdklhm5m
  (l_robotics_manip_internal_R_p_T *obj)
{
  l_robotics_manip_internal_R_p_T *b_obj;
  emxArray_char_T_kinsim_edo_T *switch_expression;
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
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  kinsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinsim_edo_emxFree_char_T(&switch_expression);
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

static n_robotics_manip_internal_R_p_T *k_RigidBodyTree_RigidBodyTree_p
  (n_robotics_manip_internal_R_p_T *obj, l_robotics_manip_internal_R_p_T *iobj_0,
   l_robotics_manip_internal_R_p_T *iobj_1, l_robotics_manip_internal_R_p_T
   *iobj_2, l_robotics_manip_internal_R_p_T *iobj_3,
   l_robotics_manip_internal_R_p_T *iobj_4, l_robotics_manip_internal_R_p_T
   *iobj_5)
{
  n_robotics_manip_internal_R_p_T *b_obj;
  m_robotics_manip_internal_R_p_T *obj_0;
  emxArray_char_T_kinsim_edo_T *switch_expression;
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
  obj->Bodies[0] = kin_RigidBody_RigidBody_pcdklhm(iobj_0);
  obj->Bodies[1] = ki_RigidBody_RigidBody_pcdklhm5(iobj_5);
  obj->Bodies[2] = k_RigidBody_RigidBody_pcdklhm5m(iobj_1);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_2->NameInternal[b_kstr] = tmp[b_kstr];
  }

  iobj_2->ParentIndex = 3.0;
  b_kstr = iobj_2->JointInternal.Type->size[0] * iobj_2->
    JointInternal.Type->size[1];
  iobj_2->JointInternal.Type->size[0] = 1;
  iobj_2->JointInternal.Type->size[1] = 5;
  kinsim_emxEnsureCapacity_char_T(iobj_2->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_2->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  kinsim_edo_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_2->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  kinsim_emxEnsureCapacity_char_T(iobj_3->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_3->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_3->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  kinsim_emxEnsureCapacity_char_T(iobj_4->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_4->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_4->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  kinsim_emxEnsureCapacity_char_T(obj->Base.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->Base.NameInternal->data[b_kstr] = tmp_9[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1];
  obj->Base.JointInternal.Type->size[0] = 1;
  obj->Base.JointInternal.Type->size[1] = 5;
  kinsim_emxEnsureCapacity_char_T(obj->Base.JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj_0->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Base.JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinsim_edo_emxFree_char_T(&switch_expression);
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

// Model step function
void kinsim_edo_step(void)
{
  emxArray_real_T_kinsim_edo_T *b;
  robotics_slmanip_internal_b_p_T *obj;
  n_robotics_manip_internal_R_p_T *obj_0;
  emxArray_f_cell_wrap_kinsim_e_T *Ttree;
  emxArray_char_T_kinsim_edo_T *bname;
  l_robotics_manip_internal_R_p_T *obj_1;
  static const char_T tmp[6] = { 'l', 'i', 'n', 'k', '_', '4' };

  static const char_T e[6] = { 'x', '_', 'c', 'o', 'o', 'r' };

  static const char_T f[6] = { 'y', '_', 'c', 'o', 'o', 'r' };

  static const char_T g[6] = { 'z', '_', 'c', 'o', 'o', 'r' };

  static const char_T e_0[7] = { 'j', 'o', 'i', 'n', 't', '_', '1' };

  static const char_T f_0[7] = { 'j', 'o', 'i', 'n', 't', '_', '2' };

  static const char_T g_0[7] = { 'j', 'o', 'i', 'n', 't', '_', '3' };

  int32_T exitg1;
  boolean_T exitg2;
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

  // MATLABSystem: '<S13>/Get Parameter'
  ParamGet_kinsim_edo_61.get_parameter(&kinsim_edo_B.bid1);

  // MATLABSystem: '<S13>/Get Parameter1'
  ParamGet_kinsim_edo_65.get_parameter(&kinsim_edo_B.value);

  // MATLABSystem: '<S13>/Get Parameter5'
  ParamGet_kinsim_edo_84.get_parameter(&kinsim_edo_B.value_h);

  // Integrator: '<Root>/Integrator' incorporates:
  //   MATLABSystem: '<S13>/Get Parameter'
  //   MATLABSystem: '<S13>/Get Parameter1'
  //   MATLABSystem: '<S13>/Get Parameter5'

  if (kinsim_edo_DW.Integrator_IWORK != 0) {
    kinsim_edo_X.Integrator_CSTATE[0] = kinsim_edo_B.bid1;
    kinsim_edo_X.Integrator_CSTATE[1] = kinsim_edo_B.value;
    kinsim_edo_X.Integrator_CSTATE[2] = kinsim_edo_B.value_h;
  }

  kinsim_edo_emxInit_real_T(&b, 2);
  kinsim_edo_emxInit_f_cell_wrap(&Ttree, 2);
  kinsim_edo_emxInit_char_T(&bname, 2);

  // MATLABSystem: '<S4>/MATLAB System' incorporates:
  //   Integrator: '<Root>/Integrator'

  RigidBodyTree_geometricJacobian(&kinsim_edo_DW.obj.TreeInternal,
    kinsim_edo_X.Integrator_CSTATE, b);

  // MATLABSystem: '<S5>/MATLAB System' incorporates:
  //   Integrator: '<Root>/Integrator'

  obj = &kinsim_edo_DW.obj_i;
  obj_0 = &kinsim_edo_DW.obj_i.TreeInternal;
  RigidBodyTree_forwardKinemati_p(&obj->TreeInternal,
    kinsim_edo_X.Integrator_CSTATE, Ttree);
  kinsim_edo_B.bid1 = -1.0;
  kinsim_edo_B.ret = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj_0->Base.NameInternal->size[1];
  kinsim_emxEnsureCapacity_char_T(bname, kinsim_edo_B.ret);
  kinsim_edo_B.loop_ub = obj_0->Base.NameInternal->size[0] *
    obj_0->Base.NameInternal->size[1] - 1;
  for (kinsim_edo_B.ret = 0; kinsim_edo_B.ret <= kinsim_edo_B.loop_ub;
       kinsim_edo_B.ret++) {
    bname->data[kinsim_edo_B.ret] = obj_0->Base.NameInternal->
      data[kinsim_edo_B.ret];
  }

  for (kinsim_edo_B.ret = 0; kinsim_edo_B.ret < 6; kinsim_edo_B.ret++) {
    kinsim_edo_B.b_m[kinsim_edo_B.ret] = tmp[kinsim_edo_B.ret];
  }

  kinsim_edo_B.b_bool = false;
  if (bname->size[1] == 6) {
    kinsim_edo_B.ret = 1;
    do {
      exitg1 = 0;
      if (kinsim_edo_B.ret - 1 < 6) {
        kinsim_edo_B.loop_ub = kinsim_edo_B.ret - 1;
        if (bname->data[kinsim_edo_B.loop_ub] !=
            kinsim_edo_B.b_m[kinsim_edo_B.loop_ub]) {
          exitg1 = 1;
        } else {
          kinsim_edo_B.ret++;
        }
      } else {
        kinsim_edo_B.b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  kinsim_edo_emxFree_char_T(&bname);

  // MATLABSystem: '<S5>/MATLAB System'
  if (kinsim_edo_B.b_bool) {
    kinsim_edo_B.bid1 = 0.0;
  } else {
    kinsim_edo_B.value = obj->TreeInternal.NumBodies;
    kinsim_edo_B.loop_ub = 0;
    exitg2 = false;
    while ((!exitg2) && (kinsim_edo_B.loop_ub <= static_cast<int32_T>
                         (kinsim_edo_B.value) - 1)) {
      obj_1 = obj_0->Bodies[kinsim_edo_B.loop_ub];
      for (kinsim_edo_B.ret = 0; kinsim_edo_B.ret < 6; kinsim_edo_B.ret++) {
        kinsim_edo_B.bname[kinsim_edo_B.ret] = obj_1->
          NameInternal[kinsim_edo_B.ret];
      }

      for (kinsim_edo_B.ret = 0; kinsim_edo_B.ret < 6; kinsim_edo_B.ret++) {
        kinsim_edo_B.b_m[kinsim_edo_B.ret] = tmp[kinsim_edo_B.ret];
      }

      kinsim_edo_B.ret = memcmp(&kinsim_edo_B.bname[0], &kinsim_edo_B.b_m[0], 6);
      if (kinsim_edo_B.ret == 0) {
        kinsim_edo_B.bid1 = static_cast<real_T>(kinsim_edo_B.loop_ub) + 1.0;
        exitg2 = true;
      } else {
        kinsim_edo_B.loop_ub++;
      }
    }
  }

  if (kinsim_edo_B.bid1 == 0.0) {
    memset(&kinsim_edo_B.T1[0], 0, sizeof(real_T) << 4U);
    kinsim_edo_B.T1[0] = 1.0;
    kinsim_edo_B.T1[5] = 1.0;
    kinsim_edo_B.T1[10] = 1.0;
    kinsim_edo_B.T1[15] = 1.0;
  } else {
    for (kinsim_edo_B.ret = 0; kinsim_edo_B.ret < 16; kinsim_edo_B.ret++) {
      kinsim_edo_B.T1[kinsim_edo_B.ret] = Ttree->data[static_cast<int32_T>
        (kinsim_edo_B.bid1) - 1].f1[kinsim_edo_B.ret];
    }
  }

  kinsim_edo_emxFree_f_cell_wrap(&Ttree);

  // MATLABSystem: '<S5>/MATLAB System'
  for (kinsim_edo_B.ret = 0; kinsim_edo_B.ret < 16; kinsim_edo_B.ret++) {
    kinsim_edo_B.T2_l[kinsim_edo_B.ret] = 0;
  }

  kinsim_edo_B.T2_l[0] = 1;
  kinsim_edo_B.T2_l[5] = 1;
  kinsim_edo_B.T2_l[10] = 1;
  kinsim_edo_B.T2_l[15] = 1;
  for (kinsim_edo_B.ret = 0; kinsim_edo_B.ret < 3; kinsim_edo_B.ret++) {
    kinsim_edo_B.R_f[3 * kinsim_edo_B.ret] = kinsim_edo_B.T2_l[kinsim_edo_B.ret];
    kinsim_edo_B.R_f[3 * kinsim_edo_B.ret + 1] =
      kinsim_edo_B.T2_l[kinsim_edo_B.ret + 4];
    kinsim_edo_B.R_f[3 * kinsim_edo_B.ret + 2] =
      kinsim_edo_B.T2_l[kinsim_edo_B.ret + 8];
  }

  for (kinsim_edo_B.ret = 0; kinsim_edo_B.ret < 9; kinsim_edo_B.ret++) {
    kinsim_edo_B.R_g[kinsim_edo_B.ret] = -kinsim_edo_B.R_f[kinsim_edo_B.ret];
  }

  for (kinsim_edo_B.ret = 0; kinsim_edo_B.ret < 3; kinsim_edo_B.ret++) {
    kinsim_edo_B.loop_ub = kinsim_edo_B.ret << 2;
    kinsim_edo_B.R[kinsim_edo_B.loop_ub] = kinsim_edo_B.R_f[3 * kinsim_edo_B.ret];
    kinsim_edo_B.R[kinsim_edo_B.loop_ub + 1] = kinsim_edo_B.R_f[3 *
      kinsim_edo_B.ret + 1];
    kinsim_edo_B.R[kinsim_edo_B.loop_ub + 2] = kinsim_edo_B.R_f[3 *
      kinsim_edo_B.ret + 2];
    kinsim_edo_B.R[kinsim_edo_B.ret + 12] = kinsim_edo_B.R_g[kinsim_edo_B.ret +
      6] * static_cast<real_T>(kinsim_edo_B.T2_l[14]) +
      (kinsim_edo_B.R_g[kinsim_edo_B.ret + 3] * static_cast<real_T>
       (kinsim_edo_B.T2_l[13]) + kinsim_edo_B.R_g[kinsim_edo_B.ret] *
       static_cast<real_T>(kinsim_edo_B.T2_l[12]));
  }

  kinsim_edo_B.R[3] = 0.0;
  kinsim_edo_B.R[7] = 0.0;
  kinsim_edo_B.R[11] = 0.0;
  kinsim_edo_B.R[15] = 1.0;
  for (kinsim_edo_B.ret = 0; kinsim_edo_B.ret < 4; kinsim_edo_B.ret++) {
    for (kinsim_edo_B.loop_ub = 0; kinsim_edo_B.loop_ub < 4;
         kinsim_edo_B.loop_ub++) {
      kinsim_edo_B.rtb_MATLABSystem_tmp_tmp = kinsim_edo_B.ret << 2;
      kinsim_edo_B.rtb_MATLABSystem_tmp = kinsim_edo_B.loop_ub +
        kinsim_edo_B.rtb_MATLABSystem_tmp_tmp;
      kinsim_edo_B.MATLABSystem[kinsim_edo_B.rtb_MATLABSystem_tmp] = 0.0;
      kinsim_edo_B.MATLABSystem[kinsim_edo_B.rtb_MATLABSystem_tmp] +=
        kinsim_edo_B.T1[kinsim_edo_B.rtb_MATLABSystem_tmp_tmp] *
        kinsim_edo_B.R[kinsim_edo_B.loop_ub];
      kinsim_edo_B.MATLABSystem[kinsim_edo_B.rtb_MATLABSystem_tmp] +=
        kinsim_edo_B.T1[kinsim_edo_B.rtb_MATLABSystem_tmp_tmp + 1] *
        kinsim_edo_B.R[kinsim_edo_B.loop_ub + 4];
      kinsim_edo_B.MATLABSystem[kinsim_edo_B.rtb_MATLABSystem_tmp] +=
        kinsim_edo_B.T1[kinsim_edo_B.rtb_MATLABSystem_tmp_tmp + 2] *
        kinsim_edo_B.R[kinsim_edo_B.loop_ub + 8];
      kinsim_edo_B.MATLABSystem[kinsim_edo_B.rtb_MATLABSystem_tmp] +=
        kinsim_edo_B.T1[kinsim_edo_B.rtb_MATLABSystem_tmp_tmp + 3] *
        kinsim_edo_B.R[kinsim_edo_B.loop_ub + 12];
    }
  }

  if (rtmIsMajorTimeStep(kinsim_edo_M)) {
    // Outputs for Atomic SubSystem: '<Root>/Subscribe'
    // MATLABSystem: '<S12>/SourceBlock' incorporates:
    //   Inport: '<S14>/In1'

    kinsim_edo_SystemCore_step(&kinsim_edo_B.b_bool,
      kinsim_edo_B.b_varargout_2_Positions,
      &kinsim_edo_B.b_varargout_2_Positions_SL_Info,
      &kinsim_edo_B.b_varargout_2_Positions_SL_In_m,
      kinsim_edo_B.b_varargout_2_Velocities,
      &kinsim_edo_B.b_varargout_2_Velocities_SL_Inf,
      &kinsim_edo_B.b_varargout_2_Velocities_SL_I_f,
      kinsim_edo_B.b_varargout_2_Accelerations,
      &kinsim_edo_B.b_varargout_2_Accelerations_SL_,
      &kinsim_edo_B.b_varargout_2_Accelerations_S_e,
      kinsim_edo_B.b_varargout_2_Effort,
      &kinsim_edo_B.b_varargout_2_Effort_SL_Info_Cu,
      &kinsim_edo_B.b_varargout_2_Effort_SL_Info_Re, &kinsim_edo_B.bid1,
      &kinsim_edo_B.value);

    // Outputs for Enabled SubSystem: '<S12>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S14>/Enable'

    if (kinsim_edo_B.b_bool) {
      kinsim_edo_B.In1.Positions_SL_Info.CurrentLength =
        kinsim_edo_B.b_varargout_2_Positions_SL_Info;
      kinsim_edo_B.In1.Positions_SL_Info.ReceivedLength =
        kinsim_edo_B.b_varargout_2_Positions_SL_In_m;
      kinsim_edo_B.In1.Velocities_SL_Info.CurrentLength =
        kinsim_edo_B.b_varargout_2_Velocities_SL_Inf;
      kinsim_edo_B.In1.Velocities_SL_Info.ReceivedLength =
        kinsim_edo_B.b_varargout_2_Velocities_SL_I_f;
      kinsim_edo_B.In1.Accelerations_SL_Info.CurrentLength =
        kinsim_edo_B.b_varargout_2_Accelerations_SL_;
      kinsim_edo_B.In1.Accelerations_SL_Info.ReceivedLength =
        kinsim_edo_B.b_varargout_2_Accelerations_S_e;
      memcpy(&kinsim_edo_B.In1.Positions[0],
             &kinsim_edo_B.b_varargout_2_Positions[0], sizeof(real_T) << 7U);
      memcpy(&kinsim_edo_B.In1.Velocities[0],
             &kinsim_edo_B.b_varargout_2_Velocities[0], sizeof(real_T) << 7U);
      memcpy(&kinsim_edo_B.In1.Accelerations[0],
             &kinsim_edo_B.b_varargout_2_Accelerations[0], sizeof(real_T) << 7U);
      memcpy(&kinsim_edo_B.In1.Effort[0], &kinsim_edo_B.b_varargout_2_Effort[0],
             sizeof(real_T) << 7U);
      kinsim_edo_B.In1.Effort_SL_Info.CurrentLength =
        kinsim_edo_B.b_varargout_2_Effort_SL_Info_Cu;
      kinsim_edo_B.In1.Effort_SL_Info.ReceivedLength =
        kinsim_edo_B.b_varargout_2_Effort_SL_Info_Re;
      kinsim_edo_B.In1.TimeFromStart.Sec = kinsim_edo_B.bid1;
      kinsim_edo_B.In1.TimeFromStart.Nsec = kinsim_edo_B.value;
    }

    // End of MATLABSystem: '<S12>/SourceBlock'
    // End of Outputs for SubSystem: '<S12>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe'
  }

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   MATLABSystem: '<S4>/MATLAB System'

  for (kinsim_edo_B.ret = 0; kinsim_edo_B.ret < 4; kinsim_edo_B.ret++) {
    kinsim_edo_B.bid1 = b->data[kinsim_edo_B.ret + 2] *
      kinsim_edo_B.In1.Velocities[0];
    kinsim_edo_B.bid1 += b->data[kinsim_edo_B.ret + 8] *
      kinsim_edo_B.In1.Velocities[1];
    kinsim_edo_B.bid1 += b->data[kinsim_edo_B.ret + 14] *
      kinsim_edo_B.In1.Velocities[2];
    kinsim_edo_B.cartVel[kinsim_edo_B.ret] = kinsim_edo_B.bid1;
  }

  kinsim_edo_emxFree_real_T(&b);

  // Clock: '<Root>/Clock1' incorporates:
  //   Clock: '<Root>/Clock'

  kinsim_edo_B.bid1 = kinsim_edo_M->Timing.t[0];

  // MATLAB Function: '<Root>/Assign to CartesianState msg' incorporates:
  //   Clock: '<Root>/Clock1'
  //   Constant: '<S6>/Constant'
  //   MATLAB Function: '<Root>/MATLAB Function'

  kinsim_edo_B.msg_d = kinsim_edo_P.Constant_Value;
  if (kinsim_edo_B.bid1 < 0.0) {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    kinsim_edo_B.value = ceil(kinsim_edo_B.bid1);
  } else {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    kinsim_edo_B.value = floor(kinsim_edo_B.bid1);
  }

  kinsim_edo_B.msg_d.Header.Stamp.Sec = kinsim_edo_B.value;
  kinsim_edo_B.value_h = (kinsim_edo_B.bid1 - kinsim_edo_B.value) * 1.0E+9;
  if (kinsim_edo_B.value_h < 0.0) {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    kinsim_edo_B.value_h = ceil(kinsim_edo_B.value_h);
  } else {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    kinsim_edo_B.value_h = floor(kinsim_edo_B.value_h);
  }

  kinsim_edo_B.msg_d.Header.Stamp.Nsec = kinsim_edo_B.value_h;
  kinsim_edo_B.msg_d.Name_SL_Info.CurrentLength = 3U;
  kinsim_edo_B.msg_d.Position_SL_Info.CurrentLength = 3U;
  kinsim_edo_B.msg_d.Velocity_SL_Info.CurrentLength = 3U;
  kinsim_edo_B.msg_d.Name[0].Data_SL_Info.CurrentLength = 6U;
  kinsim_edo_B.msg_d.Position[0] = kinsim_edo_B.MATLABSystem[12];
  kinsim_edo_B.msg_d.Velocity[0] = kinsim_edo_B.cartVel[0];
  kinsim_edo_B.msg_d.Name[1].Data_SL_Info.CurrentLength = 6U;
  kinsim_edo_B.msg_d.Position[1] = kinsim_edo_B.MATLABSystem[13];
  kinsim_edo_B.msg_d.Velocity[1] = kinsim_edo_B.cartVel[1];
  for (kinsim_edo_B.ret = 0; kinsim_edo_B.ret < 6; kinsim_edo_B.ret++) {
    kinsim_edo_B.b_i.f1[kinsim_edo_B.ret] = e[kinsim_edo_B.ret];
    kinsim_edo_B.c_o.f1[kinsim_edo_B.ret] = f[kinsim_edo_B.ret];
    kinsim_edo_B.d_n.f1[kinsim_edo_B.ret] = g[kinsim_edo_B.ret];
    kinsim_edo_B.msg_d.Name[0].Data[kinsim_edo_B.ret] = static_cast<uint8_T>
      (kinsim_edo_B.b_i.f1[kinsim_edo_B.ret]);
    kinsim_edo_B.msg_d.Name[1].Data[kinsim_edo_B.ret] = static_cast<uint8_T>
      (kinsim_edo_B.c_o.f1[kinsim_edo_B.ret]);
    kinsim_edo_B.msg_d.Name[2].Data[kinsim_edo_B.ret] = static_cast<uint8_T>
      (kinsim_edo_B.d_n.f1[kinsim_edo_B.ret]);
  }

  kinsim_edo_B.msg_d.Name[2].Data_SL_Info.CurrentLength = 6U;
  kinsim_edo_B.msg_d.Position[2] = kinsim_edo_B.MATLABSystem[14];
  kinsim_edo_B.msg_d.Velocity[2] = kinsim_edo_B.cartVel[2];

  // End of MATLAB Function: '<Root>/Assign to CartesianState msg'

  // Outputs for Atomic SubSystem: '<Root>/Publish2'
  // MATLABSystem: '<S11>/SinkBlock'
  Pub_kinsim_edo_79.publish(&kinsim_edo_B.msg_d);

  // End of Outputs for SubSystem: '<Root>/Publish2'

  // MATLAB Function: '<Root>/Assign to JointState msg' incorporates:
  //   Constant: '<S6>/Constant'
  //   Integrator: '<Root>/Integrator'

  kinsim_edo_B.msg_d = kinsim_edo_P.Constant_Value;
  kinsim_edo_B.msg_d.Header.Stamp.Sec = kinsim_edo_B.value;
  kinsim_edo_B.msg_d.Header.Stamp.Nsec = kinsim_edo_B.value_h;
  kinsim_edo_B.msg_d.Name_SL_Info.CurrentLength = 3U;
  kinsim_edo_B.msg_d.Position_SL_Info.CurrentLength = 3U;
  kinsim_edo_B.msg_d.Velocity_SL_Info.CurrentLength = 3U;
  kinsim_edo_B.msg_d.Name[0].Data_SL_Info.CurrentLength = 7U;
  kinsim_edo_B.msg_d.Position[0] = kinsim_edo_X.Integrator_CSTATE[0];
  kinsim_edo_B.msg_d.Velocity[0] = kinsim_edo_B.In1.Velocities[0];
  kinsim_edo_B.msg_d.Name[1].Data_SL_Info.CurrentLength = 7U;
  kinsim_edo_B.msg_d.Position[1] = kinsim_edo_X.Integrator_CSTATE[1];
  kinsim_edo_B.msg_d.Velocity[1] = kinsim_edo_B.In1.Velocities[1];
  for (kinsim_edo_B.ret = 0; kinsim_edo_B.ret < 7; kinsim_edo_B.ret++) {
    kinsim_edo_B.b_ny.f1[kinsim_edo_B.ret] = e_0[kinsim_edo_B.ret];
    kinsim_edo_B.c.f1[kinsim_edo_B.ret] = f_0[kinsim_edo_B.ret];
    kinsim_edo_B.d.f1[kinsim_edo_B.ret] = g_0[kinsim_edo_B.ret];
    kinsim_edo_B.msg_d.Name[0].Data[kinsim_edo_B.ret] = static_cast<uint8_T>
      (kinsim_edo_B.b_ny.f1[kinsim_edo_B.ret]);
    kinsim_edo_B.msg_d.Name[1].Data[kinsim_edo_B.ret] = static_cast<uint8_T>
      (kinsim_edo_B.c.f1[kinsim_edo_B.ret]);
    kinsim_edo_B.msg_d.Name[2].Data[kinsim_edo_B.ret] = static_cast<uint8_T>
      (kinsim_edo_B.d.f1[kinsim_edo_B.ret]);
  }

  kinsim_edo_B.msg_d.Name[2].Data_SL_Info.CurrentLength = 7U;
  kinsim_edo_B.msg_d.Position[2] = kinsim_edo_X.Integrator_CSTATE[2];
  kinsim_edo_B.msg_d.Velocity[2] = kinsim_edo_B.In1.Velocities[2];

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S9>/SinkBlock'
  Pub_kinsim_edo_22.publish(&kinsim_edo_B.msg_d);

  // End of Outputs for SubSystem: '<Root>/Publish'

  // MATLAB Function: '<Root>/Assign to Time msg'
  if (kinsim_edo_B.bid1 < 0.0) {
    kinsim_edo_B.value = ceil(kinsim_edo_B.bid1);
  } else {
    kinsim_edo_B.value = floor(kinsim_edo_B.bid1);
  }

  kinsim_edo_B.msg_l.Clock_.Sec = kinsim_edo_B.value;
  kinsim_edo_B.value_h = (kinsim_edo_B.bid1 - kinsim_edo_B.value) * 1.0E+9;
  if (kinsim_edo_B.value_h < 0.0) {
    kinsim_edo_B.msg_l.Clock_.Nsec = ceil(kinsim_edo_B.value_h);
  } else {
    kinsim_edo_B.msg_l.Clock_.Nsec = floor(kinsim_edo_B.value_h);
  }

  // End of MATLAB Function: '<Root>/Assign to Time msg'

  // Outputs for Atomic SubSystem: '<Root>/Publish1'
  // MATLABSystem: '<S10>/SinkBlock'
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
  XDot_kinsim_edo_T *_rtXdot;
  _rtXdot = ((XDot_kinsim_edo_T *) kinsim_edo_M->derivs);

  // Derivatives for Integrator: '<Root>/Integrator'
  _rtXdot->Integrator_CSTATE[0] = kinsim_edo_B.In1.Velocities[0];
  _rtXdot->Integrator_CSTATE[1] = kinsim_edo_B.In1.Velocities[1];
  _rtXdot->Integrator_CSTATE[2] = kinsim_edo_B.In1.Velocities[2];
}

// Model initialize function
void kinsim_edo_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

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
    char_T tmp[7];
    int32_T i;
    static const char_T tmp_0[17] = { '/', 'j', 'o', 'i', 'n', 't', '_', 't',
      'r', 'a', 'j', 'e', 'c', 't', 'o', 'r', 'y' };

    static const char_T tmp_1[17] = { '/', 'c', 'a', 'r', 't', 'e', 's', 'i',
      'a', 'n', '_', 's', 't', 'a', 't', 'e', 's' };

    static const char_T tmp_2[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 's',
      't', 'a', 't', 'e', 's' };

    static const char_T tmp_3[6] = { '/', 'c', 'l', 'o', 'c', 'k' };

    static const char_T tmp_4[22] = { '/', 'k', 'i', 'n', 's', 'i', 'm', '_',
      'e', 'd', 'o', '/', 'q', '1', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_5[22] = { '/', 'k', 'i', 'n', 's', 'i', 'm', '_',
      'e', 'd', 'o', '/', 'q', '2', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_6[22] = { '/', 'k', 'i', 'n', 's', 'i', 'm', '_',
      'e', 'd', 'o', '/', 'q', '3', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    // InitializeConditions for Integrator: '<Root>/Integrator'
    if (rtmIsFirstInitCond(kinsim_edo_M)) {
      kinsim_edo_X.Integrator_CSTATE[0] = 0.0;
      kinsim_edo_X.Integrator_CSTATE[1] = 0.0;
      kinsim_edo_X.Integrator_CSTATE[2] = 0.0;
    }

    kinsim_edo_DW.Integrator_IWORK = 1;

    // End of InitializeConditions for Integrator: '<Root>/Integrator'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S12>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S14>/Out1'
    kinsim_edo_B.In1 = kinsim_edo_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S12>/Enabled Subsystem'

    // Start for MATLABSystem: '<S12>/SourceBlock'
    kinsim_edo_DW.obj_p.matlabCodegenIsDeleted = false;
    kinsim_edo_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      kinsim_edo_B.cv1[i] = tmp_0[i];
    }

    kinsim_edo_B.cv1[17] = '\x00';
    Sub_kinsim_edo_16.createSubscriber(kinsim_edo_B.cv1, 1);
    kinsim_edo_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish2'
    // Start for MATLABSystem: '<S11>/SinkBlock'
    kinsim_edo_DW.obj_m.matlabCodegenIsDeleted = false;
    kinsim_edo_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      kinsim_edo_B.cv1[i] = tmp_1[i];
    }

    kinsim_edo_B.cv1[17] = '\x00';
    Pub_kinsim_edo_79.createPublisher(kinsim_edo_B.cv1, 1);
    kinsim_edo_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish2'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S9>/SinkBlock'
    kinsim_edo_DW.obj_nr.matlabCodegenIsDeleted = false;
    kinsim_edo_DW.obj_nr.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      kinsim_edo_B.cv2[i] = tmp_2[i];
    }

    kinsim_edo_B.cv2[13] = '\x00';
    Pub_kinsim_edo_22.createPublisher(kinsim_edo_B.cv2, 1);
    kinsim_edo_DW.obj_nr.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S10>/SinkBlock'
    kinsim_edo_DW.obj_f.matlabCodegenIsDeleted = false;
    kinsim_edo_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 6; i++) {
      tmp[i] = tmp_3[i];
    }

    tmp[6] = '\x00';
    Pub_kinsim_edo_50.createPublisher(tmp, 1);
    kinsim_edo_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish1'

    // Start for MATLABSystem: '<S13>/Get Parameter'
    kinsim_edo_DW.obj_e.matlabCodegenIsDeleted = false;
    kinsim_edo_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      kinsim_edo_B.cv[i] = tmp_4[i];
    }

    kinsim_edo_B.cv[22] = '\x00';
    ParamGet_kinsim_edo_61.initialize(kinsim_edo_B.cv);
    ParamGet_kinsim_edo_61.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_edo_61.set_initial_value(0.78539816339744828);
    kinsim_edo_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter'

    // Start for MATLABSystem: '<S13>/Get Parameter1'
    kinsim_edo_DW.obj_n.matlabCodegenIsDeleted = false;
    kinsim_edo_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      kinsim_edo_B.cv[i] = tmp_5[i];
    }

    kinsim_edo_B.cv[22] = '\x00';
    ParamGet_kinsim_edo_65.initialize(kinsim_edo_B.cv);
    ParamGet_kinsim_edo_65.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_edo_65.set_initial_value(-0.78539816339744828);
    kinsim_edo_DW.obj_n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter1'

    // Start for MATLABSystem: '<S13>/Get Parameter5'
    kinsim_edo_DW.obj_o.matlabCodegenIsDeleted = false;
    kinsim_edo_DW.obj_o.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      kinsim_edo_B.cv[i] = tmp_6[i];
    }

    kinsim_edo_B.cv[22] = '\x00';
    ParamGet_kinsim_edo_84.initialize(kinsim_edo_B.cv);
    ParamGet_kinsim_edo_84.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_edo_84.set_initial_value(0.78539816339744828);
    kinsim_edo_DW.obj_o.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter5'
    emxInitStruct_robotics_slmanip_(&kinsim_edo_DW.obj);
    emxInitStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_1);
    emxInitStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_12);
    emxInitStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_11);
    emxInitStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_10);
    emxInitStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_9);
    emxInitStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_8);
    emxInitStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_7);
    emxInitStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_6);
    emxInitStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_5);
    emxInitStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_4);
    emxInitStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_3);
    emxInitStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_2);

    // Start for MATLABSystem: '<S4>/MATLAB System'
    kinsim_edo_DW.obj.isInitialized = 0;
    kinsim_edo_DW.obj.isInitialized = 1;
    kin_RigidBodyTree_RigidBodyTree(&kinsim_edo_DW.obj.TreeInternal,
      &kinsim_edo_DW.gobj_2, &kinsim_edo_DW.gobj_4, &kinsim_edo_DW.gobj_5,
      &kinsim_edo_DW.gobj_6, &kinsim_edo_DW.gobj_7, &kinsim_edo_DW.gobj_3);
    emxInitStruct_robotics_slmani_p(&kinsim_edo_DW.obj_i);
    emxInitStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_1_a);
    emxInitStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_12_h);
    emxInitStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_11_e);
    emxInitStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_10_g);
    emxInitStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_9_f);
    emxInitStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_8_c);
    emxInitStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_7_d);
    emxInitStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_6_p);
    emxInitStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_5_k);
    emxInitStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_4_m);
    emxInitStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_3_g);
    emxInitStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_2_b);

    // Start for MATLABSystem: '<S5>/MATLAB System'
    kinsim_edo_DW.obj_i.isInitialized = 0;
    kinsim_edo_DW.obj_i.isInitialized = 1;
    k_RigidBodyTree_RigidBodyTree_p(&kinsim_edo_DW.obj_i.TreeInternal,
      &kinsim_edo_DW.gobj_2_b, &kinsim_edo_DW.gobj_4_m, &kinsim_edo_DW.gobj_5_k,
      &kinsim_edo_DW.gobj_6_p, &kinsim_edo_DW.gobj_7_d, &kinsim_edo_DW.gobj_3_g);
  }

  // set "at time zero" to false
  if (rtmIsFirstInitCond(kinsim_edo_M)) {
    rtmSetFirstInitCond(kinsim_edo_M, 0);
  }
}

// Model terminate function
void kinsim_edo_terminate(void)
{
  // Terminate for MATLABSystem: '<S13>/Get Parameter'
  matlabCodegenHandle_matlab_pcdk(&kinsim_edo_DW.obj_e);

  // Terminate for MATLABSystem: '<S13>/Get Parameter1'
  matlabCodegenHandle_matlab_pcdk(&kinsim_edo_DW.obj_n);

  // Terminate for MATLABSystem: '<S13>/Get Parameter5'
  matlabCodegenHandle_matlab_pcdk(&kinsim_edo_DW.obj_o);
  emxFreeStruct_robotics_slmanip_(&kinsim_edo_DW.obj);
  emxFreeStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_1);
  emxFreeStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_12);
  emxFreeStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_11);
  emxFreeStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_10);
  emxFreeStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_9);
  emxFreeStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_8);
  emxFreeStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_7);
  emxFreeStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_6);
  emxFreeStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_5);
  emxFreeStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_4);
  emxFreeStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_3);
  emxFreeStruct_l_robotics_manip_(&kinsim_edo_DW.gobj_2);
  emxFreeStruct_robotics_slmani_p(&kinsim_edo_DW.obj_i);
  emxFreeStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_1_a);
  emxFreeStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_12_h);
  emxFreeStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_11_e);
  emxFreeStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_10_g);
  emxFreeStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_9_f);
  emxFreeStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_8_c);
  emxFreeStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_7_d);
  emxFreeStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_6_p);
  emxFreeStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_5_k);
  emxFreeStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_4_m);
  emxFreeStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_3_g);
  emxFreeStruct_l_robotics_mani_p(&kinsim_edo_DW.gobj_2_b);

  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S12>/SourceBlock'
  matlabCodegenHandle_matlabC_pcd(&kinsim_edo_DW.obj_p);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Atomic SubSystem: '<Root>/Publish2'
  // Terminate for MATLABSystem: '<S11>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&kinsim_edo_DW.obj_m);

  // End of Terminate for SubSystem: '<Root>/Publish2'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S9>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&kinsim_edo_DW.obj_nr);

  // End of Terminate for SubSystem: '<Root>/Publish'

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S10>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&kinsim_edo_DW.obj_f);

  // End of Terminate for SubSystem: '<Root>/Publish1'
}

//
// File trailer for generated code.
//
// [EOF]
//
