/* Include files */

#include <stddef.h>
#include "blas.h"
#include "quadRotorSim_sfun.h"
#include "c10_quadRotorSim.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "quadRotorSim_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)
#define c10_b_l                        (5.0)
#define c10_b_L                        (6.5)
#define c10_b_d                        (1.78)
#define c10_b_d1                       (3.0)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c10_debug_family_names[21] = { "angle_d", "orgn_d", "c", "s",
  "P", "J", "chain_points", "i", "l", "L", "d", "d1", "nargin", "nargout", "th",
  "dth", "clip_par", "JR", "xyzP", "dxyzP", "xyz_Device" };

static const char * c10_b_debug_family_names[41] = { "c1", "c2", "s1", "s2",
  "xA", "yA", "xB", "yB", "R", "S", "M", "N", "a", "b", "c", "Delta", "y", "x",
  "phi1", "phi2", "c3", "s3", "Jpant", "Q", "J3", "origin1", "link1", "link2",
  "origin2", "nargin", "nargout", "th1", "th2", "th3", "l", "L", "d", "d1", "EE",
  "J", "chain_points" };

/* Function Declarations */
static void initialize_c10_quadRotorSim(SFc10_quadRotorSimInstanceStruct
  *chartInstance);
static void initialize_params_c10_quadRotorSim(SFc10_quadRotorSimInstanceStruct *
  chartInstance);
static void enable_c10_quadRotorSim(SFc10_quadRotorSimInstanceStruct
  *chartInstance);
static void disable_c10_quadRotorSim(SFc10_quadRotorSimInstanceStruct
  *chartInstance);
static void c10_update_debugger_state_c10_quadRotorSim
  (SFc10_quadRotorSimInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c10_quadRotorSim
  (SFc10_quadRotorSimInstanceStruct *chartInstance);
static void set_sim_state_c10_quadRotorSim(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_st);
static void finalize_c10_quadRotorSim(SFc10_quadRotorSimInstanceStruct
  *chartInstance);
static void sf_gateway_c10_quadRotorSim(SFc10_quadRotorSimInstanceStruct
  *chartInstance);
static void c10_chartstep_c10_quadRotorSim(SFc10_quadRotorSimInstanceStruct
  *chartInstance);
static void initSimStructsc10_quadRotorSim(SFc10_quadRotorSimInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c10_machineNumber, uint32_T
  c10_chartNumber, uint32_T c10_instanceNumber);
static const mxArray *c10_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static void c10_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct *chartInstance,
  const mxArray *c10_xyz_Device, const char_T *c10_identifier, real_T c10_y[15]);
static void c10_b_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId,
  real_T c10_y[15]);
static void c10_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData);
static const mxArray *c10_b_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static void c10_c_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_dxyzP, const char_T *c10_identifier, real_T
  c10_y[3]);
static void c10_d_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId,
  real_T c10_y[3]);
static void c10_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData);
static const mxArray *c10_c_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static void c10_e_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_JR, const char_T *c10_identifier, real_T
  c10_y[9]);
static void c10_f_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId,
  real_T c10_y[9]);
static void c10_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData);
static const mxArray *c10_d_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static const mxArray *c10_e_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static real_T c10_g_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId);
static void c10_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData);
static const mxArray *c10_f_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static void c10_h_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId,
  real_T c10_y[6]);
static void c10_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData);
static const mxArray *c10_g_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static void c10_i_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId,
  real_T c10_y[4]);
static void c10_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData);
static void c10_info_helper(const mxArray **c10_info);
static const mxArray *c10_emlrt_marshallOut(const char * c10_u);
static const mxArray *c10_b_emlrt_marshallOut(const uint32_T c10_u);
static void c10_fwd_kin(SFc10_quadRotorSimInstanceStruct *chartInstance, real_T
  c10_th1, real_T c10_th2, real_T c10_th3, real_T c10_c_l, real_T c10_c_L,
  real_T c10_c_d, real_T c10_c_d1, real_T c10_EE[3], real_T c10_J[9], real_T
  c10_chain_points[15]);
static real_T c10_mpower(SFc10_quadRotorSimInstanceStruct *chartInstance, real_T
  c10_a);
static void c10_eml_scalar_eg(SFc10_quadRotorSimInstanceStruct *chartInstance);
static real_T c10_sqrt(SFc10_quadRotorSimInstanceStruct *chartInstance, real_T
  c10_x);
static void c10_eml_error(SFc10_quadRotorSimInstanceStruct *chartInstance);
static real_T c10_angle(SFc10_quadRotorSimInstanceStruct *chartInstance, creal_T
  c10_x);
static void c10_b_eml_scalar_eg(SFc10_quadRotorSimInstanceStruct *chartInstance);
static void c10_threshold(SFc10_quadRotorSimInstanceStruct *chartInstance);
static void c10_c_eml_scalar_eg(SFc10_quadRotorSimInstanceStruct *chartInstance);
static void c10_d_eml_scalar_eg(SFc10_quadRotorSimInstanceStruct *chartInstance);
static const mxArray *c10_h_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static int32_T c10_j_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId);
static void c10_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData);
static uint8_T c10_k_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_b_is_active_c10_quadRotorSim, const char_T *
  c10_identifier);
static uint8_T c10_l_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId);
static void c10_b_sqrt(SFc10_quadRotorSimInstanceStruct *chartInstance, real_T
  *c10_x);
static void init_dsm_address_info(SFc10_quadRotorSimInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c10_quadRotorSim(SFc10_quadRotorSimInstanceStruct
  *chartInstance)
{
  chartInstance->c10_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c10_is_active_c10_quadRotorSim = 0U;
}

static void initialize_params_c10_quadRotorSim(SFc10_quadRotorSimInstanceStruct *
  chartInstance)
{
  real_T c10_d0;
  real_T c10_c_d1;
  real_T c10_d2;
  real_T c10_d3;
  sf_mex_import_named("l", sf_mex_get_sfun_param(chartInstance->S, 3, 0),
                      &c10_d0, 0, 0, 0U, 0, 0U, 0);
  chartInstance->c10_l = c10_d0;
  sf_mex_import_named("L", sf_mex_get_sfun_param(chartInstance->S, 0, 0),
                      &c10_c_d1, 0, 0, 0U, 0, 0U, 0);
  chartInstance->c10_L = c10_c_d1;
  sf_mex_import_named("d", sf_mex_get_sfun_param(chartInstance->S, 1, 0),
                      &c10_d2, 0, 0, 0U, 0, 0U, 0);
  chartInstance->c10_d = c10_d2;
  sf_mex_import_named("d1", sf_mex_get_sfun_param(chartInstance->S, 2, 0),
                      &c10_d3, 0, 0, 0U, 0, 0U, 0);
  chartInstance->c10_d1 = c10_d3;
}

static void enable_c10_quadRotorSim(SFc10_quadRotorSimInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c10_quadRotorSim(SFc10_quadRotorSimInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c10_update_debugger_state_c10_quadRotorSim
  (SFc10_quadRotorSimInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c10_quadRotorSim
  (SFc10_quadRotorSimInstanceStruct *chartInstance)
{
  const mxArray *c10_st;
  const mxArray *c10_y = NULL;
  int32_T c10_i0;
  real_T c10_u[9];
  const mxArray *c10_b_y = NULL;
  int32_T c10_i1;
  real_T c10_b_u[3];
  const mxArray *c10_c_y = NULL;
  int32_T c10_i2;
  real_T c10_c_u[3];
  const mxArray *c10_d_y = NULL;
  int32_T c10_i3;
  real_T c10_d_u[15];
  const mxArray *c10_e_y = NULL;
  uint8_T c10_hoistedGlobal;
  uint8_T c10_e_u;
  const mxArray *c10_f_y = NULL;
  real_T (*c10_xyz_Device)[15];
  real_T (*c10_xyzP)[3];
  real_T (*c10_dxyzP)[3];
  real_T (*c10_JR)[9];
  c10_xyz_Device = (real_T (*)[15])ssGetOutputPortSignal(chartInstance->S, 4);
  c10_dxyzP = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 3);
  c10_xyzP = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c10_JR = (real_T (*)[9])ssGetOutputPortSignal(chartInstance->S, 1);
  c10_st = NULL;
  c10_st = NULL;
  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_createcellmatrix(5, 1), false);
  for (c10_i0 = 0; c10_i0 < 9; c10_i0++) {
    c10_u[c10_i0] = (*c10_JR)[c10_i0];
  }

  c10_b_y = NULL;
  sf_mex_assign(&c10_b_y, sf_mex_create("y", c10_u, 0, 0U, 1U, 0U, 2, 3, 3),
                false);
  sf_mex_setcell(c10_y, 0, c10_b_y);
  for (c10_i1 = 0; c10_i1 < 3; c10_i1++) {
    c10_b_u[c10_i1] = (*c10_dxyzP)[c10_i1];
  }

  c10_c_y = NULL;
  sf_mex_assign(&c10_c_y, sf_mex_create("y", c10_b_u, 0, 0U, 1U, 0U, 1, 3),
                false);
  sf_mex_setcell(c10_y, 1, c10_c_y);
  for (c10_i2 = 0; c10_i2 < 3; c10_i2++) {
    c10_c_u[c10_i2] = (*c10_xyzP)[c10_i2];
  }

  c10_d_y = NULL;
  sf_mex_assign(&c10_d_y, sf_mex_create("y", c10_c_u, 0, 0U, 1U, 0U, 1, 3),
                false);
  sf_mex_setcell(c10_y, 2, c10_d_y);
  for (c10_i3 = 0; c10_i3 < 15; c10_i3++) {
    c10_d_u[c10_i3] = (*c10_xyz_Device)[c10_i3];
  }

  c10_e_y = NULL;
  sf_mex_assign(&c10_e_y, sf_mex_create("y", c10_d_u, 0, 0U, 1U, 0U, 2, 5, 3),
                false);
  sf_mex_setcell(c10_y, 3, c10_e_y);
  c10_hoistedGlobal = chartInstance->c10_is_active_c10_quadRotorSim;
  c10_e_u = c10_hoistedGlobal;
  c10_f_y = NULL;
  sf_mex_assign(&c10_f_y, sf_mex_create("y", &c10_e_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c10_y, 4, c10_f_y);
  sf_mex_assign(&c10_st, c10_y, false);
  return c10_st;
}

static void set_sim_state_c10_quadRotorSim(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_st)
{
  const mxArray *c10_u;
  real_T c10_dv0[9];
  int32_T c10_i4;
  real_T c10_dv1[3];
  int32_T c10_i5;
  real_T c10_dv2[3];
  int32_T c10_i6;
  real_T c10_dv3[15];
  int32_T c10_i7;
  real_T (*c10_JR)[9];
  real_T (*c10_dxyzP)[3];
  real_T (*c10_xyzP)[3];
  real_T (*c10_xyz_Device)[15];
  c10_xyz_Device = (real_T (*)[15])ssGetOutputPortSignal(chartInstance->S, 4);
  c10_dxyzP = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 3);
  c10_xyzP = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c10_JR = (real_T (*)[9])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c10_doneDoubleBufferReInit = true;
  c10_u = sf_mex_dup(c10_st);
  c10_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c10_u, 0)),
    "JR", c10_dv0);
  for (c10_i4 = 0; c10_i4 < 9; c10_i4++) {
    (*c10_JR)[c10_i4] = c10_dv0[c10_i4];
  }

  c10_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c10_u, 1)),
    "dxyzP", c10_dv1);
  for (c10_i5 = 0; c10_i5 < 3; c10_i5++) {
    (*c10_dxyzP)[c10_i5] = c10_dv1[c10_i5];
  }

  c10_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c10_u, 2)),
    "xyzP", c10_dv2);
  for (c10_i6 = 0; c10_i6 < 3; c10_i6++) {
    (*c10_xyzP)[c10_i6] = c10_dv2[c10_i6];
  }

  c10_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c10_u, 3)),
                       "xyz_Device", c10_dv3);
  for (c10_i7 = 0; c10_i7 < 15; c10_i7++) {
    (*c10_xyz_Device)[c10_i7] = c10_dv3[c10_i7];
  }

  chartInstance->c10_is_active_c10_quadRotorSim = c10_k_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c10_u, 4)),
     "is_active_c10_quadRotorSim");
  sf_mex_destroy(&c10_u);
  c10_update_debugger_state_c10_quadRotorSim(chartInstance);
  sf_mex_destroy(&c10_st);
}

static void finalize_c10_quadRotorSim(SFc10_quadRotorSimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c10_quadRotorSim(SFc10_quadRotorSimInstanceStruct
  *chartInstance)
{
  int32_T c10_i8;
  int32_T c10_i9;
  int32_T c10_i10;
  int32_T c10_i11;
  int32_T c10_i12;
  int32_T c10_i13;
  int32_T c10_i14;
  real_T (*c10_xyz_Device)[15];
  real_T (*c10_clip_par)[4];
  real_T (*c10_dxyzP)[3];
  real_T (*c10_xyzP)[3];
  real_T (*c10_dth)[3];
  real_T (*c10_th)[3];
  real_T (*c10_JR)[9];
  c10_xyz_Device = (real_T (*)[15])ssGetOutputPortSignal(chartInstance->S, 4);
  c10_clip_par = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 2);
  c10_dxyzP = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 3);
  c10_xyzP = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c10_dth = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c10_th = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  c10_JR = (real_T (*)[9])ssGetOutputPortSignal(chartInstance->S, 1);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 6U, chartInstance->c10_sfEvent);
  chartInstance->c10_sfEvent = CALL_EVENT;
  c10_chartstep_c10_quadRotorSim(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_quadRotorSimMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c10_i8 = 0; c10_i8 < 9; c10_i8++) {
    _SFD_DATA_RANGE_CHECK((*c10_JR)[c10_i8], 0U);
  }

  for (c10_i9 = 0; c10_i9 < 3; c10_i9++) {
    _SFD_DATA_RANGE_CHECK((*c10_th)[c10_i9], 1U);
  }

  for (c10_i10 = 0; c10_i10 < 3; c10_i10++) {
    _SFD_DATA_RANGE_CHECK((*c10_dth)[c10_i10], 2U);
  }

  _SFD_DATA_RANGE_CHECK(chartInstance->c10_l, 3U);
  for (c10_i11 = 0; c10_i11 < 3; c10_i11++) {
    _SFD_DATA_RANGE_CHECK((*c10_xyzP)[c10_i11], 4U);
  }

  _SFD_DATA_RANGE_CHECK(chartInstance->c10_L, 5U);
  _SFD_DATA_RANGE_CHECK(chartInstance->c10_d, 6U);
  _SFD_DATA_RANGE_CHECK(chartInstance->c10_d1, 7U);
  for (c10_i12 = 0; c10_i12 < 3; c10_i12++) {
    _SFD_DATA_RANGE_CHECK((*c10_dxyzP)[c10_i12], 8U);
  }

  for (c10_i13 = 0; c10_i13 < 4; c10_i13++) {
    _SFD_DATA_RANGE_CHECK((*c10_clip_par)[c10_i13], 9U);
  }

  for (c10_i14 = 0; c10_i14 < 15; c10_i14++) {
    _SFD_DATA_RANGE_CHECK((*c10_xyz_Device)[c10_i14], 10U);
  }
}

static void c10_chartstep_c10_quadRotorSim(SFc10_quadRotorSimInstanceStruct
  *chartInstance)
{
  int32_T c10_i15;
  real_T c10_th[3];
  int32_T c10_i16;
  real_T c10_dth[3];
  int32_T c10_i17;
  real_T c10_clip_par[4];
  uint32_T c10_debug_family_var_map[21];
  real_T c10_angle_d;
  real_T c10_orgn_d[3];
  real_T c10_c;
  real_T c10_s;
  real_T c10_P[3];
  real_T c10_J[9];
  real_T c10_chain_points[15];
  real_T c10_i;
  real_T c10_c_l;
  real_T c10_c_L;
  real_T c10_c_d;
  real_T c10_c_d1;
  real_T c10_nargin = 7.0;
  real_T c10_nargout = 4.0;
  real_T c10_JR[9];
  real_T c10_xyzP[3];
  real_T c10_dxyzP[3];
  real_T c10_xyz_Device[15];
  real_T c10_x;
  real_T c10_b_x;
  real_T c10_c_x;
  real_T c10_d_x;
  real_T c10_b_chain_points[15];
  real_T c10_b_J[9];
  real_T c10_b_P[3];
  int32_T c10_i18;
  int32_T c10_i19;
  int32_T c10_i20;
  real_T c10_a[9];
  int32_T c10_i21;
  int32_T c10_i22;
  static real_T c10_dv4[3] = { 0.0, 0.0, 1.0 };

  int32_T c10_i23;
  real_T c10_b[9];
  int32_T c10_i24;
  int32_T c10_i25;
  int32_T c10_i26;
  int32_T c10_i27;
  int32_T c10_i28;
  int32_T c10_i29;
  real_T c10_b_b[3];
  int32_T c10_i30;
  int32_T c10_i31;
  int32_T c10_i32;
  int32_T c10_i33;
  int32_T c10_i34;
  int32_T c10_i35;
  int32_T c10_i36;
  int32_T c10_i37;
  int32_T c10_i38;
  int32_T c10_i39;
  int32_T c10_i40;
  int32_T c10_i41;
  int32_T c10_i42;
  int32_T c10_i43;
  int32_T c10_i44;
  int32_T c10_i45;
  int32_T c10_i46;
  int32_T c10_b_i;
  int32_T c10_i47;
  int32_T c10_i48;
  int32_T c10_c_i;
  int32_T c10_i49;
  int32_T c10_i50;
  int32_T c10_i51;
  int32_T c10_i52;
  int32_T c10_d_i;
  int32_T c10_i53;
  int32_T c10_i54;
  int32_T c10_i55;
  int32_T c10_i56;
  int32_T c10_i57;
  int32_T c10_i58;
  int32_T c10_i59;
  int32_T c10_i60;
  int32_T c10_i61;
  int32_T c10_i62;
  int32_T c10_i63;
  int32_T c10_i64;
  int32_T c10_i65;
  int32_T c10_i66;
  int32_T c10_i67;
  int32_T c10_i68;
  int32_T c10_i69;
  int32_T c10_i70;
  int32_T c10_i71;
  int32_T c10_i72;
  real_T (*c10_b_JR)[9];
  real_T (*c10_b_xyzP)[3];
  real_T (*c10_b_dxyzP)[3];
  real_T (*c10_b_xyz_Device)[15];
  real_T (*c10_b_clip_par)[4];
  real_T (*c10_b_dth)[3];
  real_T (*c10_b_th)[3];
  c10_b_xyz_Device = (real_T (*)[15])ssGetOutputPortSignal(chartInstance->S, 4);
  c10_b_clip_par = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 2);
  c10_b_dxyzP = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 3);
  c10_b_xyzP = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c10_b_dth = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c10_b_th = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  c10_b_JR = (real_T (*)[9])ssGetOutputPortSignal(chartInstance->S, 1);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 6U, chartInstance->c10_sfEvent);
  for (c10_i15 = 0; c10_i15 < 3; c10_i15++) {
    c10_th[c10_i15] = (*c10_b_th)[c10_i15];
  }

  for (c10_i16 = 0; c10_i16 < 3; c10_i16++) {
    c10_dth[c10_i16] = (*c10_b_dth)[c10_i16];
  }

  for (c10_i17 = 0; c10_i17 < 4; c10_i17++) {
    c10_clip_par[c10_i17] = (*c10_b_clip_par)[c10_i17];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 21U, 21U, c10_debug_family_names,
    c10_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_angle_d, 0U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_orgn_d, 1U, c10_b_sf_marshallOut,
    c10_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_c, 2U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_s, 3U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_P, 4U, c10_b_sf_marshallOut,
    c10_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_J, 5U, c10_c_sf_marshallOut,
    c10_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_chain_points, 6U, c10_sf_marshallOut,
    c10_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_i, 7U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c10_c_l, 8U, c10_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c10_c_L, 9U, c10_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c10_c_d, 10U, c10_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c10_c_d1, 11U, c10_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_nargin, 12U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_nargout, 13U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c10_th, 14U, c10_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c10_dth, 15U, c10_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c10_clip_par, 16U, c10_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_JR, 17U, c10_c_sf_marshallOut,
    c10_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_xyzP, 18U, c10_b_sf_marshallOut,
    c10_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_dxyzP, 19U, c10_b_sf_marshallOut,
    c10_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_xyz_Device, 20U, c10_sf_marshallOut,
    c10_sf_marshallIn);
  c10_c_d1 = c10_b_d1;
  c10_c_d = c10_b_d;
  c10_c_L = c10_b_L;
  c10_c_l = c10_b_l;
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 8);
  c10_angle_d = c10_clip_par[0];
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 9);
  c10_orgn_d[0] = c10_clip_par[1];
  c10_orgn_d[1] = c10_clip_par[2];
  c10_orgn_d[2] = 7.9;
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 10);
  c10_x = c10_angle_d;
  c10_c = c10_x;
  c10_b_x = c10_c;
  c10_c = c10_b_x;
  c10_c = muDoubleScalarCos(c10_c);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 11);
  c10_c_x = c10_angle_d;
  c10_s = c10_c_x;
  c10_d_x = c10_s;
  c10_s = c10_d_x;
  c10_s = muDoubleScalarSin(c10_s);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 13);
  c10_fwd_kin(chartInstance, c10_th[0], c10_th[1], -c10_th[2], c10_b_l, c10_b_L,
              c10_b_d, c10_b_d1, c10_b_P, c10_b_J, c10_b_chain_points);
  for (c10_i18 = 0; c10_i18 < 3; c10_i18++) {
    c10_P[c10_i18] = c10_b_P[c10_i18];
  }

  for (c10_i19 = 0; c10_i19 < 9; c10_i19++) {
    c10_J[c10_i19] = c10_b_J[c10_i19];
  }

  for (c10_i20 = 0; c10_i20 < 15; c10_i20++) {
    c10_chain_points[c10_i20] = c10_b_chain_points[c10_i20];
  }

  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 14);
  c10_a[0] = c10_c;
  c10_a[3] = -c10_s;
  c10_a[6] = 0.0;
  c10_a[1] = c10_s;
  c10_a[4] = c10_c;
  c10_a[7] = 0.0;
  c10_i21 = 0;
  for (c10_i22 = 0; c10_i22 < 3; c10_i22++) {
    c10_a[c10_i21 + 2] = c10_dv4[c10_i22];
    c10_i21 += 3;
  }

  for (c10_i23 = 0; c10_i23 < 9; c10_i23++) {
    c10_b[c10_i23] = c10_J[c10_i23];
  }

  c10_d_eml_scalar_eg(chartInstance);
  c10_d_eml_scalar_eg(chartInstance);
  c10_threshold(chartInstance);
  for (c10_i24 = 0; c10_i24 < 3; c10_i24++) {
    c10_i25 = 0;
    for (c10_i26 = 0; c10_i26 < 3; c10_i26++) {
      c10_b_J[c10_i25 + c10_i24] = 0.0;
      c10_i27 = 0;
      for (c10_i28 = 0; c10_i28 < 3; c10_i28++) {
        c10_b_J[c10_i25 + c10_i24] += c10_a[c10_i27 + c10_i24] * c10_b[c10_i28 +
          c10_i25];
        c10_i27 += 3;
      }

      c10_i25 += 3;
    }
  }

  for (c10_i29 = 0; c10_i29 < 3; c10_i29++) {
    c10_b_b[c10_i29] = c10_dth[c10_i29];
  }

  c10_b_eml_scalar_eg(chartInstance);
  c10_b_eml_scalar_eg(chartInstance);
  for (c10_i30 = 0; c10_i30 < 3; c10_i30++) {
    c10_dxyzP[c10_i30] = 0.0;
  }

  for (c10_i31 = 0; c10_i31 < 3; c10_i31++) {
    c10_dxyzP[c10_i31] = 0.0;
  }

  for (c10_i32 = 0; c10_i32 < 3; c10_i32++) {
    c10_b_P[c10_i32] = c10_dxyzP[c10_i32];
  }

  for (c10_i33 = 0; c10_i33 < 3; c10_i33++) {
    c10_dxyzP[c10_i33] = c10_b_P[c10_i33];
  }

  c10_threshold(chartInstance);
  for (c10_i34 = 0; c10_i34 < 3; c10_i34++) {
    c10_b_P[c10_i34] = c10_dxyzP[c10_i34];
  }

  for (c10_i35 = 0; c10_i35 < 3; c10_i35++) {
    c10_dxyzP[c10_i35] = c10_b_P[c10_i35];
  }

  for (c10_i36 = 0; c10_i36 < 3; c10_i36++) {
    c10_dxyzP[c10_i36] = 0.0;
    c10_i37 = 0;
    for (c10_i38 = 0; c10_i38 < 3; c10_i38++) {
      c10_dxyzP[c10_i36] += c10_b_J[c10_i37 + c10_i36] * c10_b_b[c10_i38];
      c10_i37 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 16);
  c10_a[0] = c10_c;
  c10_a[3] = -c10_s;
  c10_a[6] = 0.0;
  c10_a[1] = c10_s;
  c10_a[4] = c10_c;
  c10_a[7] = 0.0;
  c10_i39 = 0;
  for (c10_i40 = 0; c10_i40 < 3; c10_i40++) {
    c10_a[c10_i39 + 2] = c10_dv4[c10_i40];
    c10_i39 += 3;
  }

  for (c10_i41 = 0; c10_i41 < 3; c10_i41++) {
    c10_b_b[c10_i41] = c10_P[c10_i41];
  }

  c10_b_eml_scalar_eg(chartInstance);
  c10_b_eml_scalar_eg(chartInstance);
  c10_threshold(chartInstance);
  for (c10_i42 = 0; c10_i42 < 3; c10_i42++) {
    c10_b_P[c10_i42] = 0.0;
    c10_i43 = 0;
    for (c10_i44 = 0; c10_i44 < 3; c10_i44++) {
      c10_b_P[c10_i42] += c10_a[c10_i43 + c10_i42] * c10_b_b[c10_i44];
      c10_i43 += 3;
    }
  }

  for (c10_i45 = 0; c10_i45 < 3; c10_i45++) {
    c10_P[c10_i45] = c10_b_P[c10_i45] + c10_orgn_d[c10_i45];
  }

  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 17);
  for (c10_i46 = 0; c10_i46 < 3; c10_i46++) {
    c10_xyzP[c10_i46] = c10_P[c10_i46];
  }

  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 20);
  c10_i = 1.0;
  c10_b_i = 0;
  while (c10_b_i < 5) {
    c10_i = 1.0 + (real_T)c10_b_i;
    CV_EML_FOR(0, 1, 0, 1);
    _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 21);
    c10_a[0] = c10_c;
    c10_a[3] = -c10_s;
    c10_a[6] = 0.0;
    c10_a[1] = c10_s;
    c10_a[4] = c10_c;
    c10_a[7] = 0.0;
    c10_i47 = 0;
    for (c10_i48 = 0; c10_i48 < 3; c10_i48++) {
      c10_a[c10_i47 + 2] = c10_dv4[c10_i48];
      c10_i47 += 3;
    }

    c10_c_i = _SFD_EML_ARRAY_BOUNDS_CHECK("chain_points", (int32_T)
      _SFD_INTEGER_CHECK("i", c10_i), 1, 5, 1, 0) - 1;
    for (c10_i49 = 0; c10_i49 < 3; c10_i49++) {
      c10_b_b[c10_i49] = c10_chain_points[c10_c_i + 5 * c10_i49];
    }

    c10_b_eml_scalar_eg(chartInstance);
    c10_b_eml_scalar_eg(chartInstance);
    c10_threshold(chartInstance);
    for (c10_i50 = 0; c10_i50 < 3; c10_i50++) {
      c10_b_P[c10_i50] = 0.0;
      c10_i51 = 0;
      for (c10_i52 = 0; c10_i52 < 3; c10_i52++) {
        c10_b_P[c10_i50] += c10_a[c10_i51 + c10_i50] * c10_b_b[c10_i52];
        c10_i51 += 3;
      }
    }

    c10_d_i = _SFD_EML_ARRAY_BOUNDS_CHECK("chain_points", (int32_T)
      _SFD_INTEGER_CHECK("i", c10_i), 1, 5, 1, 0) - 1;
    for (c10_i53 = 0; c10_i53 < 3; c10_i53++) {
      c10_chain_points[c10_d_i + 5 * c10_i53] = c10_b_P[c10_i53] +
        c10_orgn_d[c10_i53];
    }

    c10_b_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 0, 0);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 23);
  for (c10_i54 = 0; c10_i54 < 15; c10_i54++) {
    c10_xyz_Device[c10_i54] = c10_chain_points[c10_i54];
  }

  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 26);
  for (c10_i55 = 0; c10_i55 < 9; c10_i55++) {
    c10_a[c10_i55] = c10_J[c10_i55];
  }

  c10_b[0] = c10_c;
  c10_b[3] = c10_s;
  c10_b[6] = 0.0;
  c10_b[1] = -c10_s;
  c10_b[4] = c10_c;
  c10_b[7] = 0.0;
  c10_i56 = 0;
  for (c10_i57 = 0; c10_i57 < 3; c10_i57++) {
    c10_b[c10_i56 + 2] = c10_dv4[c10_i57];
    c10_i56 += 3;
  }

  c10_d_eml_scalar_eg(chartInstance);
  c10_d_eml_scalar_eg(chartInstance);
  for (c10_i58 = 0; c10_i58 < 9; c10_i58++) {
    c10_JR[c10_i58] = 0.0;
  }

  for (c10_i59 = 0; c10_i59 < 9; c10_i59++) {
    c10_JR[c10_i59] = 0.0;
  }

  for (c10_i60 = 0; c10_i60 < 9; c10_i60++) {
    c10_b_J[c10_i60] = c10_JR[c10_i60];
  }

  for (c10_i61 = 0; c10_i61 < 9; c10_i61++) {
    c10_JR[c10_i61] = c10_b_J[c10_i61];
  }

  c10_threshold(chartInstance);
  for (c10_i62 = 0; c10_i62 < 9; c10_i62++) {
    c10_b_J[c10_i62] = c10_JR[c10_i62];
  }

  for (c10_i63 = 0; c10_i63 < 9; c10_i63++) {
    c10_JR[c10_i63] = c10_b_J[c10_i63];
  }

  for (c10_i64 = 0; c10_i64 < 3; c10_i64++) {
    c10_i65 = 0;
    for (c10_i66 = 0; c10_i66 < 3; c10_i66++) {
      c10_JR[c10_i65 + c10_i64] = 0.0;
      c10_i67 = 0;
      for (c10_i68 = 0; c10_i68 < 3; c10_i68++) {
        c10_JR[c10_i65 + c10_i64] += c10_a[c10_i67 + c10_i64] * c10_b[c10_i68 +
          c10_i65];
        c10_i67 += 3;
      }

      c10_i65 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, -26);
  _SFD_SYMBOL_SCOPE_POP();
  for (c10_i69 = 0; c10_i69 < 9; c10_i69++) {
    (*c10_b_JR)[c10_i69] = c10_JR[c10_i69];
  }

  for (c10_i70 = 0; c10_i70 < 3; c10_i70++) {
    (*c10_b_xyzP)[c10_i70] = c10_xyzP[c10_i70];
  }

  for (c10_i71 = 0; c10_i71 < 3; c10_i71++) {
    (*c10_b_dxyzP)[c10_i71] = c10_dxyzP[c10_i71];
  }

  for (c10_i72 = 0; c10_i72 < 15; c10_i72++) {
    (*c10_b_xyz_Device)[c10_i72] = c10_xyz_Device[c10_i72];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 6U, chartInstance->c10_sfEvent);
}

static void initSimStructsc10_quadRotorSim(SFc10_quadRotorSimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c10_machineNumber, uint32_T
  c10_chartNumber, uint32_T c10_instanceNumber)
{
  (void)c10_machineNumber;
  (void)c10_chartNumber;
  (void)c10_instanceNumber;
}

static const mxArray *c10_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  int32_T c10_i73;
  int32_T c10_i74;
  int32_T c10_i75;
  real_T c10_b_inData[15];
  int32_T c10_i76;
  int32_T c10_i77;
  int32_T c10_i78;
  real_T c10_u[15];
  const mxArray *c10_y = NULL;
  SFc10_quadRotorSimInstanceStruct *chartInstance;
  chartInstance = (SFc10_quadRotorSimInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  c10_i73 = 0;
  for (c10_i74 = 0; c10_i74 < 3; c10_i74++) {
    for (c10_i75 = 0; c10_i75 < 5; c10_i75++) {
      c10_b_inData[c10_i75 + c10_i73] = (*(real_T (*)[15])c10_inData)[c10_i75 +
        c10_i73];
    }

    c10_i73 += 5;
  }

  c10_i76 = 0;
  for (c10_i77 = 0; c10_i77 < 3; c10_i77++) {
    for (c10_i78 = 0; c10_i78 < 5; c10_i78++) {
      c10_u[c10_i78 + c10_i76] = c10_b_inData[c10_i78 + c10_i76];
    }

    c10_i76 += 5;
  }

  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", c10_u, 0, 0U, 1U, 0U, 2, 5, 3), false);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, false);
  return c10_mxArrayOutData;
}

static void c10_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct *chartInstance,
  const mxArray *c10_xyz_Device, const char_T *c10_identifier, real_T c10_y[15])
{
  emlrtMsgIdentifier c10_thisId;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_xyz_Device), &c10_thisId,
    c10_y);
  sf_mex_destroy(&c10_xyz_Device);
}

static void c10_b_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId,
  real_T c10_y[15])
{
  real_T c10_dv5[15];
  int32_T c10_i79;
  (void)chartInstance;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), c10_dv5, 1, 0, 0U, 1, 0U, 2, 5,
                3);
  for (c10_i79 = 0; c10_i79 < 15; c10_i79++) {
    c10_y[c10_i79] = c10_dv5[c10_i79];
  }

  sf_mex_destroy(&c10_u);
}

static void c10_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData)
{
  const mxArray *c10_xyz_Device;
  const char_T *c10_identifier;
  emlrtMsgIdentifier c10_thisId;
  real_T c10_y[15];
  int32_T c10_i80;
  int32_T c10_i81;
  int32_T c10_i82;
  SFc10_quadRotorSimInstanceStruct *chartInstance;
  chartInstance = (SFc10_quadRotorSimInstanceStruct *)chartInstanceVoid;
  c10_xyz_Device = sf_mex_dup(c10_mxArrayInData);
  c10_identifier = c10_varName;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_xyz_Device), &c10_thisId,
    c10_y);
  sf_mex_destroy(&c10_xyz_Device);
  c10_i80 = 0;
  for (c10_i81 = 0; c10_i81 < 3; c10_i81++) {
    for (c10_i82 = 0; c10_i82 < 5; c10_i82++) {
      (*(real_T (*)[15])c10_outData)[c10_i82 + c10_i80] = c10_y[c10_i82 +
        c10_i80];
    }

    c10_i80 += 5;
  }

  sf_mex_destroy(&c10_mxArrayInData);
}

static const mxArray *c10_b_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  int32_T c10_i83;
  real_T c10_b_inData[3];
  int32_T c10_i84;
  real_T c10_u[3];
  const mxArray *c10_y = NULL;
  SFc10_quadRotorSimInstanceStruct *chartInstance;
  chartInstance = (SFc10_quadRotorSimInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  for (c10_i83 = 0; c10_i83 < 3; c10_i83++) {
    c10_b_inData[c10_i83] = (*(real_T (*)[3])c10_inData)[c10_i83];
  }

  for (c10_i84 = 0; c10_i84 < 3; c10_i84++) {
    c10_u[c10_i84] = c10_b_inData[c10_i84];
  }

  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", c10_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, false);
  return c10_mxArrayOutData;
}

static void c10_c_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_dxyzP, const char_T *c10_identifier, real_T
  c10_y[3])
{
  emlrtMsgIdentifier c10_thisId;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_dxyzP), &c10_thisId,
    c10_y);
  sf_mex_destroy(&c10_dxyzP);
}

static void c10_d_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId,
  real_T c10_y[3])
{
  real_T c10_dv6[3];
  int32_T c10_i85;
  (void)chartInstance;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), c10_dv6, 1, 0, 0U, 1, 0U, 1, 3);
  for (c10_i85 = 0; c10_i85 < 3; c10_i85++) {
    c10_y[c10_i85] = c10_dv6[c10_i85];
  }

  sf_mex_destroy(&c10_u);
}

static void c10_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData)
{
  const mxArray *c10_dxyzP;
  const char_T *c10_identifier;
  emlrtMsgIdentifier c10_thisId;
  real_T c10_y[3];
  int32_T c10_i86;
  SFc10_quadRotorSimInstanceStruct *chartInstance;
  chartInstance = (SFc10_quadRotorSimInstanceStruct *)chartInstanceVoid;
  c10_dxyzP = sf_mex_dup(c10_mxArrayInData);
  c10_identifier = c10_varName;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_dxyzP), &c10_thisId,
    c10_y);
  sf_mex_destroy(&c10_dxyzP);
  for (c10_i86 = 0; c10_i86 < 3; c10_i86++) {
    (*(real_T (*)[3])c10_outData)[c10_i86] = c10_y[c10_i86];
  }

  sf_mex_destroy(&c10_mxArrayInData);
}

static const mxArray *c10_c_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  int32_T c10_i87;
  int32_T c10_i88;
  int32_T c10_i89;
  real_T c10_b_inData[9];
  int32_T c10_i90;
  int32_T c10_i91;
  int32_T c10_i92;
  real_T c10_u[9];
  const mxArray *c10_y = NULL;
  SFc10_quadRotorSimInstanceStruct *chartInstance;
  chartInstance = (SFc10_quadRotorSimInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  c10_i87 = 0;
  for (c10_i88 = 0; c10_i88 < 3; c10_i88++) {
    for (c10_i89 = 0; c10_i89 < 3; c10_i89++) {
      c10_b_inData[c10_i89 + c10_i87] = (*(real_T (*)[9])c10_inData)[c10_i89 +
        c10_i87];
    }

    c10_i87 += 3;
  }

  c10_i90 = 0;
  for (c10_i91 = 0; c10_i91 < 3; c10_i91++) {
    for (c10_i92 = 0; c10_i92 < 3; c10_i92++) {
      c10_u[c10_i92 + c10_i90] = c10_b_inData[c10_i92 + c10_i90];
    }

    c10_i90 += 3;
  }

  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", c10_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, false);
  return c10_mxArrayOutData;
}

static void c10_e_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_JR, const char_T *c10_identifier, real_T
  c10_y[9])
{
  emlrtMsgIdentifier c10_thisId;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_JR), &c10_thisId, c10_y);
  sf_mex_destroy(&c10_JR);
}

static void c10_f_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId,
  real_T c10_y[9])
{
  real_T c10_dv7[9];
  int32_T c10_i93;
  (void)chartInstance;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), c10_dv7, 1, 0, 0U, 1, 0U, 2, 3,
                3);
  for (c10_i93 = 0; c10_i93 < 9; c10_i93++) {
    c10_y[c10_i93] = c10_dv7[c10_i93];
  }

  sf_mex_destroy(&c10_u);
}

static void c10_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData)
{
  const mxArray *c10_JR;
  const char_T *c10_identifier;
  emlrtMsgIdentifier c10_thisId;
  real_T c10_y[9];
  int32_T c10_i94;
  int32_T c10_i95;
  int32_T c10_i96;
  SFc10_quadRotorSimInstanceStruct *chartInstance;
  chartInstance = (SFc10_quadRotorSimInstanceStruct *)chartInstanceVoid;
  c10_JR = sf_mex_dup(c10_mxArrayInData);
  c10_identifier = c10_varName;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_JR), &c10_thisId, c10_y);
  sf_mex_destroy(&c10_JR);
  c10_i94 = 0;
  for (c10_i95 = 0; c10_i95 < 3; c10_i95++) {
    for (c10_i96 = 0; c10_i96 < 3; c10_i96++) {
      (*(real_T (*)[9])c10_outData)[c10_i96 + c10_i94] = c10_y[c10_i96 + c10_i94];
    }

    c10_i94 += 3;
  }

  sf_mex_destroy(&c10_mxArrayInData);
}

static const mxArray *c10_d_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  int32_T c10_i97;
  real_T c10_b_inData[4];
  int32_T c10_i98;
  real_T c10_u[4];
  const mxArray *c10_y = NULL;
  SFc10_quadRotorSimInstanceStruct *chartInstance;
  chartInstance = (SFc10_quadRotorSimInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  for (c10_i97 = 0; c10_i97 < 4; c10_i97++) {
    c10_b_inData[c10_i97] = (*(real_T (*)[4])c10_inData)[c10_i97];
  }

  for (c10_i98 = 0; c10_i98 < 4; c10_i98++) {
    c10_u[c10_i98] = c10_b_inData[c10_i98];
  }

  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", c10_u, 0, 0U, 1U, 0U, 1, 4), false);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, false);
  return c10_mxArrayOutData;
}

static const mxArray *c10_e_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  real_T c10_u;
  const mxArray *c10_y = NULL;
  SFc10_quadRotorSimInstanceStruct *chartInstance;
  chartInstance = (SFc10_quadRotorSimInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  c10_u = *(real_T *)c10_inData;
  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", &c10_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, false);
  return c10_mxArrayOutData;
}

static real_T c10_g_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId)
{
  real_T c10_y;
  real_T c10_d4;
  (void)chartInstance;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), &c10_d4, 1, 0, 0U, 0, 0U, 0);
  c10_y = c10_d4;
  sf_mex_destroy(&c10_u);
  return c10_y;
}

static void c10_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData)
{
  const mxArray *c10_nargout;
  const char_T *c10_identifier;
  emlrtMsgIdentifier c10_thisId;
  real_T c10_y;
  SFc10_quadRotorSimInstanceStruct *chartInstance;
  chartInstance = (SFc10_quadRotorSimInstanceStruct *)chartInstanceVoid;
  c10_nargout = sf_mex_dup(c10_mxArrayInData);
  c10_identifier = c10_varName;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_y = c10_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_nargout),
    &c10_thisId);
  sf_mex_destroy(&c10_nargout);
  *(real_T *)c10_outData = c10_y;
  sf_mex_destroy(&c10_mxArrayInData);
}

static const mxArray *c10_f_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  int32_T c10_i99;
  int32_T c10_i100;
  int32_T c10_i101;
  real_T c10_b_inData[6];
  int32_T c10_i102;
  int32_T c10_i103;
  int32_T c10_i104;
  real_T c10_u[6];
  const mxArray *c10_y = NULL;
  SFc10_quadRotorSimInstanceStruct *chartInstance;
  chartInstance = (SFc10_quadRotorSimInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  c10_i99 = 0;
  for (c10_i100 = 0; c10_i100 < 2; c10_i100++) {
    for (c10_i101 = 0; c10_i101 < 3; c10_i101++) {
      c10_b_inData[c10_i101 + c10_i99] = (*(real_T (*)[6])c10_inData)[c10_i101 +
        c10_i99];
    }

    c10_i99 += 3;
  }

  c10_i102 = 0;
  for (c10_i103 = 0; c10_i103 < 2; c10_i103++) {
    for (c10_i104 = 0; c10_i104 < 3; c10_i104++) {
      c10_u[c10_i104 + c10_i102] = c10_b_inData[c10_i104 + c10_i102];
    }

    c10_i102 += 3;
  }

  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", c10_u, 0, 0U, 1U, 0U, 2, 3, 2), false);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, false);
  return c10_mxArrayOutData;
}

static void c10_h_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId,
  real_T c10_y[6])
{
  real_T c10_dv8[6];
  int32_T c10_i105;
  (void)chartInstance;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), c10_dv8, 1, 0, 0U, 1, 0U, 2, 3,
                2);
  for (c10_i105 = 0; c10_i105 < 6; c10_i105++) {
    c10_y[c10_i105] = c10_dv8[c10_i105];
  }

  sf_mex_destroy(&c10_u);
}

static void c10_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData)
{
  const mxArray *c10_Q;
  const char_T *c10_identifier;
  emlrtMsgIdentifier c10_thisId;
  real_T c10_y[6];
  int32_T c10_i106;
  int32_T c10_i107;
  int32_T c10_i108;
  SFc10_quadRotorSimInstanceStruct *chartInstance;
  chartInstance = (SFc10_quadRotorSimInstanceStruct *)chartInstanceVoid;
  c10_Q = sf_mex_dup(c10_mxArrayInData);
  c10_identifier = c10_varName;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_Q), &c10_thisId, c10_y);
  sf_mex_destroy(&c10_Q);
  c10_i106 = 0;
  for (c10_i107 = 0; c10_i107 < 2; c10_i107++) {
    for (c10_i108 = 0; c10_i108 < 3; c10_i108++) {
      (*(real_T (*)[6])c10_outData)[c10_i108 + c10_i106] = c10_y[c10_i108 +
        c10_i106];
    }

    c10_i106 += 3;
  }

  sf_mex_destroy(&c10_mxArrayInData);
}

static const mxArray *c10_g_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  int32_T c10_i109;
  int32_T c10_i110;
  int32_T c10_i111;
  real_T c10_b_inData[4];
  int32_T c10_i112;
  int32_T c10_i113;
  int32_T c10_i114;
  real_T c10_u[4];
  const mxArray *c10_y = NULL;
  SFc10_quadRotorSimInstanceStruct *chartInstance;
  chartInstance = (SFc10_quadRotorSimInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  c10_i109 = 0;
  for (c10_i110 = 0; c10_i110 < 2; c10_i110++) {
    for (c10_i111 = 0; c10_i111 < 2; c10_i111++) {
      c10_b_inData[c10_i111 + c10_i109] = (*(real_T (*)[4])c10_inData)[c10_i111
        + c10_i109];
    }

    c10_i109 += 2;
  }

  c10_i112 = 0;
  for (c10_i113 = 0; c10_i113 < 2; c10_i113++) {
    for (c10_i114 = 0; c10_i114 < 2; c10_i114++) {
      c10_u[c10_i114 + c10_i112] = c10_b_inData[c10_i114 + c10_i112];
    }

    c10_i112 += 2;
  }

  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", c10_u, 0, 0U, 1U, 0U, 2, 2, 2), false);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, false);
  return c10_mxArrayOutData;
}

static void c10_i_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId,
  real_T c10_y[4])
{
  real_T c10_dv9[4];
  int32_T c10_i115;
  (void)chartInstance;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), c10_dv9, 1, 0, 0U, 1, 0U, 2, 2,
                2);
  for (c10_i115 = 0; c10_i115 < 4; c10_i115++) {
    c10_y[c10_i115] = c10_dv9[c10_i115];
  }

  sf_mex_destroy(&c10_u);
}

static void c10_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData)
{
  const mxArray *c10_Jpant;
  const char_T *c10_identifier;
  emlrtMsgIdentifier c10_thisId;
  real_T c10_y[4];
  int32_T c10_i116;
  int32_T c10_i117;
  int32_T c10_i118;
  SFc10_quadRotorSimInstanceStruct *chartInstance;
  chartInstance = (SFc10_quadRotorSimInstanceStruct *)chartInstanceVoid;
  c10_Jpant = sf_mex_dup(c10_mxArrayInData);
  c10_identifier = c10_varName;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_Jpant), &c10_thisId,
    c10_y);
  sf_mex_destroy(&c10_Jpant);
  c10_i116 = 0;
  for (c10_i117 = 0; c10_i117 < 2; c10_i117++) {
    for (c10_i118 = 0; c10_i118 < 2; c10_i118++) {
      (*(real_T (*)[4])c10_outData)[c10_i118 + c10_i116] = c10_y[c10_i118 +
        c10_i116];
    }

    c10_i116 += 2;
  }

  sf_mex_destroy(&c10_mxArrayInData);
}

const mxArray *sf_c10_quadRotorSim_get_eml_resolved_functions_info(void)
{
  const mxArray *c10_nameCaptureInfo = NULL;
  c10_nameCaptureInfo = NULL;
  sf_mex_assign(&c10_nameCaptureInfo, sf_mex_createstruct("structure", 2, 48, 1),
                false);
  c10_info_helper(&c10_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c10_nameCaptureInfo);
  return c10_nameCaptureInfo;
}

static void c10_info_helper(const mxArray **c10_info)
{
  const mxArray *c10_rhs0 = NULL;
  const mxArray *c10_lhs0 = NULL;
  const mxArray *c10_rhs1 = NULL;
  const mxArray *c10_lhs1 = NULL;
  const mxArray *c10_rhs2 = NULL;
  const mxArray *c10_lhs2 = NULL;
  const mxArray *c10_rhs3 = NULL;
  const mxArray *c10_lhs3 = NULL;
  const mxArray *c10_rhs4 = NULL;
  const mxArray *c10_lhs4 = NULL;
  const mxArray *c10_rhs5 = NULL;
  const mxArray *c10_lhs5 = NULL;
  const mxArray *c10_rhs6 = NULL;
  const mxArray *c10_lhs6 = NULL;
  const mxArray *c10_rhs7 = NULL;
  const mxArray *c10_lhs7 = NULL;
  const mxArray *c10_rhs8 = NULL;
  const mxArray *c10_lhs8 = NULL;
  const mxArray *c10_rhs9 = NULL;
  const mxArray *c10_lhs9 = NULL;
  const mxArray *c10_rhs10 = NULL;
  const mxArray *c10_lhs10 = NULL;
  const mxArray *c10_rhs11 = NULL;
  const mxArray *c10_lhs11 = NULL;
  const mxArray *c10_rhs12 = NULL;
  const mxArray *c10_lhs12 = NULL;
  const mxArray *c10_rhs13 = NULL;
  const mxArray *c10_lhs13 = NULL;
  const mxArray *c10_rhs14 = NULL;
  const mxArray *c10_lhs14 = NULL;
  const mxArray *c10_rhs15 = NULL;
  const mxArray *c10_lhs15 = NULL;
  const mxArray *c10_rhs16 = NULL;
  const mxArray *c10_lhs16 = NULL;
  const mxArray *c10_rhs17 = NULL;
  const mxArray *c10_lhs17 = NULL;
  const mxArray *c10_rhs18 = NULL;
  const mxArray *c10_lhs18 = NULL;
  const mxArray *c10_rhs19 = NULL;
  const mxArray *c10_lhs19 = NULL;
  const mxArray *c10_rhs20 = NULL;
  const mxArray *c10_lhs20 = NULL;
  const mxArray *c10_rhs21 = NULL;
  const mxArray *c10_lhs21 = NULL;
  const mxArray *c10_rhs22 = NULL;
  const mxArray *c10_lhs22 = NULL;
  const mxArray *c10_rhs23 = NULL;
  const mxArray *c10_lhs23 = NULL;
  const mxArray *c10_rhs24 = NULL;
  const mxArray *c10_lhs24 = NULL;
  const mxArray *c10_rhs25 = NULL;
  const mxArray *c10_lhs25 = NULL;
  const mxArray *c10_rhs26 = NULL;
  const mxArray *c10_lhs26 = NULL;
  const mxArray *c10_rhs27 = NULL;
  const mxArray *c10_lhs27 = NULL;
  const mxArray *c10_rhs28 = NULL;
  const mxArray *c10_lhs28 = NULL;
  const mxArray *c10_rhs29 = NULL;
  const mxArray *c10_lhs29 = NULL;
  const mxArray *c10_rhs30 = NULL;
  const mxArray *c10_lhs30 = NULL;
  const mxArray *c10_rhs31 = NULL;
  const mxArray *c10_lhs31 = NULL;
  const mxArray *c10_rhs32 = NULL;
  const mxArray *c10_lhs32 = NULL;
  const mxArray *c10_rhs33 = NULL;
  const mxArray *c10_lhs33 = NULL;
  const mxArray *c10_rhs34 = NULL;
  const mxArray *c10_lhs34 = NULL;
  const mxArray *c10_rhs35 = NULL;
  const mxArray *c10_lhs35 = NULL;
  const mxArray *c10_rhs36 = NULL;
  const mxArray *c10_lhs36 = NULL;
  const mxArray *c10_rhs37 = NULL;
  const mxArray *c10_lhs37 = NULL;
  const mxArray *c10_rhs38 = NULL;
  const mxArray *c10_lhs38 = NULL;
  const mxArray *c10_rhs39 = NULL;
  const mxArray *c10_lhs39 = NULL;
  const mxArray *c10_rhs40 = NULL;
  const mxArray *c10_lhs40 = NULL;
  const mxArray *c10_rhs41 = NULL;
  const mxArray *c10_lhs41 = NULL;
  const mxArray *c10_rhs42 = NULL;
  const mxArray *c10_lhs42 = NULL;
  const mxArray *c10_rhs43 = NULL;
  const mxArray *c10_lhs43 = NULL;
  const mxArray *c10_rhs44 = NULL;
  const mxArray *c10_lhs44 = NULL;
  const mxArray *c10_rhs45 = NULL;
  const mxArray *c10_lhs45 = NULL;
  const mxArray *c10_rhs46 = NULL;
  const mxArray *c10_lhs46 = NULL;
  const mxArray *c10_rhs47 = NULL;
  const mxArray *c10_lhs47 = NULL;
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("cos"), "name", "name", 0);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1343851972U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c10_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs0), "rhs", "rhs",
                  0);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs0), "lhs", "lhs",
                  0);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_scalar_cos"), "name",
                  "name", 1);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1286840322U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c10_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs1), "rhs", "rhs",
                  1);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs1), "lhs", "lhs",
                  1);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(""), "context", "context", 2);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("sin"), "name", "name", 2);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1343851986U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c10_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs2), "rhs", "rhs",
                  2);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs2), "lhs", "lhs",
                  2);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_scalar_sin"), "name",
                  "name", 3);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1286840336U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c10_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs3), "rhs", "rhs",
                  3);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs3), "lhs", "lhs",
                  3);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(""), "context", "context", 4);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("mpower"), "name", "name", 4);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "resolved",
                  "resolved", 4);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1363731878U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c10_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs4), "rhs", "rhs",
                  4);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs4), "lhs", "lhs",
                  4);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 5);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c10_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs5), "rhs", "rhs",
                  5);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs5), "lhs", "lhs",
                  5);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("ismatrix"), "name", "name",
                  6);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1331326458U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c10_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs6), "rhs", "rhs",
                  6);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs6), "lhs", "lhs",
                  6);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 7);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("power"), "name", "name", 7);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c10_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs7), "rhs", "rhs",
                  7);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs7), "lhs", "lhs",
                  7);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 8);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c10_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs8), "rhs", "rhs",
                  8);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs8), "lhs", "lhs",
                  8);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 9);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 9);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c10_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs9), "rhs", "rhs",
                  9);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs9), "lhs", "lhs",
                  9);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 10);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c10_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 11);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 11);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c10_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 12);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 12);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c10_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 13);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("floor"), "name", "name", 13);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 13);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1363731854U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c10_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 14);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 14);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c10_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 15);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 15);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1286840326U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c10_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 16);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 16);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 16);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c10_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(""), "context", "context", 17);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("mrdivide"), "name", "name",
                  17);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 17);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1388481696U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c10_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 18);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 18);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c10_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 19);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("rdivide"), "name", "name",
                  19);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 19);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c10_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 20);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 20);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 20);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c10_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 21);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 21);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 21);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1286840396U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c10_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 22);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_div"), "name", "name",
                  22);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 22);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c10_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 23);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 23);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 23);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c10_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(""), "context", "context", 24);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 24);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1383898894U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c10_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 25);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 25);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 25);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c10_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 26);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 26);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 26);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c10_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(""), "context", "context", 27);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("sqrt"), "name", "name", 27);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 27);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1343851986U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c10_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 28);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_error"), "name", "name",
                  28);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 28);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1343851958U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c10_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 29);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 29);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1286840338U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c10_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p!eml_fldiv"),
                  "context", "context", 30);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 30);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 30);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c10_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p!eml_fldiv"),
                  "context", "context", 31);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 31);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 31);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c10_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p!eml_fldiv"),
                  "context", "context", 32);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("abs"), "name", "name", 32);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 32);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c10_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 33);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 33);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 33);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c10_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 34);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 34);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 34);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1286840312U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c10_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(""), "context", "context", 35);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("angle"), "name", "name", 35);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/angle.m"), "resolved",
                  "resolved", 35);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1343851970U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c10_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/angle.m"), "context",
                  "context", 36);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_scalar_angle"), "name",
                  "name", 36);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_angle.m"),
                  "resolved", "resolved", 36);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1286840316U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c10_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_angle.m"),
                  "context", "context", 37);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_scalar_atan2"), "name",
                  "name", 37);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan2.m"),
                  "resolved", "resolved", 37);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1286840320U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c10_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 38);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 38);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 38);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c10_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 39);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 39);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 39);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c10_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 40);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  40);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 40);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1376002290U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c10_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 41);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 41);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 41);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c10_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 42);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("coder.internal.blas.xgemm"),
                  "name", "name", 42);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "resolved", "resolved", 42);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c10_rhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 43);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 43);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 43);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c10_rhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 44);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "coder.internal.blas.threshold"), "name", "name", 44);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 44);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c10_rhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 45);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 45);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 45);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1381871900U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c10_rhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 46);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 46);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 46);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c10_rhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs46), "lhs", "lhs",
                  46);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 47);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "coder.internal.refblas.xgemm"), "name", "name", 47);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 47);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "resolved", "resolved", 47);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 47);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 47);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 47);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 47);
  sf_mex_assign(&c10_rhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs47), "rhs", "rhs",
                  47);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs47), "lhs", "lhs",
                  47);
  sf_mex_destroy(&c10_rhs0);
  sf_mex_destroy(&c10_lhs0);
  sf_mex_destroy(&c10_rhs1);
  sf_mex_destroy(&c10_lhs1);
  sf_mex_destroy(&c10_rhs2);
  sf_mex_destroy(&c10_lhs2);
  sf_mex_destroy(&c10_rhs3);
  sf_mex_destroy(&c10_lhs3);
  sf_mex_destroy(&c10_rhs4);
  sf_mex_destroy(&c10_lhs4);
  sf_mex_destroy(&c10_rhs5);
  sf_mex_destroy(&c10_lhs5);
  sf_mex_destroy(&c10_rhs6);
  sf_mex_destroy(&c10_lhs6);
  sf_mex_destroy(&c10_rhs7);
  sf_mex_destroy(&c10_lhs7);
  sf_mex_destroy(&c10_rhs8);
  sf_mex_destroy(&c10_lhs8);
  sf_mex_destroy(&c10_rhs9);
  sf_mex_destroy(&c10_lhs9);
  sf_mex_destroy(&c10_rhs10);
  sf_mex_destroy(&c10_lhs10);
  sf_mex_destroy(&c10_rhs11);
  sf_mex_destroy(&c10_lhs11);
  sf_mex_destroy(&c10_rhs12);
  sf_mex_destroy(&c10_lhs12);
  sf_mex_destroy(&c10_rhs13);
  sf_mex_destroy(&c10_lhs13);
  sf_mex_destroy(&c10_rhs14);
  sf_mex_destroy(&c10_lhs14);
  sf_mex_destroy(&c10_rhs15);
  sf_mex_destroy(&c10_lhs15);
  sf_mex_destroy(&c10_rhs16);
  sf_mex_destroy(&c10_lhs16);
  sf_mex_destroy(&c10_rhs17);
  sf_mex_destroy(&c10_lhs17);
  sf_mex_destroy(&c10_rhs18);
  sf_mex_destroy(&c10_lhs18);
  sf_mex_destroy(&c10_rhs19);
  sf_mex_destroy(&c10_lhs19);
  sf_mex_destroy(&c10_rhs20);
  sf_mex_destroy(&c10_lhs20);
  sf_mex_destroy(&c10_rhs21);
  sf_mex_destroy(&c10_lhs21);
  sf_mex_destroy(&c10_rhs22);
  sf_mex_destroy(&c10_lhs22);
  sf_mex_destroy(&c10_rhs23);
  sf_mex_destroy(&c10_lhs23);
  sf_mex_destroy(&c10_rhs24);
  sf_mex_destroy(&c10_lhs24);
  sf_mex_destroy(&c10_rhs25);
  sf_mex_destroy(&c10_lhs25);
  sf_mex_destroy(&c10_rhs26);
  sf_mex_destroy(&c10_lhs26);
  sf_mex_destroy(&c10_rhs27);
  sf_mex_destroy(&c10_lhs27);
  sf_mex_destroy(&c10_rhs28);
  sf_mex_destroy(&c10_lhs28);
  sf_mex_destroy(&c10_rhs29);
  sf_mex_destroy(&c10_lhs29);
  sf_mex_destroy(&c10_rhs30);
  sf_mex_destroy(&c10_lhs30);
  sf_mex_destroy(&c10_rhs31);
  sf_mex_destroy(&c10_lhs31);
  sf_mex_destroy(&c10_rhs32);
  sf_mex_destroy(&c10_lhs32);
  sf_mex_destroy(&c10_rhs33);
  sf_mex_destroy(&c10_lhs33);
  sf_mex_destroy(&c10_rhs34);
  sf_mex_destroy(&c10_lhs34);
  sf_mex_destroy(&c10_rhs35);
  sf_mex_destroy(&c10_lhs35);
  sf_mex_destroy(&c10_rhs36);
  sf_mex_destroy(&c10_lhs36);
  sf_mex_destroy(&c10_rhs37);
  sf_mex_destroy(&c10_lhs37);
  sf_mex_destroy(&c10_rhs38);
  sf_mex_destroy(&c10_lhs38);
  sf_mex_destroy(&c10_rhs39);
  sf_mex_destroy(&c10_lhs39);
  sf_mex_destroy(&c10_rhs40);
  sf_mex_destroy(&c10_lhs40);
  sf_mex_destroy(&c10_rhs41);
  sf_mex_destroy(&c10_lhs41);
  sf_mex_destroy(&c10_rhs42);
  sf_mex_destroy(&c10_lhs42);
  sf_mex_destroy(&c10_rhs43);
  sf_mex_destroy(&c10_lhs43);
  sf_mex_destroy(&c10_rhs44);
  sf_mex_destroy(&c10_lhs44);
  sf_mex_destroy(&c10_rhs45);
  sf_mex_destroy(&c10_lhs45);
  sf_mex_destroy(&c10_rhs46);
  sf_mex_destroy(&c10_lhs46);
  sf_mex_destroy(&c10_rhs47);
  sf_mex_destroy(&c10_lhs47);
}

static const mxArray *c10_emlrt_marshallOut(const char * c10_u)
{
  const mxArray *c10_y = NULL;
  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", c10_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c10_u)), false);
  return c10_y;
}

static const mxArray *c10_b_emlrt_marshallOut(const uint32_T c10_u)
{
  const mxArray *c10_y = NULL;
  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", &c10_u, 7, 0U, 0U, 0U, 0), false);
  return c10_y;
}

static void c10_fwd_kin(SFc10_quadRotorSimInstanceStruct *chartInstance, real_T
  c10_th1, real_T c10_th2, real_T c10_th3, real_T c10_c_l, real_T c10_c_L,
  real_T c10_c_d, real_T c10_c_d1, real_T c10_EE[3], real_T c10_J[9], real_T
  c10_chain_points[15])
{
  uint32_T c10_debug_family_var_map[41];
  real_T c10_c1;
  real_T c10_c2;
  real_T c10_s1;
  real_T c10_s2;
  real_T c10_xA;
  real_T c10_yA;
  real_T c10_xB;
  real_T c10_yB;
  real_T c10_R;
  real_T c10_S;
  real_T c10_M;
  real_T c10_N;
  real_T c10_a;
  real_T c10_b;
  real_T c10_c;
  real_T c10_Delta;
  real_T c10_y;
  real_T c10_x;
  real_T c10_phi1;
  real_T c10_phi2;
  real_T c10_c3;
  real_T c10_s3;
  real_T c10_Jpant[4];
  real_T c10_Q[6];
  real_T c10_J3[3];
  real_T c10_origin1[3];
  real_T c10_link1[3];
  real_T c10_link2[3];
  real_T c10_origin2[3];
  real_T c10_b_R[9];
  real_T c10_nargin = 7.0;
  real_T c10_nargout = 3.0;
  real_T c10_b_x;
  real_T c10_c_x;
  real_T c10_d_x;
  real_T c10_e_x;
  real_T c10_f_x;
  real_T c10_g_x;
  real_T c10_h_x;
  real_T c10_i_x;
  real_T c10_A;
  real_T c10_B;
  real_T c10_j_x;
  real_T c10_b_y;
  real_T c10_k_x;
  real_T c10_c_y;
  real_T c10_l_x;
  real_T c10_d_y;
  real_T c10_b_A;
  real_T c10_b_B;
  real_T c10_m_x;
  real_T c10_e_y;
  real_T c10_n_x;
  real_T c10_f_y;
  real_T c10_o_x;
  real_T c10_g_y;
  real_T c10_b_a;
  boolean_T c10_b_b;
  real_T c10_d5;
  real_T c10_c_A;
  real_T c10_c_B;
  real_T c10_p_x;
  real_T c10_h_y;
  real_T c10_q_x;
  real_T c10_i_y;
  real_T c10_r_x;
  real_T c10_j_y;
  real_T c10_d_A;
  real_T c10_d_B;
  real_T c10_s_x;
  real_T c10_k_y;
  real_T c10_t_x;
  real_T c10_l_y;
  real_T c10_u_x;
  real_T c10_m_y;
  real_T c10_n_y;
  real_T c10_c_b;
  static creal_T c10_dc0 = { 0.0, 1.0 };

  creal_T c10_o_y;
  real_T c10_e_B;
  real_T c10_p_y;
  real_T c10_q_y;
  real_T c10_r_y;
  creal_T c10_s_y;
  real_T c10_ar;
  real_T c10_ai;
  real_T c10_br;
  real_T c10_bi;
  real_T c10_brm;
  real_T c10_bim;
  real_T c10_s;
  real_T c10_d_d;
  real_T c10_nr;
  real_T c10_ni;
  real_T c10_sgnbr;
  real_T c10_sgnbi;
  creal_T c10_t_y;
  real_T c10_e_A;
  real_T c10_f_B;
  real_T c10_v_x;
  real_T c10_u_y;
  real_T c10_w_x;
  real_T c10_v_y;
  real_T c10_x_x;
  real_T c10_w_y;
  real_T c10_x_y;
  real_T c10_d_b;
  real_T c10_g_B;
  real_T c10_y_y;
  real_T c10_ab_y;
  real_T c10_bb_y;
  creal_T c10_cb_y;
  real_T c10_b_ar;
  real_T c10_b_ai;
  real_T c10_b_br;
  real_T c10_b_bi;
  real_T c10_b_brm;
  real_T c10_b_bim;
  real_T c10_b_s;
  real_T c10_e_d;
  real_T c10_b_nr;
  real_T c10_b_ni;
  real_T c10_b_sgnbr;
  real_T c10_b_sgnbi;
  creal_T c10_db_y;
  real_T c10_y_x;
  real_T c10_ab_x;
  real_T c10_bb_x;
  real_T c10_cb_x;
  int32_T c10_i119;
  int32_T c10_i120;
  static real_T c10_dv10[3] = { 1.0, 0.0, 0.0 };

  int32_T c10_i121;
  real_T c10_c_a[9];
  real_T c10_e_b[3];
  int32_T c10_i122;
  int32_T c10_i123;
  int32_T c10_i124;
  real_T c10_C[3];
  int32_T c10_i125;
  int32_T c10_i126;
  int32_T c10_i127;
  int32_T c10_i128;
  int32_T c10_i129;
  int32_T c10_i130;
  real_T c10_db_x;
  real_T c10_eb_x;
  real_T c10_f_A;
  real_T c10_h_B;
  real_T c10_fb_x;
  real_T c10_eb_y;
  real_T c10_gb_x;
  real_T c10_fb_y;
  real_T c10_hb_x;
  real_T c10_gb_y;
  real_T c10_hb_y;
  real_T c10_ib_x;
  real_T c10_jb_x;
  real_T c10_kb_x;
  real_T c10_lb_x;
  real_T c10_mb_x;
  real_T c10_nb_x;
  real_T c10_ob_x;
  real_T c10_pb_x;
  real_T c10_qb_x;
  real_T c10_rb_x;
  real_T c10_sb_x;
  real_T c10_tb_x;
  real_T c10_ub_x;
  real_T c10_vb_x;
  real_T c10_wb_x;
  real_T c10_xb_x;
  real_T c10_yb_x;
  real_T c10_ac_x;
  real_T c10_bc_x;
  real_T c10_cc_x;
  real_T c10_d_a;
  real_T c10_f_b[4];
  int32_T c10_i131;
  int32_T c10_i132;
  int32_T c10_i133;
  int32_T c10_i134;
  real_T c10_e_a[6];
  int32_T c10_i135;
  int32_T c10_i136;
  int32_T c10_i137;
  int32_T c10_i138;
  int32_T c10_i139;
  int32_T c10_i140;
  int32_T c10_i141;
  int32_T c10_i142;
  real_T c10_ib_y[6];
  int32_T c10_i143;
  int32_T c10_i144;
  int32_T c10_i145;
  int32_T c10_i146;
  int32_T c10_i147;
  int32_T c10_i148;
  int32_T c10_i149;
  int32_T c10_i150;
  int32_T c10_i151;
  int32_T c10_i152;
  int32_T c10_i153;
  int32_T c10_i154;
  int32_T c10_i155;
  int32_T c10_i156;
  int32_T c10_i157;
  int32_T c10_i158;
  int32_T c10_i159;
  int32_T c10_i160;
  int32_T c10_i161;
  int32_T c10_i162;
  int32_T c10_i163;
  int32_T c10_i164;
  int32_T c10_i165;
  int32_T c10_i166;
  int32_T c10_i167;
  int32_T c10_i168;
  int32_T c10_i169;
  int32_T c10_i170;
  int32_T c10_i171;
  int32_T c10_i172;
  int32_T c10_i173;
  int32_T c10_i174;
  int32_T c10_i175;
  int32_T c10_i176;
  int32_T c10_i177;
  int32_T c10_i178;
  int32_T c10_i179;
  int32_T c10_i180;
  int32_T c10_i181;
  int32_T c10_i182;
  int32_T c10_i183;
  int32_T c10_i184;
  int32_T c10_i185;
  int32_T c10_i186;
  int32_T c10_i187;
  int32_T c10_i188;
  int32_T c10_i189;
  int32_T c10_i190;
  int32_T c10_i191;
  int32_T c10_i192;
  int32_T c10_i193;
  int32_T c10_i194;
  int32_T c10_i195;
  int32_T c10_i196;
  int32_T c10_i197;
  int32_T c10_i198;
  int32_T c10_i199;
  int32_T c10_i200;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 41U, 42U, c10_b_debug_family_names,
    c10_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_c1, 0U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_c2, 1U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_s1, 2U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_s2, 3U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_xA, 4U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_yA, 5U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_xB, 6U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_yB, 7U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_R, MAX_uint32_T,
    c10_e_sf_marshallOut, c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_S, 9U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_M, 10U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_N, 11U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_a, 12U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_b, 13U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_c, 14U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_Delta, 15U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_y, 16U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_x, 17U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_phi1, 18U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_phi2, 19U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_c3, 20U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_s3, 21U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_Jpant, 22U, c10_g_sf_marshallOut,
    c10_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_Q, 23U, c10_f_sf_marshallOut,
    c10_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_J3, 24U, c10_b_sf_marshallOut,
    c10_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_origin1, 25U, c10_b_sf_marshallOut,
    c10_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_link1, 26U, c10_b_sf_marshallOut,
    c10_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_link2, 27U, c10_b_sf_marshallOut,
    c10_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_origin2, 28U, c10_b_sf_marshallOut,
    c10_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_b_R, MAX_uint32_T,
    c10_c_sf_marshallOut, c10_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_nargin, 29U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_nargout, 30U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_th1, 31U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_th2, 32U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_th3, 33U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_c_l, 34U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_c_L, 35U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_c_d, 36U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_c_d1, 37U, c10_e_sf_marshallOut,
    c10_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_EE, 38U, c10_b_sf_marshallOut,
    c10_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_J, 39U, c10_c_sf_marshallOut,
    c10_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_chain_points, 40U, c10_sf_marshallOut,
    c10_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 33);
  c10_b_x = c10_th1;
  c10_c1 = c10_b_x;
  c10_c_x = c10_c1;
  c10_c1 = c10_c_x;
  c10_c1 = muDoubleScalarCos(c10_c1);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 34);
  c10_d_x = c10_th2;
  c10_c2 = c10_d_x;
  c10_e_x = c10_c2;
  c10_c2 = c10_e_x;
  c10_c2 = muDoubleScalarCos(c10_c2);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 35);
  c10_f_x = c10_th1;
  c10_s1 = c10_f_x;
  c10_g_x = c10_s1;
  c10_s1 = c10_g_x;
  c10_s1 = muDoubleScalarSin(c10_s1);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 36);
  c10_h_x = c10_th2;
  c10_s2 = c10_h_x;
  c10_i_x = c10_s2;
  c10_s2 = c10_i_x;
  c10_s2 = muDoubleScalarSin(c10_s2);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 37);
  c10_xA = c10_c_l * c10_c1;
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 37);
  c10_yA = c10_c_l * c10_s1;
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 38);
  c10_xB = c10_c_d + c10_c_l * c10_c2;
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 38);
  c10_yB = c10_c_l * c10_s2;
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 39);
  c10_R = c10_mpower(chartInstance, c10_xA) + c10_mpower(chartInstance, c10_yA);
  _SFD_SYMBOL_SWITCH(8U, 8U);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 40);
  c10_S = c10_mpower(chartInstance, c10_xB) + c10_mpower(chartInstance, c10_yB);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 41);
  c10_A = c10_yA - c10_yB;
  c10_B = c10_xB - c10_xA;
  c10_j_x = c10_A;
  c10_b_y = c10_B;
  c10_k_x = c10_j_x;
  c10_c_y = c10_b_y;
  c10_l_x = c10_k_x;
  c10_d_y = c10_c_y;
  c10_M = c10_l_x / c10_d_y;
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 42);
  c10_b_A = 0.5 * (c10_S - c10_R);
  c10_b_B = c10_xB - c10_xA;
  c10_m_x = c10_b_A;
  c10_e_y = c10_b_B;
  c10_n_x = c10_m_x;
  c10_f_y = c10_e_y;
  c10_o_x = c10_n_x;
  c10_g_y = c10_f_y;
  c10_N = c10_o_x / c10_g_y;
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 43);
  c10_a = c10_mpower(chartInstance, c10_M) + 1.0;
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 44);
  c10_b = 2.0 * ((c10_M * c10_N - c10_M * c10_xA) - c10_yA);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 45);
  c10_c = ((c10_mpower(chartInstance, c10_N) - 2.0 * c10_N * c10_xA) + c10_R) -
    c10_mpower(chartInstance, c10_c_L);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 46);
  c10_Delta = c10_mpower(chartInstance, c10_b) - 4.0 * c10_a * c10_c;
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 47);
  c10_b_a = c10_Delta;
  c10_b_b = (c10_Delta > 0.0);
  c10_Delta = c10_b_a * (real_T)c10_b_b;
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 48);
  c10_d5 = c10_Delta;
  c10_b_sqrt(chartInstance, &c10_d5);
  c10_c_A = -c10_b + c10_d5;
  c10_c_B = 2.0 * c10_a;
  c10_p_x = c10_c_A;
  c10_h_y = c10_c_B;
  c10_q_x = c10_p_x;
  c10_i_y = c10_h_y;
  c10_r_x = c10_q_x;
  c10_j_y = c10_i_y;
  c10_y = c10_r_x / c10_j_y;
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 49);
  c10_x = c10_M * c10_y + c10_N;
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 50);
  c10_d_A = c10_x - c10_c_l * c10_c1;
  c10_d_B = c10_c_L;
  c10_s_x = c10_d_A;
  c10_k_y = c10_d_B;
  c10_t_x = c10_s_x;
  c10_l_y = c10_k_y;
  c10_u_x = c10_t_x;
  c10_m_y = c10_l_y;
  c10_n_y = c10_u_x / c10_m_y;
  c10_c_b = c10_y - c10_c_l * c10_s1;
  c10_o_y.re = c10_c_b * c10_dc0.re;
  c10_o_y.im = c10_c_b * c10_dc0.im;
  c10_e_B = c10_c_L;
  c10_p_y = c10_e_B;
  c10_q_y = c10_p_y;
  c10_r_y = c10_q_y;
  c10_s_y = c10_o_y;
  c10_ar = c10_s_y.re;
  c10_ai = c10_s_y.im;
  c10_br = c10_r_y;
  c10_bi = 0.0;
  if (c10_bi == 0.0) {
    if (c10_ai == 0.0) {
      c10_o_y.re = c10_ar / c10_br;
      c10_o_y.im = 0.0;
    } else if (c10_ar == 0.0) {
      c10_o_y.re = 0.0;
      c10_o_y.im = c10_ai / c10_br;
    } else {
      c10_o_y.re = c10_ar / c10_br;
      c10_o_y.im = c10_ai / c10_br;
    }
  } else if (c10_br == 0.0) {
    if (c10_ar == 0.0) {
      c10_o_y.re = c10_ai / c10_bi;
      c10_o_y.im = 0.0;
    } else if (c10_ai == 0.0) {
      c10_o_y.re = 0.0;
      c10_o_y.im = -(c10_ar / c10_bi);
    } else {
      c10_o_y.re = c10_ai / c10_bi;
      c10_o_y.im = -(c10_ar / c10_bi);
    }
  } else {
    c10_brm = muDoubleScalarAbs(c10_br);
    c10_bim = muDoubleScalarAbs(c10_bi);
    if (c10_brm > c10_bim) {
      c10_s = c10_bi / c10_br;
      c10_d_d = c10_br + c10_s * c10_bi;
      c10_nr = c10_ar + c10_s * c10_ai;
      c10_ni = c10_ai - c10_s * c10_ar;
      c10_o_y.re = c10_nr / c10_d_d;
      c10_o_y.im = c10_ni / c10_d_d;
    } else if (c10_bim == c10_brm) {
      if (c10_br > 0.0) {
        c10_sgnbr = 0.5;
      } else {
        c10_sgnbr = -0.5;
      }

      if (c10_bi > 0.0) {
        c10_sgnbi = 0.5;
      } else {
        c10_sgnbi = -0.5;
      }

      c10_nr = c10_ar * c10_sgnbr + c10_ai * c10_sgnbi;
      c10_ni = c10_ai * c10_sgnbr - c10_ar * c10_sgnbi;
      c10_o_y.re = c10_nr / c10_brm;
      c10_o_y.im = c10_ni / c10_brm;
    } else {
      c10_s = c10_br / c10_bi;
      c10_d_d = c10_bi + c10_s * c10_br;
      c10_nr = c10_s * c10_ar + c10_ai;
      c10_ni = c10_s * c10_ai - c10_ar;
      c10_o_y.re = c10_nr / c10_d_d;
      c10_o_y.im = c10_ni / c10_d_d;
    }
  }

  c10_t_y.re = c10_n_y + c10_o_y.re;
  c10_t_y.im = c10_o_y.im;
  c10_phi1 = c10_angle(chartInstance, c10_t_y);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 51);
  c10_e_A = (c10_x - c10_c_d) - c10_c_l * c10_c2;
  c10_f_B = c10_c_L;
  c10_v_x = c10_e_A;
  c10_u_y = c10_f_B;
  c10_w_x = c10_v_x;
  c10_v_y = c10_u_y;
  c10_x_x = c10_w_x;
  c10_w_y = c10_v_y;
  c10_x_y = c10_x_x / c10_w_y;
  c10_d_b = c10_y - c10_c_l * c10_s2;
  c10_o_y.re = c10_d_b * c10_dc0.re;
  c10_o_y.im = c10_d_b * c10_dc0.im;
  c10_g_B = c10_c_L;
  c10_y_y = c10_g_B;
  c10_ab_y = c10_y_y;
  c10_bb_y = c10_ab_y;
  c10_cb_y = c10_o_y;
  c10_b_ar = c10_cb_y.re;
  c10_b_ai = c10_cb_y.im;
  c10_b_br = c10_bb_y;
  c10_b_bi = 0.0;
  if (c10_b_bi == 0.0) {
    if (c10_b_ai == 0.0) {
      c10_o_y.re = c10_b_ar / c10_b_br;
      c10_o_y.im = 0.0;
    } else if (c10_b_ar == 0.0) {
      c10_o_y.re = 0.0;
      c10_o_y.im = c10_b_ai / c10_b_br;
    } else {
      c10_o_y.re = c10_b_ar / c10_b_br;
      c10_o_y.im = c10_b_ai / c10_b_br;
    }
  } else if (c10_b_br == 0.0) {
    if (c10_b_ar == 0.0) {
      c10_o_y.re = c10_b_ai / c10_b_bi;
      c10_o_y.im = 0.0;
    } else if (c10_b_ai == 0.0) {
      c10_o_y.re = 0.0;
      c10_o_y.im = -(c10_b_ar / c10_b_bi);
    } else {
      c10_o_y.re = c10_b_ai / c10_b_bi;
      c10_o_y.im = -(c10_b_ar / c10_b_bi);
    }
  } else {
    c10_b_brm = muDoubleScalarAbs(c10_b_br);
    c10_b_bim = muDoubleScalarAbs(c10_b_bi);
    if (c10_b_brm > c10_b_bim) {
      c10_b_s = c10_b_bi / c10_b_br;
      c10_e_d = c10_b_br + c10_b_s * c10_b_bi;
      c10_b_nr = c10_b_ar + c10_b_s * c10_b_ai;
      c10_b_ni = c10_b_ai - c10_b_s * c10_b_ar;
      c10_o_y.re = c10_b_nr / c10_e_d;
      c10_o_y.im = c10_b_ni / c10_e_d;
    } else if (c10_b_bim == c10_b_brm) {
      if (c10_b_br > 0.0) {
        c10_b_sgnbr = 0.5;
      } else {
        c10_b_sgnbr = -0.5;
      }

      if (c10_b_bi > 0.0) {
        c10_b_sgnbi = 0.5;
      } else {
        c10_b_sgnbi = -0.5;
      }

      c10_b_nr = c10_b_ar * c10_b_sgnbr + c10_b_ai * c10_b_sgnbi;
      c10_b_ni = c10_b_ai * c10_b_sgnbr - c10_b_ar * c10_b_sgnbi;
      c10_o_y.re = c10_b_nr / c10_b_brm;
      c10_o_y.im = c10_b_ni / c10_b_brm;
    } else {
      c10_b_s = c10_b_br / c10_b_bi;
      c10_e_d = c10_b_bi + c10_b_s * c10_b_br;
      c10_b_nr = c10_b_s * c10_b_ar + c10_b_ai;
      c10_b_ni = c10_b_s * c10_b_ai - c10_b_ar;
      c10_o_y.re = c10_b_nr / c10_e_d;
      c10_o_y.im = c10_b_ni / c10_e_d;
    }
  }

  c10_db_y.re = c10_x_y + c10_o_y.re;
  c10_db_y.im = c10_o_y.im;
  c10_phi2 = c10_angle(chartInstance, c10_db_y);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 53);
  c10_y_x = c10_th3;
  c10_c3 = c10_y_x;
  c10_ab_x = c10_c3;
  c10_c3 = c10_ab_x;
  c10_c3 = muDoubleScalarCos(c10_c3);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 54);
  c10_bb_x = c10_th3;
  c10_s3 = c10_bb_x;
  c10_cb_x = c10_s3;
  c10_s3 = c10_cb_x;
  c10_s3 = muDoubleScalarSin(c10_s3);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 57);
  c10_i119 = 0;
  for (c10_i120 = 0; c10_i120 < 3; c10_i120++) {
    c10_b_R[c10_i119] = c10_dv10[c10_i120];
    c10_i119 += 3;
  }

  c10_b_R[1] = 0.0;
  c10_b_R[4] = c10_c3;
  c10_b_R[7] = -c10_s3;
  c10_b_R[2] = 0.0;
  c10_b_R[5] = c10_s3;
  c10_b_R[8] = c10_c3;
  _SFD_SYMBOL_SWITCH(8U, 29U);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 61);
  for (c10_i121 = 0; c10_i121 < 9; c10_i121++) {
    c10_c_a[c10_i121] = c10_b_R[c10_i121];
  }

  c10_e_b[0] = c10_x;
  c10_e_b[1] = c10_y;
  c10_e_b[2] = c10_c_d1;
  c10_b_eml_scalar_eg(chartInstance);
  c10_b_eml_scalar_eg(chartInstance);
  for (c10_i122 = 0; c10_i122 < 3; c10_i122++) {
    c10_EE[c10_i122] = 0.0;
  }

  for (c10_i123 = 0; c10_i123 < 3; c10_i123++) {
    c10_EE[c10_i123] = 0.0;
  }

  for (c10_i124 = 0; c10_i124 < 3; c10_i124++) {
    c10_C[c10_i124] = c10_EE[c10_i124];
  }

  for (c10_i125 = 0; c10_i125 < 3; c10_i125++) {
    c10_EE[c10_i125] = c10_C[c10_i125];
  }

  c10_threshold(chartInstance);
  for (c10_i126 = 0; c10_i126 < 3; c10_i126++) {
    c10_C[c10_i126] = c10_EE[c10_i126];
  }

  for (c10_i127 = 0; c10_i127 < 3; c10_i127++) {
    c10_EE[c10_i127] = c10_C[c10_i127];
  }

  for (c10_i128 = 0; c10_i128 < 3; c10_i128++) {
    c10_EE[c10_i128] = 0.0;
    c10_i129 = 0;
    for (c10_i130 = 0; c10_i130 < 3; c10_i130++) {
      c10_EE[c10_i128] += c10_c_a[c10_i129 + c10_i128] * c10_e_b[c10_i130];
      c10_i129 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 64);
  c10_db_x = c10_phi2 - c10_phi1;
  c10_eb_x = c10_db_x;
  c10_eb_x = muDoubleScalarSin(c10_eb_x);
  c10_f_A = -c10_c_l;
  c10_h_B = c10_eb_x;
  c10_fb_x = c10_f_A;
  c10_eb_y = c10_h_B;
  c10_gb_x = c10_fb_x;
  c10_fb_y = c10_eb_y;
  c10_hb_x = c10_gb_x;
  c10_gb_y = c10_fb_y;
  c10_hb_y = c10_hb_x / c10_gb_y;
  c10_ib_x = c10_phi2 - c10_phi1;
  c10_jb_x = c10_ib_x;
  c10_jb_x = muDoubleScalarSin(c10_jb_x);
  c10_kb_x = c10_phi1;
  c10_lb_x = c10_kb_x;
  c10_lb_x = muDoubleScalarSin(c10_lb_x);
  c10_mb_x = c10_th1 - c10_phi2;
  c10_nb_x = c10_mb_x;
  c10_nb_x = muDoubleScalarSin(c10_nb_x);
  c10_ob_x = c10_phi1;
  c10_pb_x = c10_ob_x;
  c10_pb_x = muDoubleScalarSin(c10_pb_x);
  c10_qb_x = c10_th2 - c10_phi2;
  c10_rb_x = c10_qb_x;
  c10_rb_x = muDoubleScalarSin(c10_rb_x);
  c10_sb_x = c10_phi2 - c10_phi1;
  c10_tb_x = c10_sb_x;
  c10_tb_x = muDoubleScalarSin(c10_tb_x);
  c10_ub_x = c10_phi1;
  c10_vb_x = c10_ub_x;
  c10_vb_x = muDoubleScalarCos(c10_vb_x);
  c10_wb_x = c10_th1 - c10_phi2;
  c10_xb_x = c10_wb_x;
  c10_xb_x = muDoubleScalarSin(c10_xb_x);
  c10_yb_x = c10_phi1;
  c10_ac_x = c10_yb_x;
  c10_ac_x = muDoubleScalarCos(c10_ac_x);
  c10_bc_x = c10_th2 - c10_phi2;
  c10_cc_x = c10_bc_x;
  c10_cc_x = muDoubleScalarSin(c10_cc_x);
  c10_d_a = c10_hb_y;
  c10_f_b[0] = c10_s1 * c10_jb_x + c10_lb_x * c10_nb_x;
  c10_f_b[2] = -c10_pb_x * c10_rb_x;
  c10_f_b[1] = -c10_c1 * c10_tb_x - c10_vb_x * c10_xb_x;
  c10_f_b[3] = c10_ac_x * c10_cc_x;
  for (c10_i131 = 0; c10_i131 < 4; c10_i131++) {
    c10_Jpant[c10_i131] = c10_d_a * c10_f_b[c10_i131];
  }

  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 66);
  c10_i132 = 0;
  for (c10_i133 = 0; c10_i133 < 2; c10_i133++) {
    c10_Q[c10_i132] = 1.0 - (real_T)c10_i133;
    c10_i132 += 3;
  }

  c10_Q[1] = 0.0;
  c10_Q[4] = c10_c3;
  c10_Q[2] = 0.0;
  c10_Q[5] = c10_s3;
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 67);
  c10_J3[0] = 0.0;
  c10_J3[1] = -c10_s3 * c10_y - c10_c3 * c10_c_d1;
  c10_J3[2] = c10_c3 * c10_y - c10_s3 * c10_c_d1;
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 69);
  for (c10_i134 = 0; c10_i134 < 6; c10_i134++) {
    c10_e_a[c10_i134] = c10_Q[c10_i134];
  }

  c10_i135 = 0;
  for (c10_i136 = 0; c10_i136 < 2; c10_i136++) {
    c10_i137 = 0;
    for (c10_i138 = 0; c10_i138 < 2; c10_i138++) {
      c10_f_b[c10_i138 + c10_i135] = c10_Jpant[c10_i137 + c10_i136];
      c10_i137 += 2;
    }

    c10_i135 += 2;
  }

  c10_c_eml_scalar_eg(chartInstance);
  c10_c_eml_scalar_eg(chartInstance);
  c10_threshold(chartInstance);
  for (c10_i139 = 0; c10_i139 < 3; c10_i139++) {
    c10_i140 = 0;
    c10_i141 = 0;
    for (c10_i142 = 0; c10_i142 < 2; c10_i142++) {
      c10_ib_y[c10_i140 + c10_i139] = 0.0;
      c10_i143 = 0;
      for (c10_i144 = 0; c10_i144 < 2; c10_i144++) {
        c10_ib_y[c10_i140 + c10_i139] += c10_e_a[c10_i143 + c10_i139] *
          c10_f_b[c10_i144 + c10_i141];
        c10_i143 += 3;
      }

      c10_i140 += 3;
      c10_i141 += 2;
    }
  }

  c10_i145 = 0;
  for (c10_i146 = 0; c10_i146 < 3; c10_i146++) {
    c10_i147 = 0;
    for (c10_i148 = 0; c10_i148 < 2; c10_i148++) {
      c10_J[c10_i148 + c10_i145] = c10_ib_y[c10_i147 + c10_i146];
      c10_i147 += 3;
    }

    c10_i145 += 3;
  }

  c10_i149 = 0;
  for (c10_i150 = 0; c10_i150 < 3; c10_i150++) {
    c10_J[c10_i149 + 2] = c10_J3[c10_i150];
    c10_i149 += 3;
  }

  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 75);
  for (c10_i151 = 0; c10_i151 < 9; c10_i151++) {
    c10_c_a[c10_i151] = c10_b_R[c10_i151];
  }

  c10_e_b[0] = 0.0;
  c10_e_b[1] = 0.0;
  c10_e_b[2] = c10_c_d1;
  c10_b_eml_scalar_eg(chartInstance);
  c10_b_eml_scalar_eg(chartInstance);
  for (c10_i152 = 0; c10_i152 < 3; c10_i152++) {
    c10_origin1[c10_i152] = 0.0;
  }

  for (c10_i153 = 0; c10_i153 < 3; c10_i153++) {
    c10_origin1[c10_i153] = 0.0;
  }

  for (c10_i154 = 0; c10_i154 < 3; c10_i154++) {
    c10_C[c10_i154] = c10_origin1[c10_i154];
  }

  for (c10_i155 = 0; c10_i155 < 3; c10_i155++) {
    c10_origin1[c10_i155] = c10_C[c10_i155];
  }

  c10_threshold(chartInstance);
  for (c10_i156 = 0; c10_i156 < 3; c10_i156++) {
    c10_C[c10_i156] = c10_origin1[c10_i156];
  }

  for (c10_i157 = 0; c10_i157 < 3; c10_i157++) {
    c10_origin1[c10_i157] = c10_C[c10_i157];
  }

  for (c10_i158 = 0; c10_i158 < 3; c10_i158++) {
    c10_origin1[c10_i158] = 0.0;
    c10_i159 = 0;
    for (c10_i160 = 0; c10_i160 < 3; c10_i160++) {
      c10_origin1[c10_i158] += c10_c_a[c10_i159 + c10_i158] * c10_e_b[c10_i160];
      c10_i159 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 76);
  for (c10_i161 = 0; c10_i161 < 9; c10_i161++) {
    c10_c_a[c10_i161] = c10_b_R[c10_i161];
  }

  c10_e_b[0] = c10_xA;
  c10_e_b[1] = c10_yA;
  c10_e_b[2] = c10_c_d1;
  c10_b_eml_scalar_eg(chartInstance);
  c10_b_eml_scalar_eg(chartInstance);
  for (c10_i162 = 0; c10_i162 < 3; c10_i162++) {
    c10_link1[c10_i162] = 0.0;
  }

  for (c10_i163 = 0; c10_i163 < 3; c10_i163++) {
    c10_link1[c10_i163] = 0.0;
  }

  for (c10_i164 = 0; c10_i164 < 3; c10_i164++) {
    c10_C[c10_i164] = c10_link1[c10_i164];
  }

  for (c10_i165 = 0; c10_i165 < 3; c10_i165++) {
    c10_link1[c10_i165] = c10_C[c10_i165];
  }

  c10_threshold(chartInstance);
  for (c10_i166 = 0; c10_i166 < 3; c10_i166++) {
    c10_C[c10_i166] = c10_link1[c10_i166];
  }

  for (c10_i167 = 0; c10_i167 < 3; c10_i167++) {
    c10_link1[c10_i167] = c10_C[c10_i167];
  }

  for (c10_i168 = 0; c10_i168 < 3; c10_i168++) {
    c10_link1[c10_i168] = 0.0;
    c10_i169 = 0;
    for (c10_i170 = 0; c10_i170 < 3; c10_i170++) {
      c10_link1[c10_i168] += c10_c_a[c10_i169 + c10_i168] * c10_e_b[c10_i170];
      c10_i169 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 77);
  for (c10_i171 = 0; c10_i171 < 9; c10_i171++) {
    c10_c_a[c10_i171] = c10_b_R[c10_i171];
  }

  c10_e_b[0] = c10_xB;
  c10_e_b[1] = c10_yB;
  c10_e_b[2] = c10_c_d1;
  c10_b_eml_scalar_eg(chartInstance);
  c10_b_eml_scalar_eg(chartInstance);
  for (c10_i172 = 0; c10_i172 < 3; c10_i172++) {
    c10_link2[c10_i172] = 0.0;
  }

  for (c10_i173 = 0; c10_i173 < 3; c10_i173++) {
    c10_link2[c10_i173] = 0.0;
  }

  for (c10_i174 = 0; c10_i174 < 3; c10_i174++) {
    c10_C[c10_i174] = c10_link2[c10_i174];
  }

  for (c10_i175 = 0; c10_i175 < 3; c10_i175++) {
    c10_link2[c10_i175] = c10_C[c10_i175];
  }

  c10_threshold(chartInstance);
  for (c10_i176 = 0; c10_i176 < 3; c10_i176++) {
    c10_C[c10_i176] = c10_link2[c10_i176];
  }

  for (c10_i177 = 0; c10_i177 < 3; c10_i177++) {
    c10_link2[c10_i177] = c10_C[c10_i177];
  }

  for (c10_i178 = 0; c10_i178 < 3; c10_i178++) {
    c10_link2[c10_i178] = 0.0;
    c10_i179 = 0;
    for (c10_i180 = 0; c10_i180 < 3; c10_i180++) {
      c10_link2[c10_i178] += c10_c_a[c10_i179 + c10_i178] * c10_e_b[c10_i180];
      c10_i179 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 78);
  for (c10_i181 = 0; c10_i181 < 9; c10_i181++) {
    c10_c_a[c10_i181] = c10_b_R[c10_i181];
  }

  c10_e_b[0] = c10_c_d;
  c10_e_b[1] = 0.0;
  c10_e_b[2] = c10_c_d1;
  c10_b_eml_scalar_eg(chartInstance);
  c10_b_eml_scalar_eg(chartInstance);
  for (c10_i182 = 0; c10_i182 < 3; c10_i182++) {
    c10_origin2[c10_i182] = 0.0;
  }

  for (c10_i183 = 0; c10_i183 < 3; c10_i183++) {
    c10_origin2[c10_i183] = 0.0;
  }

  for (c10_i184 = 0; c10_i184 < 3; c10_i184++) {
    c10_C[c10_i184] = c10_origin2[c10_i184];
  }

  for (c10_i185 = 0; c10_i185 < 3; c10_i185++) {
    c10_origin2[c10_i185] = c10_C[c10_i185];
  }

  c10_threshold(chartInstance);
  for (c10_i186 = 0; c10_i186 < 3; c10_i186++) {
    c10_C[c10_i186] = c10_origin2[c10_i186];
  }

  for (c10_i187 = 0; c10_i187 < 3; c10_i187++) {
    c10_origin2[c10_i187] = c10_C[c10_i187];
  }

  for (c10_i188 = 0; c10_i188 < 3; c10_i188++) {
    c10_origin2[c10_i188] = 0.0;
    c10_i189 = 0;
    for (c10_i190 = 0; c10_i190 < 3; c10_i190++) {
      c10_origin2[c10_i188] += c10_c_a[c10_i189 + c10_i188] * c10_e_b[c10_i190];
      c10_i189 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 80);
  c10_i191 = 0;
  for (c10_i192 = 0; c10_i192 < 3; c10_i192++) {
    c10_chain_points[c10_i191] = c10_origin1[c10_i192];
    c10_i191 += 5;
  }

  c10_i193 = 0;
  for (c10_i194 = 0; c10_i194 < 3; c10_i194++) {
    c10_chain_points[c10_i193 + 1] = c10_link1[c10_i194];
    c10_i193 += 5;
  }

  c10_i195 = 0;
  for (c10_i196 = 0; c10_i196 < 3; c10_i196++) {
    c10_chain_points[c10_i195 + 2] = c10_EE[c10_i196];
    c10_i195 += 5;
  }

  c10_i197 = 0;
  for (c10_i198 = 0; c10_i198 < 3; c10_i198++) {
    c10_chain_points[c10_i197 + 3] = c10_link2[c10_i198];
    c10_i197 += 5;
  }

  c10_i199 = 0;
  for (c10_i200 = 0; c10_i200 < 3; c10_i200++) {
    c10_chain_points[c10_i199 + 4] = c10_origin2[c10_i200];
    c10_i199 += 5;
  }

  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, -80);
  _SFD_SYMBOL_SCOPE_POP();
}

static real_T c10_mpower(SFc10_quadRotorSimInstanceStruct *chartInstance, real_T
  c10_a)
{
  real_T c10_b_a;
  real_T c10_c_a;
  real_T c10_ak;
  real_T c10_d_a;
  c10_b_a = c10_a;
  c10_c_a = c10_b_a;
  c10_eml_scalar_eg(chartInstance);
  c10_ak = c10_c_a;
  c10_d_a = c10_ak;
  c10_eml_scalar_eg(chartInstance);
  return c10_d_a * c10_d_a;
}

static void c10_eml_scalar_eg(SFc10_quadRotorSimInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real_T c10_sqrt(SFc10_quadRotorSimInstanceStruct *chartInstance, real_T
  c10_x)
{
  real_T c10_b_x;
  c10_b_x = c10_x;
  c10_b_sqrt(chartInstance, &c10_b_x);
  return c10_b_x;
}

static void c10_eml_error(SFc10_quadRotorSimInstanceStruct *chartInstance)
{
  int32_T c10_i201;
  static char_T c10_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c10_u[30];
  const mxArray *c10_y = NULL;
  int32_T c10_i202;
  static char_T c10_cv1[4] = { 's', 'q', 'r', 't' };

  char_T c10_b_u[4];
  const mxArray *c10_b_y = NULL;
  (void)chartInstance;
  for (c10_i201 = 0; c10_i201 < 30; c10_i201++) {
    c10_u[c10_i201] = c10_cv0[c10_i201];
  }

  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", c10_u, 10, 0U, 1U, 0U, 2, 1, 30),
                false);
  for (c10_i202 = 0; c10_i202 < 4; c10_i202++) {
    c10_b_u[c10_i202] = c10_cv1[c10_i202];
  }

  c10_b_y = NULL;
  sf_mex_assign(&c10_b_y, sf_mex_create("y", c10_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c10_y, 14, c10_b_y));
}

static real_T c10_angle(SFc10_quadRotorSimInstanceStruct *chartInstance, creal_T
  c10_x)
{
  real_T c10_b_y;
  real_T c10_b_x;
  (void)chartInstance;
  c10_b_y = c10_x.im;
  c10_b_x = c10_x.re;
  return muDoubleScalarAtan2(c10_b_y, c10_b_x);
}

static void c10_b_eml_scalar_eg(SFc10_quadRotorSimInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c10_threshold(SFc10_quadRotorSimInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c10_c_eml_scalar_eg(SFc10_quadRotorSimInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c10_d_eml_scalar_eg(SFc10_quadRotorSimInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *c10_h_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  int32_T c10_u;
  const mxArray *c10_y = NULL;
  SFc10_quadRotorSimInstanceStruct *chartInstance;
  chartInstance = (SFc10_quadRotorSimInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  c10_u = *(int32_T *)c10_inData;
  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", &c10_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, false);
  return c10_mxArrayOutData;
}

static int32_T c10_j_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId)
{
  int32_T c10_y;
  int32_T c10_i203;
  (void)chartInstance;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), &c10_i203, 1, 6, 0U, 0, 0U, 0);
  c10_y = c10_i203;
  sf_mex_destroy(&c10_u);
  return c10_y;
}

static void c10_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData)
{
  const mxArray *c10_b_sfEvent;
  const char_T *c10_identifier;
  emlrtMsgIdentifier c10_thisId;
  int32_T c10_y;
  SFc10_quadRotorSimInstanceStruct *chartInstance;
  chartInstance = (SFc10_quadRotorSimInstanceStruct *)chartInstanceVoid;
  c10_b_sfEvent = sf_mex_dup(c10_mxArrayInData);
  c10_identifier = c10_varName;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_y = c10_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_b_sfEvent),
    &c10_thisId);
  sf_mex_destroy(&c10_b_sfEvent);
  *(int32_T *)c10_outData = c10_y;
  sf_mex_destroy(&c10_mxArrayInData);
}

static uint8_T c10_k_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_b_is_active_c10_quadRotorSim, const char_T *
  c10_identifier)
{
  uint8_T c10_y;
  emlrtMsgIdentifier c10_thisId;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_y = c10_l_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c10_b_is_active_c10_quadRotorSim), &c10_thisId);
  sf_mex_destroy(&c10_b_is_active_c10_quadRotorSim);
  return c10_y;
}

static uint8_T c10_l_emlrt_marshallIn(SFc10_quadRotorSimInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId)
{
  uint8_T c10_y;
  uint8_T c10_u0;
  (void)chartInstance;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), &c10_u0, 1, 3, 0U, 0, 0U, 0);
  c10_y = c10_u0;
  sf_mex_destroy(&c10_u);
  return c10_y;
}

static void c10_b_sqrt(SFc10_quadRotorSimInstanceStruct *chartInstance, real_T
  *c10_x)
{
  if (*c10_x < 0.0) {
    c10_eml_error(chartInstance);
  }

  *c10_x = muDoubleScalarSqrt(*c10_x);
}

static void init_dsm_address_info(SFc10_quadRotorSimInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c10_quadRotorSim_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(866593575U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2082662492U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2426672453U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2544190994U);
}

mxArray *sf_c10_quadRotorSim_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("uio8uNmGPnrVTlcCB1SujE");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(3);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(5);
      pr[1] = (double)(3);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c10_quadRotorSim_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c10_quadRotorSim_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c10_quadRotorSim(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x5'type','srcId','name','auxInfo'{{M[1],M[19],T\"JR\",},{M[1],M[13],T\"dxyzP\",},{M[1],M[5],T\"xyzP\",},{M[1],M[22],T\"xyz_Device\",},{M[8],M[0],T\"is_active_c10_quadRotorSim\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 5, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c10_quadRotorSim_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc10_quadRotorSimInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc10_quadRotorSimInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _quadRotorSimMachineNumber_,
           10,
           1,
           1,
           0,
           11,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize ist own list of scripts */
        init_script_number_translation(_quadRotorSimMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_quadRotorSimMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _quadRotorSimMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,2,0,1,"JR");
          _SFD_SET_DATA_PROPS(1,1,1,0,"th");
          _SFD_SET_DATA_PROPS(2,1,1,0,"dth");
          _SFD_SET_DATA_PROPS(3,10,0,0,"l");
          _SFD_SET_DATA_PROPS(4,2,0,1,"xyzP");
          _SFD_SET_DATA_PROPS(5,10,0,0,"L");
          _SFD_SET_DATA_PROPS(6,10,0,0,"d");
          _SFD_SET_DATA_PROPS(7,10,0,0,"d1");
          _SFD_SET_DATA_PROPS(8,2,0,1,"dxyzP");
          _SFD_SET_DATA_PROPS(9,1,1,0,"clip_par");
          _SFD_SET_DATA_PROPS(10,2,0,1,"xyz_Device");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,2,0,0,0,0,1,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,758);
        _SFD_CV_INIT_EML_FCN(0,1,"fwd_kin",758,-1,1807);
        _SFD_CV_INIT_EML_FOR(0,1,0,589,618,691);

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c10_c_sf_marshallOut,(MexInFcnForType)
            c10_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c10_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c10_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c10_e_sf_marshallOut,(MexInFcnForType)
          c10_d_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c10_b_sf_marshallOut,(MexInFcnForType)
            c10_b_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c10_e_sf_marshallOut,(MexInFcnForType)
          c10_d_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c10_e_sf_marshallOut,(MexInFcnForType)
          c10_d_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c10_e_sf_marshallOut,(MexInFcnForType)
          c10_d_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c10_b_sf_marshallOut,(MexInFcnForType)
            c10_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c10_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 5;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c10_sf_marshallOut,(MexInFcnForType)
            c10_sf_marshallIn);
        }

        {
          real_T (*c10_JR)[9];
          real_T (*c10_th)[3];
          real_T (*c10_dth)[3];
          real_T (*c10_xyzP)[3];
          real_T (*c10_dxyzP)[3];
          real_T (*c10_clip_par)[4];
          real_T (*c10_xyz_Device)[15];
          c10_xyz_Device = (real_T (*)[15])ssGetOutputPortSignal
            (chartInstance->S, 4);
          c10_clip_par = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 2);
          c10_dxyzP = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 3);
          c10_xyzP = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
          c10_dth = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
          c10_th = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
          c10_JR = (real_T (*)[9])ssGetOutputPortSignal(chartInstance->S, 1);
          _SFD_SET_DATA_VALUE_PTR(0U, *c10_JR);
          _SFD_SET_DATA_VALUE_PTR(1U, *c10_th);
          _SFD_SET_DATA_VALUE_PTR(2U, *c10_dth);
          _SFD_SET_DATA_VALUE_PTR(3U, &chartInstance->c10_l);
          _SFD_SET_DATA_VALUE_PTR(4U, *c10_xyzP);
          _SFD_SET_DATA_VALUE_PTR(5U, &chartInstance->c10_L);
          _SFD_SET_DATA_VALUE_PTR(6U, &chartInstance->c10_d);
          _SFD_SET_DATA_VALUE_PTR(7U, &chartInstance->c10_d1);
          _SFD_SET_DATA_VALUE_PTR(8U, *c10_dxyzP);
          _SFD_SET_DATA_VALUE_PTR(9U, *c10_clip_par);
          _SFD_SET_DATA_VALUE_PTR(10U, *c10_xyz_Device);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _quadRotorSimMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "2nGeczpOMdczJXfd6RaeUE";
}

static void sf_opaque_initialize_c10_quadRotorSim(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc10_quadRotorSimInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c10_quadRotorSim((SFc10_quadRotorSimInstanceStruct*)
    chartInstanceVar);
  initialize_c10_quadRotorSim((SFc10_quadRotorSimInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c10_quadRotorSim(void *chartInstanceVar)
{
  enable_c10_quadRotorSim((SFc10_quadRotorSimInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c10_quadRotorSim(void *chartInstanceVar)
{
  disable_c10_quadRotorSim((SFc10_quadRotorSimInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c10_quadRotorSim(void *chartInstanceVar)
{
  sf_gateway_c10_quadRotorSim((SFc10_quadRotorSimInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c10_quadRotorSim(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c10_quadRotorSim
    ((SFc10_quadRotorSimInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c10_quadRotorSim();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c10_quadRotorSim(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c10_quadRotorSim();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c10_quadRotorSim((SFc10_quadRotorSimInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c10_quadRotorSim(SimStruct* S)
{
  return sf_internal_get_sim_state_c10_quadRotorSim(S);
}

static void sf_opaque_set_sim_state_c10_quadRotorSim(SimStruct* S, const mxArray
  *st)
{
  sf_internal_set_sim_state_c10_quadRotorSim(S, st);
}

static void sf_opaque_terminate_c10_quadRotorSim(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc10_quadRotorSimInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_quadRotorSim_optimization_info();
    }

    finalize_c10_quadRotorSim((SFc10_quadRotorSimInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    if (crtInfo != NULL) {
      utFree((void *)crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc10_quadRotorSim((SFc10_quadRotorSimInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c10_quadRotorSim(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c10_quadRotorSim((SFc10_quadRotorSimInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c10_quadRotorSim(SimStruct *S)
{
  /* Actual parameters from chart:
     L d d1 l
   */
  const char_T *rtParamNames[] = { "L", "d", "d1", "l" };

  ssSetNumRunTimeParams(S,ssGetSFcnParamsCount(S));

  /* registration for L*/
  ssRegDlgParamAsRunTimeParam(S, 0, 0, rtParamNames[0], SS_DOUBLE);

  /* registration for d*/
  ssRegDlgParamAsRunTimeParam(S, 1, 1, rtParamNames[1], SS_DOUBLE);

  /* registration for d1*/
  ssRegDlgParamAsRunTimeParam(S, 2, 2, rtParamNames[2], SS_DOUBLE);

  /* registration for l*/
  ssRegDlgParamAsRunTimeParam(S, 3, 3, rtParamNames[3], SS_DOUBLE);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_quadRotorSim_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,
      10);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,10,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,10,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,10);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,10,3);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,10,4);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=4; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 3; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,10);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(691457452U));
  ssSetChecksum1(S,(3548100949U));
  ssSetChecksum2(S,(2350116309U));
  ssSetChecksum3(S,(590991671U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c10_quadRotorSim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c10_quadRotorSim(SimStruct *S)
{
  SFc10_quadRotorSimInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc10_quadRotorSimInstanceStruct *)utMalloc(sizeof
    (SFc10_quadRotorSimInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc10_quadRotorSimInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c10_quadRotorSim;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c10_quadRotorSim;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c10_quadRotorSim;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c10_quadRotorSim;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c10_quadRotorSim;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c10_quadRotorSim;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c10_quadRotorSim;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c10_quadRotorSim;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c10_quadRotorSim;
  chartInstance->chartInfo.mdlStart = mdlStart_c10_quadRotorSim;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c10_quadRotorSim;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c10_quadRotorSim_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c10_quadRotorSim(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c10_quadRotorSim(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c10_quadRotorSim(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c10_quadRotorSim_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
