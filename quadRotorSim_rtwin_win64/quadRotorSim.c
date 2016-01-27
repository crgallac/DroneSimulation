/*
 * quadRotorSim.c
 *
 * Code generation for model "quadRotorSim".
 *
 * Model version              : 1.64
 * Simulink Coder version : 8.6 (R2014a) 27-Dec-2013
 * C source code generated on : Wed Jan 27 18:14:53 2016
 *
 * Target selection: rtwin.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->32-bit x86 compatible
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */
#include "quadRotorSim.h"
#include "quadRotorSim_private.h"
#include "quadRotorSim_dt.h"

/* Named constants for MATLAB Function: '<S3>/Embedded MATLAB Function' */
#define quadRotorSim_L                 (6.5)
#define quadRotorSim_d                 (1.78)
#define quadRotorSim_d1                (3.0)
#define quadRotorSim_l                 (5.0)

/* options for Real-Time Windows Target board 0 */
static double RTWinBoardOptions0[] = {
  115200.0,
  8.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
};

/* list of Real-Time Windows Target timers */
const int RTWinTimerCount = 2;
const double RTWinTimers[4] = {
  0.002, 0.0,
  0.02, 0.0,
};

/* list of Real-Time Windows Target boards */
const int RTWinBoardCount = 1;
RTWINBOARD RTWinBoards[1] = {
  { "Standard_Devices/Serial_Port", 22U, 8, RTWinBoardOptions0 },
};

/* Block signals (auto storage) */
B_quadRotorSim_T quadRotorSim_B;

/* Block states (auto storage) */
DW_quadRotorSim_T quadRotorSim_DW;

/* Real-time model */
RT_MODEL_quadRotorSim_T quadRotorSim_M_;
RT_MODEL_quadRotorSim_T *const quadRotorSim_M = &quadRotorSim_M_;

/* Forward declaration for local functions */
static void quadRotorSim_fwd_kin(real_T th1, real_T th2, real_T th3, real_T b_l,
  real_T b_L, real_T b_d, real_T b_d1, real_T EE[3], real_T J[9], real_T
  chain_points[15]);
static real_T quadRotorSim_norm(const real_T x[3]);
static real_T quadRotorSim_dot(const real_T a[3], const real_T b[3]);
static void rate_monotonic_scheduler(void);
time_T rt_SimUpdateDiscreteEvents(
  int_T rtmNumSampTimes, void *rtmTimingData, int_T *rtmSampleHitPtr, int_T
  *rtmPerTaskSampleHits )
{
  rtmSampleHitPtr[1] = rtmStepTask(quadRotorSim_M, 1);
  UNUSED_PARAMETER(rtmNumSampTimes);
  UNUSED_PARAMETER(rtmTimingData);
  UNUSED_PARAMETER(rtmPerTaskSampleHits);
  return(-1);
}

/*
 *   This function updates active task flag for each subrate
 * and rate transition flags for tasks that exchange data.
 * The function assumes rate-monotonic multitasking scheduler.
 * The function must be called at model base rate so that
 * the generated code self-manages all its subrates and rate
 * transition flags.
 */
static void rate_monotonic_scheduler(void)
{
  /* To ensure a deterministic data transfer between two rates,
   * data is transferred at the priority of a fast task and the frequency
   * of the slow task.  The following flags indicate when the data transfer
   * happens.  That is, a rate interaction flag is set true when both rates
   * will run, and false otherwise.
   */

  /* tid 0 shares data with slower tid rate: 1 */
  quadRotorSim_M->Timing.RateInteraction.TID0_1 =
    (quadRotorSim_M->Timing.TaskCounters.TID[1] == 0);

  /* update PerTaskSampleHits matrix for non-inline sfcn */
  quadRotorSim_M->Timing.perTaskSampleHits[1] =
    quadRotorSim_M->Timing.RateInteraction.TID0_1;

  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (quadRotorSim_M->Timing.TaskCounters.TID[1])++;
  if ((quadRotorSim_M->Timing.TaskCounters.TID[1]) > 9) {/* Sample time: [0.02s, 0.0s] */
    quadRotorSim_M->Timing.TaskCounters.TID[1] = 0;
  }
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(u0_0, u1_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Function for MATLAB Function: '<S3>/Embedded MATLAB Function' */
static void quadRotorSim_fwd_kin(real_T th1, real_T th2, real_T th3, real_T b_l,
  real_T b_L, real_T b_d, real_T b_d1, real_T EE[3], real_T J[9], real_T
  chain_points[15])
{
  real_T c1;
  real_T c2;
  real_T s1;
  real_T s2;
  real_T xA;
  real_T yA;
  real_T xB;
  real_T yB;
  real_T R;
  real_T M;
  real_T N;
  real_T b;
  real_T y;
  real_T phi2;
  real_T b_R[9];
  int32_T i;
  real_T tmp[6];
  real_T tmp_0[6];
  real_T b_R_0[3];
  real_T b_R_1[3];
  real_T b_R_2[3];
  real_T b_R_3[3];
  real_T ai;
  real_T f_re;

  /* '<S10>:1:50' */
  /*  some intermediate variables */
  /* '<S10>:1:33' */
  c1 = cos(th1);

  /* '<S10>:1:34' */
  c2 = cos(th2);

  /* '<S10>:1:35' */
  s1 = sin(th1);

  /* '<S10>:1:36' */
  s2 = sin(th2);

  /* '<S10>:1:37' */
  xA = b_l * c1;

  /* '<S10>:1:37' */
  yA = b_l * s1;

  /* '<S10>:1:38' */
  xB = b_l * c2 + b_d;

  /* '<S10>:1:38' */
  yB = b_l * s2;

  /* '<S10>:1:39' */
  R = xA * xA + yA * yA;

  /* '<S10>:1:40' */
  /* '<S10>:1:41' */
  M = (yA - yB) / (xB - xA);

  /* '<S10>:1:42' */
  N = ((xB * xB + yB * yB) - R) * 0.5 / (xB - xA);

  /* '<S10>:1:43' */
  phi2 = M * M;

  /* '<S10>:1:44' */
  b = ((M * N - M * xA) - yA) * 2.0;

  /* '<S10>:1:45' */
  /* '<S10>:1:46' */
  R = b * b - (((N * N - 2.0 * N * xA) + R) - b_L * b_L) * ((phi2 + 1.0) * 4.0);

  /* '<S10>:1:47' */
  R *= (real_T)(R > 0.0);

  /* '<S10>:1:48' */
  y = (-b + sqrt(R)) / ((phi2 + 1.0) * 2.0);

  /*  +- */
  /* '<S10>:1:49' */
  R = M * y + N;

  /* '<S10>:1:50' */
  phi2 = (y - b_l * s1) * 0.0;
  ai = y - b_l * s1;
  if (ai == 0.0) {
    f_re = phi2 / b_L;
    phi2 = 0.0;
  } else if (phi2 == 0.0) {
    f_re = 0.0;
    phi2 = ai / b_L;
  } else {
    f_re = phi2 / b_L;
    phi2 = ai / b_L;
  }

  N = rt_atan2d_snf(phi2, (R - b_l * c1) / b_L + f_re);

  /* '<S10>:1:51' */
  phi2 = (y - b_l * s2) * 0.0;
  ai = y - b_l * s2;
  if (ai == 0.0) {
    f_re = phi2 / b_L;
    phi2 = 0.0;
  } else if (phi2 == 0.0) {
    f_re = 0.0;
    phi2 = ai / b_L;
  } else {
    f_re = phi2 / b_L;
    phi2 = ai / b_L;
  }

  phi2 = rt_atan2d_snf(phi2, ((R - b_d) - b_l * c2) / b_L + f_re);

  /* '<S10>:1:53' */
  b = cos(th3);

  /* '<S10>:1:54' */
  M = sin(th3);

  /* '<S10>:1:57' */
  b_R[0] = 1.0;
  b_R[3] = 0.0;
  b_R[6] = 0.0;
  b_R[1] = 0.0;
  b_R[4] = b;
  b_R[7] = -M;
  b_R[2] = 0.0;
  b_R[5] = M;
  b_R[8] = b;

  /*  Points coordinates */
  /* '<S10>:1:61' */
  for (i = 0; i < 3; i++) {
    EE[i] = b_R[i + 6] * b_d1 + (b_R[i + 3] * y + b_R[i] * R);
  }

  /*  Jacobian transposed calculations */
  /* '<S10>:1:64' */
  R = -b_l / sin(phi2 - N);

  /* '<S10>:1:66' */
  /* '<S10>:1:67' */
  /* '<S10>:1:69' */
  tmp[0] = 1.0;
  tmp[3] = 0.0;
  tmp[1] = 0.0;
  tmp[4] = b;
  tmp[2] = 0.0;
  tmp[5] = M;
  f_re = (sin(phi2 - N) * s1 + sin(th1 - phi2) * sin(N)) * R;
  c2 = sin(th2 - phi2) * -sin(N) * R;
  ai = (sin(phi2 - N) * -c1 - sin(th1 - phi2) * cos(N)) * R;
  phi2 = sin(th2 - phi2) * cos(N) * R;
  for (i = 0; i < 3; i++) {
    tmp_0[i] = 0.0;
    tmp_0[i] += tmp[i] * f_re;
    tmp_0[i] += tmp[i + 3] * c2;
    tmp_0[i + 3] = 0.0;
    tmp_0[i + 3] += tmp[i] * ai;
    tmp_0[i + 3] += tmp[i + 3] * phi2;
  }

  for (i = 0; i < 3; i++) {
    J[3 * i] = tmp_0[i];
    J[1 + 3 * i] = tmp_0[i + 3];
  }

  J[2] = 0.0;
  J[5] = -M * y - b * b_d1;
  J[8] = b * y - M * b_d1;

  /*  chain_points=[0;l*exp(1i*th1);E;d+l*exp(1i*th2);d]; */
  /* '<S10>:1:75' */
  /* '<S10>:1:76' */
  /* '<S10>:1:77' */
  /* '<S10>:1:78' */
  /* '<S10>:1:80' */
  for (i = 0; i < 3; i++) {
    b_R_0[i] = b_R[i + 6] * b_d1 + b_R[i + 3] * 0.0;
  }

  for (i = 0; i < 3; i++) {
    b_R_1[i] = b_R[i + 6] * b_d1 + (b_R[i + 3] * yA + b_R[i] * xA);
  }

  for (i = 0; i < 3; i++) {
    b_R_2[i] = b_R[i + 6] * b_d1 + (b_R[i + 3] * yB + b_R[i] * xB);
  }

  for (i = 0; i < 3; i++) {
    b_R_3[i] = b_R[i + 6] * b_d1 + (b_R[i + 3] * 0.0 + b_R[i] * b_d);
  }

  chain_points[0] = b_R_0[0];
  chain_points[5] = b_R_0[1];
  chain_points[10] = b_R_0[2];
  chain_points[1] = b_R_1[0];
  chain_points[6] = b_R_1[1];
  chain_points[11] = b_R_1[2];
  chain_points[2] = EE[0];
  chain_points[7] = EE[1];
  chain_points[12] = EE[2];
  chain_points[3] = b_R_2[0];
  chain_points[8] = b_R_2[1];
  chain_points[13] = b_R_2[2];
  chain_points[4] = b_R_3[0];
  chain_points[9] = b_R_3[1];
  chain_points[14] = b_R_3[2];
}

/* Function for MATLAB Function: '<S7>/Embedded MATLAB Function' */
static real_T quadRotorSim_norm(const real_T x[3])
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  scale = 2.2250738585072014E-308;
  absxk = fabs(x[0]);
  if (absxk > 2.2250738585072014E-308) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 2.2250738585072014E-308;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<S7>/Embedded MATLAB Function' */
static real_T quadRotorSim_dot(const real_T a[3], const real_T b[3])
{
  return (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2];
}

/* Model output function for TID0 */
void quadRotorSim_output0(void)        /* Sample time: [0.002s, 0.0s] */
{
  real_T c;
  real_T s;
  real_T J[9];
  static const int8_T a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  real_T x_diff;
  real_T y_diff;
  real_T z_diff;
  static const int8_T a_0[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  real_T vectF[3];
  real_T vect1[3];
  real_T vect2[3];
  static const real_T e_b[3] = { 0.0, 0.0, 1.0 };

  static const real_T f_b[3] = { 0.0, 1.0, 0.0 };

  static const int8_T a_1[9] = { -1, 0, 0, 0, 0, 1, 0, 1, 0 };

  static const int8_T a_2[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const int8_T a_3[9] = { -1, 0, 0, 0, 0, 1, 0, 1, 0 };

  static const real_T b[3] = { 0.0, 0.2, 1.7 };

  static const int8_T a_4[9] = { -1, 0, 0, 0, 0, 1, 0, 1, 0 };

  real_T rtb_Sum1[3];
  uint32_T rtb_FixPtSum1;
  real_T rtb_dxyzPout[3];
  real_T rtb_xyz_Device[15];
  real_T c_0[9];
  int32_T i;
  real_T c_1[9];
  real_T a_5[3];
  real_T c_2[9];
  int32_T i_0;
  real_T u;

  {                                    /* Sample time: [0.002s, 0.0s] */
    rate_monotonic_scheduler();
  }

  /* S-Function Block: <S8>/Packet Output5 */

  /* no code required */

  /* S-Function Block: <S8>/Packet Input4 */
  {
    uint8_T indata[16U];
    int status = RTBIO_DriverIO(0, STREAMINPUT, IOREAD, 16U,
      &quadRotorSim_P.PacketInput4_PacketID, (double*) indata, NULL);
    if (status & 0x1) {
      RTWin_ANYTYPEPTR indp;
      indp.p_uint8_T = indata;
      quadRotorSim_B.PacketInput4_o1[0] = *indp.p_real32_T++;
      quadRotorSim_B.PacketInput4_o1[1] = *indp.p_real32_T++;
      quadRotorSim_B.PacketInput4_o1[2] = *indp.p_real32_T++;
      quadRotorSim_B.PacketInput4_o2 = *indp.p_uint32_T++;
    }
  }

  /* DataTypeConversion: '<S8>/Conversion' */
  quadRotorSim_B.Conversion[0] = quadRotorSim_B.PacketInput4_o1[0];
  quadRotorSim_B.Conversion[1] = quadRotorSim_B.PacketInput4_o1[1];
  quadRotorSim_B.Conversion[2] = quadRotorSim_B.PacketInput4_o1[2];

  /* UnitDelay: '<S11>/Output' */
  quadRotorSim_B.Output = quadRotorSim_DW.Output_DSTATE;

  /* Sum: '<S8>/Sum1' */
  quadRotorSim_B.Sum1 = quadRotorSim_B.Output - quadRotorSim_B.PacketInput4_o2;

  /* Sum: '<S3>/Sum1' */
  rtb_Sum1[0] = 0.0 + quadRotorSim_B.Conversion[0];
  rtb_Sum1[1] = 0.0 + quadRotorSim_B.Conversion[1];
  rtb_Sum1[2] = 0.0 + quadRotorSim_B.Conversion[2];

  /* SampleTimeMath: '<S9>/TSamp'
   *
   * About '<S9>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  quadRotorSim_B.TSamp[0] = rtb_Sum1[0] * quadRotorSim_P.TSamp_WtEt;
  quadRotorSim_B.TSamp[1] = rtb_Sum1[1] * quadRotorSim_P.TSamp_WtEt;
  quadRotorSim_B.TSamp[2] = rtb_Sum1[2] * quadRotorSim_P.TSamp_WtEt;

  /* MATLAB Function: '<S3>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S3>/Constant1'
   */
  /* MATLAB Function 'Haplet 1/Embedded MATLAB Function': '<S10>:1' */
  /*  K is the stiffness of the plate */
  /*  Ks is the stiffness of the virtual spring in the first demo */
  /*  clip_par=[angle_d;orgn_d] specifies the angle and orgn at which the */
  /*  device is clipped on the screen */
  /* '<S10>:1:8' */
  /* '<S10>:1:9' */
  /* '<S10>:1:10' */
  c = cos(quadRotorSim_P.Constant1_Value[0]);

  /* '<S10>:1:11' */
  s = sin(quadRotorSim_P.Constant1_Value[0]);
  quadRotorSim_fwd_kin(rtb_Sum1[0], rtb_Sum1[1], -rtb_Sum1[2], quadRotorSim_l,
                       quadRotorSim_L, quadRotorSim_d, quadRotorSim_d1, vectF, J,
                       rtb_xyz_Device);

  /* '<S10>:1:13' */
  /* '<S10>:1:14' */
  /*  device to VW coords */
  /* '<S10>:1:16' */
  /* '<S10>:1:17' */
  c_0[0] = c;
  c_0[3] = -s;
  c_0[6] = 0.0;
  c_0[1] = s;
  c_0[4] = c;
  c_0[7] = 0.0;
  c_0[2] = 0.0;
  c_0[5] = 0.0;
  c_0[8] = 1.0;
  for (i = 0; i < 3; i++) {
    rtb_Sum1[i] = c_0[i + 6] * vectF[2] + (c_0[i + 3] * vectF[1] + c_0[i] *
      vectF[0]);
  }

  quadRotorSim_B.xyzP[0] = rtb_Sum1[0] + quadRotorSim_P.Constant1_Value[1];
  quadRotorSim_B.xyzP[1] = rtb_Sum1[1] + quadRotorSim_P.Constant1_Value[2];
  quadRotorSim_B.xyzP[2] = rtb_Sum1[2] + 7.9;

  /* MATLAB Function: '<Root>/Fr_d -> Fr_uav' */
  /*  xyzP(2)=-xyzP(2); */
  /* '<S10>:1:20' */
  /* '<S10>:1:23' */
  /*  J=J';  */
  /* '<S10>:1:26' */
  /* MATLAB Function 'Fr_d -> Fr_uav': '<S1>:1' */
  /*  R=[0 -1 0; 0 0 1; -1 0 0;];  */
  /* '<S1>:1:5' */
  /* '<S1>:1:6' */
  x_diff = quadRotorSim_B.xyzP[0] - quadRotorSim_P.originOffset[0];
  y_diff = quadRotorSim_B.xyzP[1] - quadRotorSim_P.originOffset[1];
  z_diff = quadRotorSim_B.xyzP[2] - quadRotorSim_P.originOffset[2];
  for (i = 0; i < 3; i++) {
    quadRotorSim_B.xyzPout[i] = 0.0;
    quadRotorSim_B.xyzPout[i] += (real_T)a[i] * x_diff;
    quadRotorSim_B.xyzPout[i] += (real_T)a[i + 3] * y_diff;
    quadRotorSim_B.xyzPout[i] += (real_T)a[i + 6] * z_diff;
  }

  /* MATLAB Function: '<S3>/Embedded MATLAB Function' */
  c_1[0] = c;
  c_1[3] = -s;
  c_1[6] = 0.0;
  c_1[1] = s;
  c_1[4] = c;
  c_1[7] = 0.0;
  c_1[2] = 0.0;
  c_1[5] = 0.0;
  c_1[8] = 1.0;
  for (i = 0; i < 3; i++) {
    for (i_0 = 0; i_0 < 3; i_0++) {
      c_0[i + 3 * i_0] = 0.0;
      c_0[i + 3 * i_0] += J[3 * i_0] * c_1[i];
      c_0[i + 3 * i_0] += J[3 * i_0 + 1] * c_1[i + 3];
      c_0[i + 3 * i_0] += J[3 * i_0 + 2] * c_1[i + 6];
    }
  }

  /* Sum: '<S9>/Diff' incorporates:
   *  MATLAB Function: '<S3>/Embedded MATLAB Function'
   *  UnitDelay: '<S9>/UD'
   */
  x_diff = quadRotorSim_B.TSamp[0] - quadRotorSim_DW.UD_DSTATE[0];
  y_diff = quadRotorSim_B.TSamp[1] - quadRotorSim_DW.UD_DSTATE[1];
  z_diff = quadRotorSim_B.TSamp[2] - quadRotorSim_DW.UD_DSTATE[2];

  /* MATLAB Function: '<S3>/Embedded MATLAB Function' incorporates:
   *  MATLAB Function: '<Root>/Fr_d -> Fr_uav'
   */
  for (i = 0; i < 3; i++) {
    rtb_Sum1[i] = c_0[i + 6] * z_diff + (c_0[i + 3] * y_diff + c_0[i] * x_diff);
  }

  /* MATLAB Function: '<Root>/Fr_d -> Fr_uav' */
  for (i = 0; i < 3; i++) {
    rtb_dxyzPout[i] = (real_T)a[i + 6] * rtb_Sum1[2] + ((real_T)a[i + 3] *
      rtb_Sum1[1] + (real_T)a[i] * rtb_Sum1[0]);
  }

  /* SampleTimeMath: '<S14>/TSamp'
   *
   * About '<S14>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  quadRotorSim_B.TSamp_a[0] = quadRotorSim_B.xyzPout[0] *
    quadRotorSim_P.TSamp_WtEt_c;
  quadRotorSim_B.TSamp_a[1] = quadRotorSim_B.xyzPout[1] *
    quadRotorSim_P.TSamp_WtEt_c;
  quadRotorSim_B.TSamp_a[2] = quadRotorSim_B.xyzPout[2] *
    quadRotorSim_P.TSamp_WtEt_c;

  /* MATLAB Function: '<S6>/MATLAB Function1' incorporates:
   *  Constant: '<S6>/Constant'
   *  Constant: '<S6>/b_trans'
   *  Constant: '<S6>/k_trans'
   *  Constant: '<S6>/moveConst'
   *  Constant: '<S6>/moveTimeDelay'
   *  Constant: '<S6>/workspace'
   *  Sum: '<S14>/Diff'
   *  UnitDelay: '<S14>/UD'
   *  UnitDelay: '<S6>/Unit Delay'
   */
  /* MATLAB Function 'Subsystem/MATLAB Function1': '<S15>:1' */
  /*  vel and pos are 3x1 vectors in cartesian coordinates */
  /*  workspace is a 6x1 vector with elements corresponding to x, y, z position */
  /*  of the center of the cylinder, the radius, the length of the cylinder, */
  /*  and the rotational limit */
  /*  device forces are 3x1, tableMove is 3x1 (w/ tableMove(2) = 0 since no y */
  /*  translation) */
  /* '<S15>:1:18' */
  quadRotorSim_B.deviceForces[0] = 0.0;
  quadRotorSim_B.deviceForces[1] = 0.0;
  quadRotorSim_B.deviceForces[2] = 0.0;

  /* '<S15>:1:19' */
  quadRotorSim_B.tableMoveOut[0] = quadRotorSim_DW.UnitDelay_DSTATE[0];
  quadRotorSim_B.tableMoveOut[1] = quadRotorSim_DW.UnitDelay_DSTATE[1];
  quadRotorSim_B.tableMoveOut[2] = quadRotorSim_DW.UnitDelay_DSTATE[2];

  /* '<S15>:1:20' */
  quadRotorSim_DW.clock += quadRotorSim_P.dt;

  /*  pos_rad = sqrt((pos(1) - workspace(1))^2 + (pos(3) - workspace(3))^2); */
  /* '<S15>:1:23' */
  x_diff = fabs(quadRotorSim_B.xyzPout[0] - quadRotorSim_P.workspace_Value[0]);

  /* '<S15>:1:24' */
  y_diff = fabs(quadRotorSim_B.xyzPout[1] - quadRotorSim_P.workspace_Value[1]);

  /* '<S15>:1:25' */
  z_diff = fabs(quadRotorSim_B.xyzPout[2] - quadRotorSim_P.workspace_Value[2]);
  if (x_diff > quadRotorSim_P.workspace_Value[3]) {
    /* '<S15>:1:29' */
    /* '<S15>:1:31' */
    u = quadRotorSim_B.xyzPout[0] - quadRotorSim_P.workspace_Value[0];
    if (u < 0.0) {
      u = -1.0;
    } else if (u > 0.0) {
      u = 1.0;
    } else {
      if (u == 0.0) {
        u = 0.0;
      }
    }

    quadRotorSim_B.deviceForces[0] = -quadRotorSim_P.k_trans_Value * u * fabs
      (x_diff - quadRotorSim_P.workspace_Value[3]) -
      quadRotorSim_P.b_trans_Value * (quadRotorSim_B.TSamp_a[0] -
      quadRotorSim_DW.UD_DSTATE_j[0]);
  }

  if (y_diff > quadRotorSim_P.workspace_Value[3]) {
    /* '<S15>:1:35' */
    /* '<S15>:1:37' */
    u = quadRotorSim_B.xyzPout[1] - quadRotorSim_P.workspace_Value[1];
    if (u < 0.0) {
      u = -1.0;
    } else if (u > 0.0) {
      u = 1.0;
    } else {
      if (u == 0.0) {
        u = 0.0;
      }
    }

    quadRotorSim_B.deviceForces[1] = -quadRotorSim_P.k_trans_Value * u * fabs
      (y_diff - quadRotorSim_P.workspace_Value[3]) -
      quadRotorSim_P.b_trans_Value * (quadRotorSim_B.TSamp_a[1] -
      quadRotorSim_DW.UD_DSTATE_j[1]);
  }

  if (z_diff > quadRotorSim_P.workspace_Value[3]) {
    /* '<S15>:1:41' */
    /* '<S15>:1:43' */
    u = quadRotorSim_B.xyzPout[2] - quadRotorSim_P.workspace_Value[2];
    if (u < 0.0) {
      u = -1.0;
    } else if (u > 0.0) {
      u = 1.0;
    } else {
      if (u == 0.0) {
        u = 0.0;
      }
    }

    quadRotorSim_B.deviceForces[2] = -quadRotorSim_P.k_trans_Value * u * fabs
      (z_diff - quadRotorSim_P.workspace_Value[3]) -
      quadRotorSim_P.b_trans_Value * (quadRotorSim_B.TSamp_a[2] -
      quadRotorSim_DW.UD_DSTATE_j[2]);
  }

  /* '<S15>:1:48' */
  for (i = 0; i < 3; i++) {
    a_5[i] = (real_T)a_0[i + 6] * quadRotorSim_B.deviceForces[2] + ((real_T)
      a_0[i + 3] * quadRotorSim_B.deviceForces[1] + (real_T)a_0[i] *
      quadRotorSim_B.deviceForces[0]);
  }

  quadRotorSim_B.deviceForces[0] = a_5[0];
  quadRotorSim_B.deviceForces[1] = a_5[1];
  quadRotorSim_B.deviceForces[2] = a_5[2];
  if ((x_diff < quadRotorSim_P.workspace_Value[3]) && (y_diff <
       quadRotorSim_P.workspace_Value[3]) && (z_diff <
       quadRotorSim_P.workspace_Value[3])) {
    /* '<S15>:1:51' */
    /* '<S15>:1:53' */
    quadRotorSim_B.time1 = quadRotorSim_DW.clock;
  } else {
    /* '<S15>:1:57' */
    quadRotorSim_B.time1 = quadRotorSim_DW.UnitDelay_DSTATE[3];
  }

  if ((quadRotorSim_DW.clock - quadRotorSim_DW.UnitDelay_DSTATE[3] >
       quadRotorSim_P.moveTimeDelay_Value) && (x_diff >
       quadRotorSim_P.workspace_Value[3])) {
    /* '<S15>:1:61' */
    /* '<S15>:1:63' */
    quadRotorSim_B.time1 = quadRotorSim_DW.UnitDelay_DSTATE[3];

    /* '<S15>:1:64' */
    u = quadRotorSim_B.xyzPout[0] - quadRotorSim_P.workspace_Value[0];
    if (u < 0.0) {
      u = -1.0;
    } else if (u > 0.0) {
      u = 1.0;
    } else {
      if (u == 0.0) {
        u = 0.0;
      }
    }

    quadRotorSim_B.tableMoveOut[0] = cos(quadRotorSim_P.Constant_Value) *
      quadRotorSim_P.moveConst_Value * u * (x_diff -
      quadRotorSim_P.workspace_Value[3]) + quadRotorSim_DW.UnitDelay_DSTATE[0];

    /* '<S15>:1:65' */
    u = quadRotorSim_B.xyzPout[0] - quadRotorSim_P.workspace_Value[0];
    if (u < 0.0) {
      u = -1.0;
    } else if (u > 0.0) {
      u = 1.0;
    } else {
      if (u == 0.0) {
        u = 0.0;
      }
    }

    quadRotorSim_B.tableMoveOut[1] = sin(quadRotorSim_P.Constant_Value) *
      quadRotorSim_P.moveConst_Value * u * (x_diff -
      quadRotorSim_P.workspace_Value[3]) + quadRotorSim_DW.UnitDelay_DSTATE[1];
    if ((quadRotorSim_DW.clock - quadRotorSim_DW.UnitDelay_DSTATE[3] >
         quadRotorSim_P.moveTimeDelay_Value) && (y_diff >
         quadRotorSim_P.workspace_Value[3])) {
      /* '<S15>:1:67' */
      /* '<S15>:1:69' */
      u = quadRotorSim_B.xyzPout[1] - quadRotorSim_P.workspace_Value[1];
      if (u < 0.0) {
        u = -1.0;
      } else if (u > 0.0) {
        u = 1.0;
      } else {
        if (u == 0.0) {
          u = 0.0;
        }
      }

      quadRotorSim_B.tableMoveOut[0] -= sin(quadRotorSim_P.Constant_Value) *
        quadRotorSim_P.moveConst_Value * u * (y_diff -
        quadRotorSim_P.workspace_Value[3]);

      /* '<S15>:1:70' */
      u = quadRotorSim_B.xyzPout[1] - quadRotorSim_P.workspace_Value[1];
      if (u < 0.0) {
        u = -1.0;
      } else if (u > 0.0) {
        u = 1.0;
      } else {
        if (u == 0.0) {
          u = 0.0;
        }
      }

      quadRotorSim_B.tableMoveOut[1] += cos(quadRotorSim_P.Constant_Value) *
        quadRotorSim_P.moveConst_Value * u * (y_diff -
        quadRotorSim_P.workspace_Value[3]);
    }
  }

  if ((quadRotorSim_DW.clock - quadRotorSim_DW.UnitDelay_DSTATE[3] >
       quadRotorSim_P.moveTimeDelay_Value) && (y_diff >
       quadRotorSim_P.workspace_Value[3])) {
    /* '<S15>:1:78' */
    /* '<S15>:1:80' */
    quadRotorSim_B.time1 = quadRotorSim_DW.UnitDelay_DSTATE[3];

    /* '<S15>:1:81' */
    u = quadRotorSim_B.xyzPout[1] - quadRotorSim_P.workspace_Value[1];
    if (u < 0.0) {
      u = -1.0;
    } else if (u > 0.0) {
      u = 1.0;
    } else {
      if (u == 0.0) {
        u = 0.0;
      }
    }

    quadRotorSim_B.tableMoveOut[0] = quadRotorSim_DW.UnitDelay_DSTATE[0] - sin
      (quadRotorSim_P.Constant_Value) * quadRotorSim_P.moveConst_Value * u *
      (y_diff - quadRotorSim_P.workspace_Value[3]);

    /* '<S15>:1:82' */
    u = quadRotorSim_B.xyzPout[1] - quadRotorSim_P.workspace_Value[1];
    if (u < 0.0) {
      u = -1.0;
    } else if (u > 0.0) {
      u = 1.0;
    } else {
      if (u == 0.0) {
        u = 0.0;
      }
    }

    quadRotorSim_B.tableMoveOut[1] = cos(quadRotorSim_P.Constant_Value) *
      quadRotorSim_P.moveConst_Value * u * (y_diff -
      quadRotorSim_P.workspace_Value[3]) + quadRotorSim_DW.UnitDelay_DSTATE[1];
    if ((quadRotorSim_DW.clock - quadRotorSim_DW.UnitDelay_DSTATE[3] >
         quadRotorSim_P.moveTimeDelay_Value) && (x_diff >
         quadRotorSim_P.workspace_Value[3])) {
      /* '<S15>:1:84' */
      /* '<S15>:1:86' */
      u = quadRotorSim_B.xyzPout[0] - quadRotorSim_P.workspace_Value[0];
      if (u < 0.0) {
        u = -1.0;
      } else if (u > 0.0) {
        u = 1.0;
      } else {
        if (u == 0.0) {
          u = 0.0;
        }
      }

      quadRotorSim_B.tableMoveOut[0] += cos(quadRotorSim_P.Constant_Value) *
        quadRotorSim_P.moveConst_Value * u * (x_diff -
        quadRotorSim_P.workspace_Value[3]);

      /* '<S15>:1:87' */
      u = quadRotorSim_B.xyzPout[0] - quadRotorSim_P.workspace_Value[0];
      if (u < 0.0) {
        u = -1.0;
      } else if (u > 0.0) {
        u = 1.0;
      } else {
        if (u == 0.0) {
          u = 0.0;
        }
      }

      quadRotorSim_B.tableMoveOut[1] += sin(quadRotorSim_P.Constant_Value) *
        quadRotorSim_P.moveConst_Value * u * (x_diff -
        quadRotorSim_P.workspace_Value[3]);
    }
  }

  if ((quadRotorSim_DW.clock - quadRotorSim_DW.UnitDelay_DSTATE[3] >
       quadRotorSim_P.moveTimeDelay_Value) && (z_diff >
       quadRotorSim_P.workspace_Value[3])) {
    /* '<S15>:1:93' */
    /* '<S15>:1:95' */
    u = quadRotorSim_B.xyzPout[2] - quadRotorSim_P.workspace_Value[2];
    if (u < 0.0) {
      u = -1.0;
    } else if (u > 0.0) {
      u = 1.0;
    } else {
      if (u == 0.0) {
        u = 0.0;
      }
    }

    quadRotorSim_B.tableMoveOut[2] = quadRotorSim_P.moveConst_Value * u *
      (z_diff - quadRotorSim_P.workspace_Value[3]) +
      quadRotorSim_DW.UnitDelay_DSTATE[2];
  }

  /* End of MATLAB Function: '<S6>/MATLAB Function1' */

  /* MATLAB Function: '<S7>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S7>/Constant'
   *  Constant: '<S7>/Constant1'
   *  Constant: '<S7>/Constant2'
   */
  /* MATLAB Function 'Virtual World/Embedded MATLAB Function': '<S16>:1' */
  /*  xyP,dxyP,xyC,dxyC,r_EE,r_B,K, scr_h, wallThickness,B_water, rho, g */
  /*  F=zeros(10,1);  */
  /* '<S16>:1:5' */
  quadRotorSim_B.F[0] = 0.0;
  quadRotorSim_B.F[1] = 0.0;
  quadRotorSim_B.F[2] = 0.0;

  /*  Xwall=-3;  */
  /*  Ywall=10;  */
  /*  Zwall=13; */
  /*  xyzC=[1; 7; 10.0];  */
  /*  redBallRadius= 3; */
  /*  radiusAvatar=0.5;  */
  /*  K=150;  */
  /*   */
  /*  B=0.035;  */
  /*  if xyzP(2)>Ywall;  */
  /*      F=-K*[0;1;0]*(Ywall-xyzP(2))+ B*dxyzP; */
  /*  end */
  /*   */
  /*           */
  /*  if xyzP(3)>Zwall;  */
  /*      F=-K*[0;0;1]*(Zwall-xyzP(3))+ B*dxyzP; */
  /*  elseif xyzP(3)<7;  */
  /*      F=-K*[0;0;1]*(7-xyzP(3))+ B*dxyzP; */
  /*  end */
  /* '<S16>:1:29' */
  /* '<S16>:1:30' */
  /* '<S16>:1:33' */
  /* '<S16>:1:34' */
  /* '<S16>:1:35' */
  for (i = 0; i < 3; i++) {
    rtb_Sum1[i] = quadRotorSim_B.xyzPout[i] - ((((real_T)a_1[i + 3] *
      quadRotorSim_P.Obstacles[1] + (real_T)a_1[i] * quadRotorSim_P.Obstacles[0])
      + (real_T)a_1[i + 6] * quadRotorSim_P.Obstacles[2]) -
      quadRotorSim_B.tableMoveOut[i]);
  }

  /* '<S16>:1:36' */
  vectF[0] = quadRotorSim_B.xyzPout[0] - (0.0 - quadRotorSim_B.tableMoveOut[0]);
  vectF[1] = quadRotorSim_B.xyzPout[1] - (0.0 - quadRotorSim_B.tableMoveOut[1]);
  vectF[2] = quadRotorSim_B.xyzPout[2] - (0.0 - quadRotorSim_B.tableMoveOut[2]);

  /* '<S16>:1:37' */
  for (i = 0; i < 3; i++) {
    a_5[i] = (((real_T)a_1[i + 3] * quadRotorSim_P.Obstacles[4] + (real_T)a_1[i]
               * quadRotorSim_P.Obstacles[3]) + (real_T)a_1[i + 6] *
              quadRotorSim_P.Obstacles[5]) - quadRotorSim_B.tableMoveOut[i];
  }

  vect1[0] = quadRotorSim_B.xyzPout[0] - a_5[0];
  vect1[1] = quadRotorSim_B.xyzPout[1] - a_5[1];
  vect1[2] = 0.0;

  /* '<S16>:1:38' */
  for (i = 0; i < 3; i++) {
    a_5[i] = (((real_T)a_1[i + 3] * quadRotorSim_P.Obstacles[8] + (real_T)a_1[i]
               * quadRotorSim_P.Obstacles[7]) + (real_T)a_1[i + 6] *
              quadRotorSim_P.Obstacles[9]) - quadRotorSim_B.tableMoveOut[i];
  }

  vect2[0] = quadRotorSim_B.xyzPout[0] - a_5[0];
  vect2[1] = quadRotorSim_B.xyzPout[1] - a_5[1];
  vect2[2] = 0.0;

  /* '<S16>:1:41' */
  y_diff = quadRotorSim_norm(vect1);

  /* '<S16>:1:42' */
  z_diff = quadRotorSim_norm(vect2);
  if (quadRotorSim_dot(rtb_Sum1, f_b) < quadRotorSim_P.radiusAvatar) {
    /* '<S16>:1:45' */
    /* '<S16>:1:46' */
    x_diff = quadRotorSim_P.radiusAvatar - quadRotorSim_dot(rtb_Sum1, f_b);
    quadRotorSim_B.F[0] = (0.0 - quadRotorSim_P.Constant_Value_c * 0.0 * x_diff)
      + quadRotorSim_P.Constant1_Value_d * rtb_dxyzPout[0];
    quadRotorSim_B.F[1] = (0.0 - quadRotorSim_P.Constant_Value_c * x_diff) +
      quadRotorSim_P.Constant1_Value_d * rtb_dxyzPout[1];
    quadRotorSim_B.F[2] = (0.0 - quadRotorSim_P.Constant_Value_c * 0.0 * x_diff)
      + quadRotorSim_P.Constant1_Value_d * rtb_dxyzPout[2];
  }

  if (quadRotorSim_dot(vectF, e_b) < quadRotorSim_P.radiusAvatar) {
    /* '<S16>:1:48' */
    /* '<S16>:1:49' */
    x_diff = quadRotorSim_P.radiusAvatar - quadRotorSim_dot(vectF, e_b);
    quadRotorSim_B.F[0] = (quadRotorSim_B.F[0] - quadRotorSim_P.Constant_Value_c
      * 0.0 * x_diff) + quadRotorSim_P.Constant1_Value_d * rtb_dxyzPout[0];
    quadRotorSim_B.F[1] = (quadRotorSim_B.F[1] - quadRotorSim_P.Constant_Value_c
      * 0.0 * x_diff) + quadRotorSim_P.Constant1_Value_d * rtb_dxyzPout[1];
    quadRotorSim_B.F[2] = (quadRotorSim_B.F[2] - quadRotorSim_P.Constant_Value_c
      * x_diff) + quadRotorSim_P.Constant1_Value_d * rtb_dxyzPout[2];
  }

  if (y_diff < quadRotorSim_P.Obstacles[6] + quadRotorSim_P.radiusAvatar) {
    /* '<S16>:1:51' */
    /* '<S16>:1:52' */
    x_diff = (quadRotorSim_P.radiusAvatar + quadRotorSim_P.Obstacles[6]) -
      y_diff;
    quadRotorSim_B.F[0] = (quadRotorSim_B.F[0] - quadRotorSim_P.Constant_Value_c
      * vect1[0] / y_diff * x_diff) + quadRotorSim_P.Constant1_Value_d *
      rtb_dxyzPout[0];
    quadRotorSim_B.F[1] = (quadRotorSim_B.F[1] - quadRotorSim_P.Constant_Value_c
      * vect1[1] / y_diff * x_diff) + quadRotorSim_P.Constant1_Value_d *
      rtb_dxyzPout[1];
    quadRotorSim_B.F[2] = (quadRotorSim_B.F[2] - quadRotorSim_P.Constant_Value_c
      * 0.0 / y_diff * x_diff) + quadRotorSim_P.Constant1_Value_d *
      rtb_dxyzPout[2];
  }

  if (z_diff < quadRotorSim_P.Obstacles[10] + quadRotorSim_P.radiusAvatar) {
    /* '<S16>:1:54' */
    /* '<S16>:1:55' */
    x_diff = (quadRotorSim_P.radiusAvatar + quadRotorSim_P.Obstacles[10]) -
      z_diff;
    quadRotorSim_B.F[0] = (quadRotorSim_B.F[0] - quadRotorSim_P.Constant_Value_c
      * vect2[0] / z_diff * x_diff) + quadRotorSim_P.Constant1_Value_d *
      rtb_dxyzPout[0];
    quadRotorSim_B.F[1] = (quadRotorSim_B.F[1] - quadRotorSim_P.Constant_Value_c
      * vect2[1] / z_diff * x_diff) + quadRotorSim_P.Constant1_Value_d *
      rtb_dxyzPout[1];
    quadRotorSim_B.F[2] = (quadRotorSim_B.F[2] - quadRotorSim_P.Constant_Value_c
      * 0.0 / z_diff * x_diff) + quadRotorSim_P.Constant1_Value_d *
      rtb_dxyzPout[2];
  }

  /* End of MATLAB Function: '<S7>/Embedded MATLAB Function' */

  /* Sum: '<Root>/Sum1' incorporates:
   *  Constant: '<Root>/Constant3'
   *  Gain: '<Root>/Gain'
   *  MATLAB Function: '<Root>/Fr_uav-Fr_d'
   *  Sum: '<Root>/Sum2'
   */
  /* MATLAB Function 'Fr_uav-Fr_d': '<S2>:1' */
  /*  R=[0 -1 0; 0 0 1; -1 0 0;];  */
  /* '<S2>:1:5' */
  x_diff = quadRotorSim_P.Constant3_Value[0] - (quadRotorSim_B.deviceForces[0] +
    quadRotorSim_B.F[0]) * quadRotorSim_P.Gain_Gain;
  y_diff = quadRotorSim_P.Constant3_Value[1] - (quadRotorSim_B.deviceForces[1] +
    quadRotorSim_B.F[1]) * quadRotorSim_P.Gain_Gain;
  z_diff = quadRotorSim_P.Constant3_Value[2] - (quadRotorSim_B.deviceForces[2] +
    quadRotorSim_B.F[2]) * quadRotorSim_P.Gain_Gain;

  /* MATLAB Function: '<Root>/Fr_uav-Fr_d' */
  for (i = 0; i < 3; i++) {
    quadRotorSim_B.y[i] = 0.0;
    quadRotorSim_B.y[i] += (real_T)a_2[i] * x_diff;
    quadRotorSim_B.y[i] += (real_T)a_2[i + 3] * y_diff;
    quadRotorSim_B.y[i] += (real_T)a_2[i + 6] * z_diff;
  }

  /* MATLAB Function: '<S3>/Embedded MATLAB Function' incorporates:
   *  Product: '<S3>/Product'
   */
  c_2[0] = c;
  c_2[3] = s;
  c_2[6] = 0.0;
  c_2[1] = -s;
  c_2[4] = c;
  c_2[7] = 0.0;
  c_2[2] = 0.0;
  c_2[5] = 0.0;
  c_2[8] = 1.0;
  for (i = 0; i < 3; i++) {
    for (i_0 = 0; i_0 < 3; i_0++) {
      c_0[i + 3 * i_0] = 0.0;
      c_0[i + 3 * i_0] += c_2[3 * i_0] * J[i];
      c_0[i + 3 * i_0] += c_2[3 * i_0 + 1] * J[i + 3];
      c_0[i + 3 * i_0] += c_2[3 * i_0 + 2] * J[i + 6];
    }
  }

  /* ManualSwitch: '<S3>/Manual Switch1' incorporates:
   *  Constant: '<S3>/Constant4'
   *  Product: '<S3>/Product'
   */
  if (quadRotorSim_P.ManualSwitch1_CurrentSetting == 1) {
    x_diff = quadRotorSim_B.y[0];
    y_diff = quadRotorSim_B.y[1];
    z_diff = quadRotorSim_B.y[2];
  } else {
    x_diff = quadRotorSim_P.Constant4_Value[0];
    y_diff = quadRotorSim_P.Constant4_Value[1];
    z_diff = quadRotorSim_P.Constant4_Value[2];
  }

  /* End of ManualSwitch: '<S3>/Manual Switch1' */

  /* ManualSwitch: '<S3>/Manual Switch' incorporates:
   *  Constant: '<S3>/Constant3'
   *  Product: '<S3>/Product'
   */
  if (quadRotorSim_P.ManualSwitch_CurrentSetting == 1) {
    for (i = 0; i < 3; i++) {
      quadRotorSim_B.ManualSwitch[i] = (c_0[i + 3] * y_diff + c_0[i] * x_diff) +
        c_0[i + 6] * z_diff;
    }
  } else {
    for (i = 0; i < 3; i++) {
      quadRotorSim_B.ManualSwitch[i] = quadRotorSim_P.Constant3_Value_d[i];
    }
  }

  /* End of ManualSwitch: '<S3>/Manual Switch' */

  /* DataTypeConversion: '<S8>/Conversion1' */
  quadRotorSim_B.Conversion1[0] = (real32_T)quadRotorSim_B.ManualSwitch[0];
  quadRotorSim_B.Conversion1[1] = (real32_T)quadRotorSim_B.ManualSwitch[1];
  quadRotorSim_B.Conversion1[2] = (real32_T)quadRotorSim_B.ManualSwitch[2];

  /* Sum: '<S12>/FixPt Sum1' incorporates:
   *  Constant: '<S12>/FixPt Constant'
   */
  rtb_FixPtSum1 = quadRotorSim_B.Output + quadRotorSim_P.FixPtConstant_Value;

  /* Switch: '<S13>/FixPt Switch' incorporates:
   *  Constant: '<S13>/Constant'
   */
  if (rtb_FixPtSum1 > quadRotorSim_P.WrapToZero_Threshold) {
    quadRotorSim_B.FixPtSwitch = quadRotorSim_P.Constant_Value_g;
  } else {
    quadRotorSim_B.FixPtSwitch = rtb_FixPtSum1;
  }

  /* End of Switch: '<S13>/FixPt Switch' */

  /* ManualSwitch: '<Root>/Manual Switch' incorporates:
   *  Constant: '<Root>/Constant8'
   */
  if (quadRotorSim_P.ManualSwitch_CurrentSetting_d == 1) {
    vectF[0] = quadRotorSim_B.tableMoveOut[0];
    vectF[1] = quadRotorSim_B.tableMoveOut[1];
    vectF[2] = quadRotorSim_B.tableMoveOut[2];
  } else {
    vectF[0] = quadRotorSim_P.Constant8_Value[0];
    vectF[1] = quadRotorSim_P.Constant8_Value[1];
    vectF[2] = quadRotorSim_P.Constant8_Value[2];
  }

  /* End of ManualSwitch: '<Root>/Manual Switch' */

  /* Sum: '<Root>/Sum3' */
  quadRotorSim_B.Sum3[0] = vectF[0] + quadRotorSim_B.xyzPout[0];
  quadRotorSim_B.Sum3[1] = vectF[1] + quadRotorSim_B.xyzPout[1];
  quadRotorSim_B.Sum3[2] = vectF[2] + quadRotorSim_B.xyzPout[2];

  /* MATLAB Function: '<Root>/MATLAB Function' */
  /* MATLAB Function 'MATLAB Function': '<S4>:1' */
  /*  R=[0 -1 0; 0 0 1; -1 0 0;];  */
  /* '<S4>:1:5' */
  for (i = 0; i < 3; i++) {
    rtb_Sum1[i] = (real_T)a_3[i + 6] * quadRotorSim_B.Sum3[2] + ((real_T)a_3[i +
      3] * quadRotorSim_B.Sum3[1] + (real_T)a_3[i] * quadRotorSim_B.Sum3[0]);
  }

  /* End of MATLAB Function: '<Root>/MATLAB Function' */

  /* RateTransition: '<Root>/Rate Transition' */
  if (quadRotorSim_M->Timing.RateInteraction.TID0_1) {
    quadRotorSim_B.RateTransition[0] = rtb_Sum1[0];
    quadRotorSim_B.RateTransition[1] = rtb_Sum1[1];
    quadRotorSim_B.RateTransition[2] = rtb_Sum1[2];
  }

  /* End of RateTransition: '<Root>/Rate Transition' */

  /* MATLAB Function: '<Root>/MATLAB Function1' */
  /* MATLAB Function 'MATLAB Function1': '<S5>:1' */
  /*  R=[0 -1 0; 0 0 1; -1 0 0;];  */
  /* '<S5>:1:5' */
  for (i = 0; i < 3; i++) {
    rtb_Sum1[i] = (((real_T)a_4[i + 3] * vectF[1] + (real_T)a_4[i] * vectF[0]) +
                   (real_T)a_4[i + 6] * vectF[2]) + b[i];
  }

  /* End of MATLAB Function: '<Root>/MATLAB Function1' */

  /* RateTransition: '<Root>/Rate Transition1' */
  if (quadRotorSim_M->Timing.RateInteraction.TID0_1) {
    quadRotorSim_B.RateTransition1[0] = rtb_Sum1[0];
    quadRotorSim_B.RateTransition1[1] = rtb_Sum1[1];
    quadRotorSim_B.RateTransition1[2] = rtb_Sum1[2];
  }

  /* End of RateTransition: '<Root>/Rate Transition1' */
}

/* Model update function for TID0 */
void quadRotorSim_update0(void)        /* Sample time: [0.002s, 0.0s] */
{
  /* S-Function Block: <S8>/Packet Output5 */
  {
    uint8_T outdata[16U];
    RTWin_ANYTYPEPTR outdp;
    outdp.p_uint8_T = outdata;

    {
      uint32_T pktout = quadRotorSim_B.Output;
      *outdp.p_uint32_T++ = pktout;
    }

    {
      real32_T pktout = quadRotorSim_B.Conversion1[0];
      *outdp.p_real32_T++ = pktout;
    }

    {
      real32_T pktout = quadRotorSim_B.Conversion1[1];
      *outdp.p_real32_T++ = pktout;
    }

    {
      real32_T pktout = quadRotorSim_B.Conversion1[2];
      *outdp.p_real32_T++ = pktout;
    }

    RTBIO_DriverIO(0, STREAMOUTPUT, IOWRITE, 16U,
                   &quadRotorSim_P.PacketOutput5_PacketID, (double*) outdata,
                   NULL);
  }

  /* Update for UnitDelay: '<S11>/Output' */
  quadRotorSim_DW.Output_DSTATE = quadRotorSim_B.FixPtSwitch;

  /* Update for UnitDelay: '<S9>/UD' */
  quadRotorSim_DW.UD_DSTATE[0] = quadRotorSim_B.TSamp[0];
  quadRotorSim_DW.UD_DSTATE[1] = quadRotorSim_B.TSamp[1];
  quadRotorSim_DW.UD_DSTATE[2] = quadRotorSim_B.TSamp[2];

  /* Update for UnitDelay: '<S14>/UD' */
  quadRotorSim_DW.UD_DSTATE_j[0] = quadRotorSim_B.TSamp_a[0];
  quadRotorSim_DW.UD_DSTATE_j[1] = quadRotorSim_B.TSamp_a[1];
  quadRotorSim_DW.UD_DSTATE_j[2] = quadRotorSim_B.TSamp_a[2];

  /* Update for UnitDelay: '<S6>/Unit Delay' */
  quadRotorSim_DW.UnitDelay_DSTATE[0] = quadRotorSim_B.tableMoveOut[0];
  quadRotorSim_DW.UnitDelay_DSTATE[1] = quadRotorSim_B.tableMoveOut[1];
  quadRotorSim_DW.UnitDelay_DSTATE[2] = quadRotorSim_B.tableMoveOut[2];
  quadRotorSim_DW.UnitDelay_DSTATE[3] = quadRotorSim_B.time1;

  /* Update absolute time */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++quadRotorSim_M->Timing.clockTick0)) {
    ++quadRotorSim_M->Timing.clockTickH0;
  }

  quadRotorSim_M->Timing.t[0] = quadRotorSim_M->Timing.clockTick0 *
    quadRotorSim_M->Timing.stepSize0 + quadRotorSim_M->Timing.clockTickH0 *
    quadRotorSim_M->Timing.stepSize0 * 4294967296.0;
}

/* Model output function for TID1 */
void quadRotorSim_output1(void)        /* Sample time: [0.02s, 0.0s] */
{
  /* Constant: '<Root>/Constant9' */
  memcpy(&quadRotorSim_B.Constant9[0], &quadRotorSim_P.Obstacles[0], 11U *
         sizeof(real_T));

  /* Constant: '<Root>/Constant4' */
  quadRotorSim_B.Constant4 = quadRotorSim_P.radiusAvatar;

  /* Constant: '<Root>/Constant1' */
  quadRotorSim_B.Constant1 = quadRotorSim_P.Constant1_Value_o;

  /* Constant: '<Root>/Constant5' */
  quadRotorSim_B.Constant5 = quadRotorSim_P.Constant5_Value;

  /* Gain: '<Root>/Gain1' */
  quadRotorSim_B.Gain1 = quadRotorSim_P.Gain1_Gain *
    quadRotorSim_B.RateTransition[1];

  /* Constant: '<Root>/Constant6' */
  quadRotorSim_B.Constant6 = quadRotorSim_P.Constant6_Value;

  /* SignalConversion: '<Root>/TmpSignal ConversionAtVR SinkInport5' */
  quadRotorSim_B.TmpSignalConversionAtVRSinkInport5[0] =
    quadRotorSim_B.Constant5;
  quadRotorSim_B.TmpSignalConversionAtVRSinkInport5[1] = quadRotorSim_B.Gain1;
  quadRotorSim_B.TmpSignalConversionAtVRSinkInport5[2] =
    quadRotorSim_B.Constant6;

  /* Constant: '<Root>/Constant2' */
  quadRotorSim_B.Constant2 = quadRotorSim_P.Constant2_Value;

  /* Constant: '<Root>/Constant7' */
  quadRotorSim_B.Constant7 = quadRotorSim_P.navSphere;
}

/* Model update function for TID1 */
void quadRotorSim_update1(void)        /* Sample time: [0.02s, 0.0s] */
{
  /* Update absolute time */
  /* The "clockTick1" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick1"
   * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick1 and the high bits
   * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++quadRotorSim_M->Timing.clockTick1)) {
    ++quadRotorSim_M->Timing.clockTickH1;
  }

  quadRotorSim_M->Timing.t[1] = quadRotorSim_M->Timing.clockTick1 *
    quadRotorSim_M->Timing.stepSize1 + quadRotorSim_M->Timing.clockTickH1 *
    quadRotorSim_M->Timing.stepSize1 * 4294967296.0;
}

/* Model output wrapper function for compatibility with a static main program */
void quadRotorSim_output(int_T tid)
{
  switch (tid) {
   case 0 :
    quadRotorSim_output0();
    break;

   case 1 :
    quadRotorSim_output1();
    break;

   default :
    break;
  }
}

/* Model update wrapper function for compatibility with a static main program */
void quadRotorSim_update(int_T tid)
{
  switch (tid) {
   case 0 :
    quadRotorSim_update0();
    break;

   case 1 :
    quadRotorSim_update1();
    break;

   default :
    break;
  }
}

/* Model initialize function */
void quadRotorSim_initialize(void)
{
  /* S-Function Block: <S8>/Packet Output5 */
  /* no initial value should be set */

  /* Start for Constant: '<Root>/Constant9' */
  memcpy(&quadRotorSim_B.Constant9[0], &quadRotorSim_P.Obstacles[0], 11U *
         sizeof(real_T));

  /* Start for Constant: '<Root>/Constant4' */
  quadRotorSim_B.Constant4 = quadRotorSim_P.radiusAvatar;

  /* Start for Constant: '<Root>/Constant1' */
  quadRotorSim_B.Constant1 = quadRotorSim_P.Constant1_Value_o;

  /* Start for Constant: '<Root>/Constant2' */
  quadRotorSim_B.Constant2 = quadRotorSim_P.Constant2_Value;

  /* Start for Constant: '<Root>/Constant7' */
  quadRotorSim_B.Constant7 = quadRotorSim_P.navSphere;

  /* InitializeConditions for UnitDelay: '<S11>/Output' */
  quadRotorSim_DW.Output_DSTATE = quadRotorSim_P.Output_InitialCondition;

  /* InitializeConditions for UnitDelay: '<S9>/UD' */
  quadRotorSim_DW.UD_DSTATE[0] = quadRotorSim_P.DiscrDer_ICPrevScaledInput[0];
  quadRotorSim_DW.UD_DSTATE[1] = quadRotorSim_P.DiscrDer_ICPrevScaledInput[1];
  quadRotorSim_DW.UD_DSTATE[2] = quadRotorSim_P.DiscrDer_ICPrevScaledInput[2];

  /* InitializeConditions for UnitDelay: '<S14>/UD' */
  quadRotorSim_DW.UD_DSTATE_j[0] =
    quadRotorSim_P.DiscreteDerivative_ICPrevScaledInput;
  quadRotorSim_DW.UD_DSTATE_j[1] =
    quadRotorSim_P.DiscreteDerivative_ICPrevScaledInput;
  quadRotorSim_DW.UD_DSTATE_j[2] =
    quadRotorSim_P.DiscreteDerivative_ICPrevScaledInput;

  /* InitializeConditions for UnitDelay: '<S6>/Unit Delay' */
  quadRotorSim_DW.UnitDelay_DSTATE[0] =
    quadRotorSim_P.UnitDelay_InitialCondition[0];
  quadRotorSim_DW.UnitDelay_DSTATE[1] =
    quadRotorSim_P.UnitDelay_InitialCondition[1];
  quadRotorSim_DW.UnitDelay_DSTATE[2] =
    quadRotorSim_P.UnitDelay_InitialCondition[2];
  quadRotorSim_DW.UnitDelay_DSTATE[3] =
    quadRotorSim_P.UnitDelay_InitialCondition[3];

  /* InitializeConditions for MATLAB Function: '<S6>/MATLAB Function1' */
  quadRotorSim_DW.clock = 0.0;
}

/* Model terminate function */
void quadRotorSim_terminate(void)
{
  /* S-Function Block: <S8>/Packet Output5 */
  /* no initial value should be set */
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/
void MdlOutputs(int_T tid)
{
  quadRotorSim_output(tid);
}

void MdlUpdate(int_T tid)
{
  quadRotorSim_update(tid);
}

void MdlInitializeSizes(void)
{
}

void MdlInitializeSampleTimes(void)
{
}

void MdlInitialize(void)
{
}

void MdlStart(void)
{
  quadRotorSim_initialize();
}

void MdlTerminate(void)
{
  quadRotorSim_terminate();
}

/* Registration function */
RT_MODEL_quadRotorSim_T *quadRotorSim(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* non-finite (run-time) assignments */
  quadRotorSim_P.PacketOutput5_MaxMissedTicks = rtInf;
  quadRotorSim_P.PacketInput4_MaxMissedTicks = rtInf;

  /* initialize real-time model */
  (void) memset((void *)quadRotorSim_M, 0,
                sizeof(RT_MODEL_quadRotorSim_T));

  /* Initialize timing info */
  {
    int_T *mdlTsMap = quadRotorSim_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    quadRotorSim_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    quadRotorSim_M->Timing.sampleTimes =
      (&quadRotorSim_M->Timing.sampleTimesArray[0]);
    quadRotorSim_M->Timing.offsetTimes =
      (&quadRotorSim_M->Timing.offsetTimesArray[0]);

    /* task periods */
    quadRotorSim_M->Timing.sampleTimes[0] = (0.002);
    quadRotorSim_M->Timing.sampleTimes[1] = (0.02);

    /* task offsets */
    quadRotorSim_M->Timing.offsetTimes[0] = (0.0);
    quadRotorSim_M->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr(quadRotorSim_M, &quadRotorSim_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = quadRotorSim_M->Timing.sampleHitArray;
    int_T *mdlPerTaskSampleHits = quadRotorSim_M->Timing.perTaskSampleHitsArray;
    quadRotorSim_M->Timing.perTaskSampleHits = (&mdlPerTaskSampleHits[0]);
    mdlSampleHits[0] = 1;
    quadRotorSim_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(quadRotorSim_M, -1);
  quadRotorSim_M->Timing.stepSize0 = 0.002;
  quadRotorSim_M->Timing.stepSize1 = 0.02;

  /* External mode info */
  quadRotorSim_M->Sizes.checksums[0] = (3811013152U);
  quadRotorSim_M->Sizes.checksums[1] = (1689044043U);
  quadRotorSim_M->Sizes.checksums[2] = (1072623783U);
  quadRotorSim_M->Sizes.checksums[3] = (2143496835U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[9];
    quadRotorSim_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    systemRan[3] = &rtAlwaysEnabled;
    systemRan[4] = &rtAlwaysEnabled;
    systemRan[5] = &rtAlwaysEnabled;
    systemRan[6] = &rtAlwaysEnabled;
    systemRan[7] = &rtAlwaysEnabled;
    systemRan[8] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(quadRotorSim_M->extModeInfo,
      &quadRotorSim_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(quadRotorSim_M->extModeInfo,
                        quadRotorSim_M->Sizes.checksums);
    rteiSetTPtr(quadRotorSim_M->extModeInfo, rtmGetTPtr(quadRotorSim_M));
  }

  quadRotorSim_M->solverInfoPtr = (&quadRotorSim_M->solverInfo);
  quadRotorSim_M->Timing.stepSize = (0.002);
  rtsiSetFixedStepSize(&quadRotorSim_M->solverInfo, 0.002);
  rtsiSetSolverMode(&quadRotorSim_M->solverInfo, SOLVER_MODE_MULTITASKING);

  /* block I/O */
  quadRotorSim_M->ModelData.blockIO = ((void *) &quadRotorSim_B);
  (void) memset(((void *) &quadRotorSim_B), 0,
                sizeof(B_quadRotorSim_T));

  /* parameters */
  quadRotorSim_M->ModelData.defaultParam = ((real_T *)&quadRotorSim_P);

  /* states (dwork) */
  quadRotorSim_M->ModelData.dwork = ((void *) &quadRotorSim_DW);
  (void) memset((void *)&quadRotorSim_DW, 0,
                sizeof(DW_quadRotorSim_T));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    quadRotorSim_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 14;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.B = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.P = &rtPTransTable;
  }

  /* Initialize Sizes */
  quadRotorSim_M->Sizes.numContStates = (0);/* Number of continuous states */
  quadRotorSim_M->Sizes.numY = (0);    /* Number of model outputs */
  quadRotorSim_M->Sizes.numU = (0);    /* Number of model inputs */
  quadRotorSim_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  quadRotorSim_M->Sizes.numSampTimes = (2);/* Number of sample times */
  quadRotorSim_M->Sizes.numBlocks = (76);/* Number of blocks */
  quadRotorSim_M->Sizes.numBlockIO = (29);/* Number of block outputs */
  quadRotorSim_M->Sizes.numBlockPrms = (73);/* Sum of parameter "widths" */
  return quadRotorSim_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/
