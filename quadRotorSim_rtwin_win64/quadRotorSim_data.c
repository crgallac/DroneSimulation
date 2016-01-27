/*
 * quadRotorSim_data.c
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

/* Block parameters (auto storage) */
P_quadRotorSim_T quadRotorSim_P = {
  /*  Variable: Obstacles
   * Referenced by:
   *   '<Root>/Constant9'
   *   '<S7>/Constant2'
   */
  { 0.0, 5.0, -10.0, 0.0, 500.0, -11.0, 2.0, -6.0, 500.0, -11.0, 4.0 },
  0.002,                               /* Variable: dt
                                        * Referenced by: '<S6>/MATLAB Function1'
                                        */
  1.0,                                 /* Variable: navSphere
                                        * Referenced by: '<Root>/Constant7'
                                        */

  /*  Variable: originOffset
   * Referenced by: '<Root>/Fr_d -> Fr_uav'
   */
  { -0.5, 6.0, 9.6 },
  0.2,                                 /* Variable: radiusAvatar
                                        * Referenced by:
                                        *   '<Root>/Constant4'
                                        *   '<S7>/Embedded MATLAB Function'
                                        */

  /*  Mask Parameter: DiscrDer_ICPrevScaledInput
   * Referenced by: '<S9>/UD'
   */
  { 0.0, 0.0, 0.0 },
  0.0,                                 /* Mask Parameter: DiscreteDerivative_ICPrevScaledInput
                                        * Referenced by: '<S14>/UD'
                                        */
  0.0,                                 /* Mask Parameter: PacketOutput5_MaxMissedTicks
                                        * Referenced by: '<S8>/Packet Output5'
                                        */
  0.0,                                 /* Mask Parameter: PacketInput4_MaxMissedTicks
                                        * Referenced by: '<S8>/Packet Input4'
                                        */
  1.0,                                 /* Mask Parameter: PacketOutput5_YieldWhenWaiting
                                        * Referenced by: '<S8>/Packet Output5'
                                        */
  1.0,                                 /* Mask Parameter: PacketInput4_YieldWhenWaiting
                                        * Referenced by: '<S8>/Packet Input4'
                                        */
  1,                                   /* Mask Parameter: PacketOutput5_PacketID
                                        * Referenced by: '<S8>/Packet Output5'
                                        */
  1,                                   /* Mask Parameter: PacketInput4_PacketID
                                        * Referenced by: '<S8>/Packet Input4'
                                        */
  4294967295U,                         /* Mask Parameter: WrapToZero_Threshold
                                        * Referenced by: '<S13>/FixPt Switch'
                                        */
  500.0,                               /* Computed Parameter: TSamp_WtEt
                                        * Referenced by: '<S9>/TSamp'
                                        */

  /*  Expression: [angle_d1;orgn_d1]
   * Referenced by: '<S3>/Constant1'
   */
  { 0.0, 0.0, 0.0, 8.6 },

  /*  Expression: [0;0;0;]
   * Referenced by: '<Root>/Constant3'
   */
  { 0.0, 0.0, 0.0 },
  500.0,                               /* Computed Parameter: TSamp_WtEt_c
                                        * Referenced by: '<S14>/TSamp'
                                        */
  20.0,                                /* Expression: 20
                                        * Referenced by: '<S6>/k_trans'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S6>/b_trans'
                                        */

  /*  Expression: [0 0 0 navSphere]'
   * Referenced by: '<S6>/workspace'
   */
  { 0.0, 0.0, 0.0, 1.0 },
  0.5,                                 /* Expression: .5
                                        * Referenced by: '<S6>/moveTimeDelay'
                                        */
  0.005,                               /* Expression: .005
                                        * Referenced by: '<S6>/moveConst'
                                        */

  /*  Expression: [0 0 0 0]
   * Referenced by: '<S6>/Unit Delay'
   */
  { 0.0, 0.0, 0.0, 0.0 },
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S6>/Constant'
                                        */
  400.0,                               /* Expression: 400
                                        * Referenced by: '<S7>/Constant'
                                        */
  0.1,                                 /* Expression: .1
                                        * Referenced by: '<S7>/Constant1'
                                        */
  0.0002,                              /* Expression: 2e-4
                                        * Referenced by: '<Root>/Gain'
                                        */

  /*  Expression: [0.00;.00;0.01;]
   * Referenced by: '<S3>/Constant4'
   */
  { 0.0, 0.0, 0.01 },

  /*  Expression: [0;0;0;]
   * Referenced by: '<S3>/Constant3'
   */
  { 0.0, 0.0, 0.0 },

  /*  Expression: [0;0;0;]
   * Referenced by: '<Root>/Constant8'
   */
  { 0.0, 0.0, 0.0 },
  1.0,                                 /* Expression: 1
                                        * Referenced by: '<Root>/Constant1'
                                        */
  -1.7,                                /* Expression: -1.7
                                        * Referenced by: '<Root>/Constant5'
                                        */
  -1.0,                                /* Expression: -1
                                        * Referenced by: '<Root>/Gain1'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Constant6'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Constant2'
                                        */
  0U,                                  /* Computed Parameter: Constant_Value_g
                                        * Referenced by: '<S13>/Constant'
                                        */
  0U,                                  /* Computed Parameter: Output_InitialCondition
                                        * Referenced by: '<S11>/Output'
                                        */
  1U,                                  /* Computed Parameter: FixPtConstant_Value
                                        * Referenced by: '<S12>/FixPt Constant'
                                        */
  1U,                                  /* Computed Parameter: ManualSwitch1_CurrentSetting
                                        * Referenced by: '<S3>/Manual Switch1'
                                        */
  1U,                                  /* Computed Parameter: ManualSwitch_CurrentSetting
                                        * Referenced by: '<S3>/Manual Switch'
                                        */
  1U                                   /* Computed Parameter: ManualSwitch_CurrentSetting_d
                                        * Referenced by: '<Root>/Manual Switch'
                                        */
};
