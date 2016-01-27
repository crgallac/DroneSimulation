/*
 * quadRotorSim_dt.h
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

#include "ext_types.h"

/* data type size table */
static uint_T rtDataTypeSizes[] = {
  sizeof(real_T),
  sizeof(real32_T),
  sizeof(int8_T),
  sizeof(uint8_T),
  sizeof(int16_T),
  sizeof(uint16_T),
  sizeof(int32_T),
  sizeof(uint32_T),
  sizeof(boolean_T),
  sizeof(fcn_call_T),
  sizeof(int_T),
  sizeof(pointer_T),
  sizeof(action_T),
  2*sizeof(uint32_T)
};

/* data type name table */
static const char_T * rtDataTypeNames[] = {
  "real_T",
  "real32_T",
  "int8_T",
  "uint8_T",
  "int16_T",
  "uint16_T",
  "int32_T",
  "uint32_T",
  "boolean_T",
  "fcn_call_T",
  "int_T",
  "pointer_T",
  "action_T",
  "timer_uint32_pair_T"
};

/* data type transitions for block I/O structure */
static DataTypeTransition rtBTransitions[] = {
  { (char_T *)(&quadRotorSim_B.Conversion[0]), 0, 0, 61 },

  { (char_T *)(&quadRotorSim_B.PacketInput4_o1[0]), 1, 0, 6 },

  { (char_T *)(&quadRotorSim_B.PacketInput4_o2), 7, 0, 4 }
  ,

  { (char_T *)(&quadRotorSim_DW.UD_DSTATE[0]), 0, 0, 11 },

  { (char_T *)(&quadRotorSim_DW.PacketOutput5_PWORK[0]), 11, 0, 31 },

  { (char_T *)(&quadRotorSim_DW.Output_DSTATE), 7, 0, 1 }
};

/* data type transition table for block I/O structure */
static DataTypeTransitionTable rtBTransTable = {
  6U,
  rtBTransitions
};

/* data type transitions for Parameters structure */
static DataTypeTransition rtPTransitions[] = {
  { (char_T *)(&quadRotorSim_P.Obstacles[0]), 0, 0, 25 },

  { (char_T *)(&quadRotorSim_P.PacketOutput5_PacketID), 6, 0, 2 },

  { (char_T *)(&quadRotorSim_P.WrapToZero_Threshold), 7, 0, 1 },

  { (char_T *)(&quadRotorSim_P.TSamp_WtEt), 0, 0, 39 },

  { (char_T *)(&quadRotorSim_P.Constant_Value_g), 7, 0, 3 },

  { (char_T *)(&quadRotorSim_P.ManualSwitch1_CurrentSetting), 3, 0, 3 }
};

/* data type transition table for Parameters structure */
static DataTypeTransitionTable rtPTransTable = {
  6U,
  rtPTransitions
};

/* [EOF] quadRotorSim_dt.h */
