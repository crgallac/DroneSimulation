/*
 * quadRotorSim_private.h
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
#ifndef RTW_HEADER_quadRotorSim_private_h_
#define RTW_HEADER_quadRotorSim_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#ifndef __RTWTYPES_H__
#error This file requires rtwtypes.h to be included
#endif                                 /* __RTWTYPES_H__ */

extern real_T rt_atan2d_snf(real_T u0, real_T u1);
void quadRotorSim_output0(void);
void quadRotorSim_update0(void);
void quadRotorSim_output1(void);
void quadRotorSim_update1(void);

#endif                                 /* RTW_HEADER_quadRotorSim_private_h_ */
