#ifndef __c4_quadRotorSim_h__
#define __c4_quadRotorSim_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc4_quadRotorSimInstanceStruct
#define typedef_SFc4_quadRotorSimInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c4_sfEvent;
  boolean_T c4_isStable;
  boolean_T c4_doneDoubleBufferReInit;
  uint8_T c4_is_active_c4_quadRotorSim;
  real_T c4_Cuav_d[9];
  real_T c4_originOffset[3];
} SFc4_quadRotorSimInstanceStruct;

#endif                                 /*typedef_SFc4_quadRotorSimInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c4_quadRotorSim_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c4_quadRotorSim_get_check_sum(mxArray *plhs[]);
extern void c4_quadRotorSim_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
