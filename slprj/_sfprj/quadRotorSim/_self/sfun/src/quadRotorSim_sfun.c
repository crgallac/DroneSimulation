/* Include files */

#include "quadRotorSim_sfun.h"
#include "quadRotorSim_sfun_debug_macros.h"
#include "c1_quadRotorSim.h"
#include "c2_quadRotorSim.h"
#include "c3_quadRotorSim.h"
#include "c4_quadRotorSim.h"
#include "c5_quadRotorSim.h"
#include "c6_quadRotorSim.h"
#include "c10_quadRotorSim.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _quadRotorSimMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void quadRotorSim_initializer(void)
{
}

void quadRotorSim_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_quadRotorSim_method_dispatcher(SimStruct *simstructPtr, unsigned
  int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==1) {
    c1_quadRotorSim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==2) {
    c2_quadRotorSim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_quadRotorSim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_quadRotorSim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==5) {
    c5_quadRotorSim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==6) {
    c6_quadRotorSim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==10) {
    c10_quadRotorSim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_quadRotorSim_process_testpoint_info_call( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char machineName[128];
  if (nrhs < 3 || !mxIsChar(prhs[0]) || !mxIsChar(prhs[1]))
    return 0;

  /* Possible call to get testpoint info. */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_testpoint_info"))
    return 0;
  mxGetString(prhs[1], machineName, sizeof(machineName)/sizeof(char));
  machineName[(sizeof(machineName)/sizeof(char)-1)] = '\0';
  if (!strcmp(machineName, "quadRotorSim")) {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
    switch (chartFileNumber) {
     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }

    return 1;
  }

  return 0;

#else

  return 0;

#endif

}

unsigned int sf_quadRotorSim_process_check_sum_call( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3704830823U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3240509431U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(617918188U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(268252632U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(101038022U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4059026793U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3676045932U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2894591869U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_quadRotorSim_get_check_sum(mxArray *plhs[]);
          sf_c1_quadRotorSim_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_c2_quadRotorSim_get_check_sum(mxArray *plhs[]);
          sf_c2_quadRotorSim_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_quadRotorSim_get_check_sum(mxArray *plhs[]);
          sf_c3_quadRotorSim_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_c4_quadRotorSim_get_check_sum(mxArray *plhs[]);
          sf_c4_quadRotorSim_get_check_sum(plhs);
          break;
        }

       case 5:
        {
          extern void sf_c5_quadRotorSim_get_check_sum(mxArray *plhs[]);
          sf_c5_quadRotorSim_get_check_sum(plhs);
          break;
        }

       case 6:
        {
          extern void sf_c6_quadRotorSim_get_check_sum(mxArray *plhs[]);
          sf_c6_quadRotorSim_get_check_sum(plhs);
          break;
        }

       case 10:
        {
          extern void sf_c10_quadRotorSim_get_check_sum(mxArray *plhs[]);
          sf_c10_quadRotorSim_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2200832437U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3057028725U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1374829788U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1795241056U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1189276707U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(422796452U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(39376106U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(4019287099U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_quadRotorSim_autoinheritance_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(aiChksum, "s1icZ2uoFn4pOlafHWfLIB") == 0) {
          extern mxArray *sf_c1_quadRotorSim_get_autoinheritance_info(void);
          plhs[0] = sf_c1_quadRotorSim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 2:
      {
        if (strcmp(aiChksum, "dRl8l3J1LZohZcCbc489LH") == 0) {
          extern mxArray *sf_c2_quadRotorSim_get_autoinheritance_info(void);
          plhs[0] = sf_c2_quadRotorSim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "hg0pWUbZCGhlmqHcZMBTjG") == 0) {
          extern mxArray *sf_c3_quadRotorSim_get_autoinheritance_info(void);
          plhs[0] = sf_c3_quadRotorSim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 4:
      {
        if (strcmp(aiChksum, "1rC1qB28Iw6n2XuezsPZ8") == 0) {
          extern mxArray *sf_c4_quadRotorSim_get_autoinheritance_info(void);
          plhs[0] = sf_c4_quadRotorSim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 5:
      {
        if (strcmp(aiChksum, "K3fAislN2g5ucrJS1kATVH") == 0) {
          extern mxArray *sf_c5_quadRotorSim_get_autoinheritance_info(void);
          plhs[0] = sf_c5_quadRotorSim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 6:
      {
        if (strcmp(aiChksum, "bByezJaKFYAqUWjcMIPihF") == 0) {
          extern mxArray *sf_c6_quadRotorSim_get_autoinheritance_info(void);
          plhs[0] = sf_c6_quadRotorSim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 10:
      {
        if (strcmp(aiChksum, "uio8uNmGPnrVTlcCB1SujE") == 0) {
          extern mxArray *sf_c10_quadRotorSim_get_autoinheritance_info(void);
          plhs[0] = sf_c10_quadRotorSim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_quadRotorSim_get_eml_resolved_functions_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        extern const mxArray *sf_c1_quadRotorSim_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_quadRotorSim_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 2:
      {
        extern const mxArray *sf_c2_quadRotorSim_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_quadRotorSim_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray *sf_c3_quadRotorSim_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_quadRotorSim_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 4:
      {
        extern const mxArray *sf_c4_quadRotorSim_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_quadRotorSim_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 5:
      {
        extern const mxArray *sf_c5_quadRotorSim_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c5_quadRotorSim_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 6:
      {
        extern const mxArray *sf_c6_quadRotorSim_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c6_quadRotorSim_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 10:
      {
        extern const mxArray
          *sf_c10_quadRotorSim_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c10_quadRotorSim_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_quadRotorSim_third_party_uses_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "oCrhcGlHLxm1rgrktM0ZBF") == 0) {
          extern mxArray *sf_c1_quadRotorSim_third_party_uses_info(void);
          plhs[0] = sf_c1_quadRotorSim_third_party_uses_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "BBy605ZD2HyBI7Dus2N1bB") == 0) {
          extern mxArray *sf_c2_quadRotorSim_third_party_uses_info(void);
          plhs[0] = sf_c2_quadRotorSim_third_party_uses_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "JdvcTb5FDnanNEeEEZVYuC") == 0) {
          extern mxArray *sf_c3_quadRotorSim_third_party_uses_info(void);
          plhs[0] = sf_c3_quadRotorSim_third_party_uses_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "gfD6ESHoX2LBbCS5E5kGgG") == 0) {
          extern mxArray *sf_c4_quadRotorSim_third_party_uses_info(void);
          plhs[0] = sf_c4_quadRotorSim_third_party_uses_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "iEgHuzCBqQmkZcpnIJYu6D") == 0) {
          extern mxArray *sf_c5_quadRotorSim_third_party_uses_info(void);
          plhs[0] = sf_c5_quadRotorSim_third_party_uses_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "cNIFu5fff8Ry1gRcZfr8DE") == 0) {
          extern mxArray *sf_c6_quadRotorSim_third_party_uses_info(void);
          plhs[0] = sf_c6_quadRotorSim_third_party_uses_info();
          break;
        }
      }

     case 10:
      {
        if (strcmp(tpChksum, "2nGeczpOMdczJXfd6RaeUE") == 0) {
          extern mxArray *sf_c10_quadRotorSim_third_party_uses_info(void);
          plhs[0] = sf_c10_quadRotorSim_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_quadRotorSim_updateBuildInfo_args_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the updateBuildInfo_args_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "oCrhcGlHLxm1rgrktM0ZBF") == 0) {
          extern mxArray *sf_c1_quadRotorSim_updateBuildInfo_args_info(void);
          plhs[0] = sf_c1_quadRotorSim_updateBuildInfo_args_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "BBy605ZD2HyBI7Dus2N1bB") == 0) {
          extern mxArray *sf_c2_quadRotorSim_updateBuildInfo_args_info(void);
          plhs[0] = sf_c2_quadRotorSim_updateBuildInfo_args_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "JdvcTb5FDnanNEeEEZVYuC") == 0) {
          extern mxArray *sf_c3_quadRotorSim_updateBuildInfo_args_info(void);
          plhs[0] = sf_c3_quadRotorSim_updateBuildInfo_args_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "gfD6ESHoX2LBbCS5E5kGgG") == 0) {
          extern mxArray *sf_c4_quadRotorSim_updateBuildInfo_args_info(void);
          plhs[0] = sf_c4_quadRotorSim_updateBuildInfo_args_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "iEgHuzCBqQmkZcpnIJYu6D") == 0) {
          extern mxArray *sf_c5_quadRotorSim_updateBuildInfo_args_info(void);
          plhs[0] = sf_c5_quadRotorSim_updateBuildInfo_args_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "cNIFu5fff8Ry1gRcZfr8DE") == 0) {
          extern mxArray *sf_c6_quadRotorSim_updateBuildInfo_args_info(void);
          plhs[0] = sf_c6_quadRotorSim_updateBuildInfo_args_info();
          break;
        }
      }

     case 10:
      {
        if (strcmp(tpChksum, "2nGeczpOMdczJXfd6RaeUE") == 0) {
          extern mxArray *sf_c10_quadRotorSim_updateBuildInfo_args_info(void);
          plhs[0] = sf_c10_quadRotorSim_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void quadRotorSim_debug_initialize(struct SfDebugInstanceStruct* debugInstance)
{
  _quadRotorSimMachineNumber_ = sf_debug_initialize_machine(debugInstance,
    "quadRotorSim","sfun",0,7,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _quadRotorSimMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,_quadRotorSimMachineNumber_,
    0);
}

void quadRotorSim_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_quadRotorSim_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info("quadRotorSim",
      "quadRotorSim");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_quadRotorSim_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
