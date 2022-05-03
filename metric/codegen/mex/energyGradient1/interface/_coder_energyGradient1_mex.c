/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_energyGradient1_mex.c
 *
 * Code generation for function '_coder_energyGradient1_mex'
 *
 */

/* Include files */
#include "_coder_energyGradient1_mex.h"
#include "_coder_energyGradient1_api.h"
#include "energyGradient1_data.h"
#include "energyGradient1_initialize.h"
#include "energyGradient1_terminate.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void energyGradient1_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
                                 const mxArray *prhs[7])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 7) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 7, 4,
                        15, "energyGradient1");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 15,
                        "energyGradient1");
  }
  /* Call the function. */
  energyGradient1_api(prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&energyGradient1_atexit);
  /* Module initialization. */
  energyGradient1_initialize();
  /* Dispatch the entry-point. */
  energyGradient1_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  energyGradient1_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2021a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_energyGradient1_mex.c) */
