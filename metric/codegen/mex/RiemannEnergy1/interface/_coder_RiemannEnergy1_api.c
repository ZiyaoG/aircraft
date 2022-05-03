/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_RiemannEnergy1_api.c
 *
 * Code generation for function '_coder_RiemannEnergy1_api'
 *
 */

/* Include files */
#include "_coder_RiemannEnergy1_api.h"
#include "RiemannEnergy1.h"
#include "RiemannEnergy1_data.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[12];

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *n,
                                 const char_T *identifier);

static real_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *T,
                                   const char_T *identifier))[28];

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *c,
                                 const char_T *identifier))[12];

static const mxArray *emlrt_marshallOut(const real_T u);

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[28];

static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *w,
                                   const char_T *identifier))[7];

static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[7];

static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[12];

static real_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[28];

static real_T (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[7];

/* Function Definitions */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[12]
{
  real_T(*y)[12];
  y = i_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *n,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(n), &thisId);
  emlrtDestroyArray(&n);
  return y;
}

static real_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = j_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *T,
                                   const char_T *identifier))[28]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[28];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(T), &thisId);
  emlrtDestroyArray(&T);
  return y;
}

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *c,
                                 const char_T *identifier))[12]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[12];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(c), &thisId);
  emlrtDestroyArray(&c);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[28]
{
  real_T(*y)[28];
  y = k_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *w,
                                   const char_T *identifier))[7]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[7];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = h_emlrt_marshallIn(sp, emlrtAlias(w), &thisId);
  emlrtDestroyArray(&w);
  return y;
}

static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[7]
{
  real_T(*y)[7];
  y = l_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[12]
{
  static const int32_T dims = 12;
  real_T(*ret)[12];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)&dims);
  ret = (real_T(*)[12])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 0U, (void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[28]
{
  static const int32_T dims[2] = {4, 7};
  real_T(*ret)[28];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  ret = (real_T(*)[28])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[7]
{
  static const int32_T dims[2] = {1, 7};
  real_T(*ret)[7];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  ret = (real_T(*)[7])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

void RiemannEnergy1_api(const mxArray *const prhs[7], const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*T)[28];
  real_T(*T_dot)[28];
  real_T(*c)[12];
  real_T(*w)[7];
  real_T D;
  real_T N;
  real_T n;
  st.tls = emlrtRootTLSGlobal;
  /* Marshall function inputs */
  c = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "c");
  n = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "n");
  D = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "D");
  N = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "N");
  T = e_emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "T");
  T_dot = e_emlrt_marshallIn(&st, emlrtAlias(prhs[5]), "T_dot");
  w = g_emlrt_marshallIn(&st, emlrtAlias(prhs[6]), "w");
  /* Invoke the target function */
  n = RiemannEnergy1(&st, *c, n, D, N, *T, *T_dot, *w);
  /* Marshall function outputs */
  *plhs = emlrt_marshallOut(n);
}

/* End of code generation (_coder_RiemannEnergy1_api.c) */
