/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * norm.c
 *
 * Code generation for function 'norm'
 *
 */

/* Include files */
#include "norm.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo k_emlrtRSI = {
    47,     /* lineNo */
    "norm", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\matfun\\norm.m" /* pathName
                                                                        */
};

static emlrtRSInfo l_emlrtRSI = {
    69,         /* lineNo */
    "mat2norm", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\matfun\\norm.m" /* pathName
                                                                        */
};

static emlrtRSInfo m_emlrtRSI = {
    29,    /* lineNo */
    "svd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+internal\\svd.m" /* pathName
                                                                          */
};

static emlrtRSInfo n_emlrtRSI = {
    108,          /* lineNo */
    "callLAPACK", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+internal\\svd.m" /* pathName
                                                                          */
};

static emlrtRSInfo o_emlrtRSI = {
    31,       /* lineNo */
    "xgesvd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgesvd.m" /* pathName */
};

static emlrtRSInfo p_emlrtRSI = {
    197,            /* lineNo */
    "ceval_xgesvd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgesvd.m" /* pathName */
};

static emlrtRTEInfo k_emlrtRTEI = {
    47,          /* lineNo */
    13,          /* colNo */
    "infocheck", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\infocheck.m" /* pName */
};

static emlrtRTEInfo l_emlrtRTEI = {
    44,          /* lineNo */
    13,          /* colNo */
    "infocheck", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\infocheck.m" /* pName */
};

static emlrtRTEInfo m_emlrtRTEI = {
    112,          /* lineNo */
    5,            /* colNo */
    "callLAPACK", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+internal\\svd.m" /* pName
                                                                          */
};

/* Function Definitions */
real_T b_norm(const emlrtStack *sp, const real_T x_data[],
              const int32_T x_size[2])
{
  static const char_T fname[14] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                                   '_', 'd', 'g', 'e', 's', 'v', 'd'};
  ptrdiff_t incx_t;
  ptrdiff_t info_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  real_T A_data[144];
  real_T s_data[12];
  real_T superb_data[11];
  real_T absx;
  real_T y;
  int32_T i;
  int32_T j;
  int32_T m;
  int32_T n;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  if ((x_size[0] == 0) || (x_size[1] == 0)) {
    y = 0.0;
  } else if ((x_size[0] == 1) || (x_size[1] == 1)) {
    info_t = (ptrdiff_t)(x_size[0] * x_size[1]);
    incx_t = (ptrdiff_t)1;
    y = dnrm2(&info_t, &x_data[0], &incx_t);
  } else {
    st.site = &k_emlrtRSI;
    m = x_size[0];
    n = x_size[1];
    y = 0.0;
    for (j = 0; j < n; j++) {
      for (i = 0; i < m; i++) {
        absx = muDoubleScalarAbs(x_data[i + x_size[0] * j]);
        if (muDoubleScalarIsNaN(absx) || (absx > y)) {
          y = absx;
        }
      }
    }
    if ((!muDoubleScalarIsInf(y)) && (!muDoubleScalarIsNaN(y))) {
      b_st.site = &l_emlrtRSI;
      c_st.site = &m_emlrtRSI;
      d_st.site = &n_emlrtRSI;
      e_st.site = &o_emlrtRSI;
      m = x_size[0] * x_size[1];
      if (0 <= m - 1) {
        memcpy(&A_data[0], &x_data[0], m * sizeof(real_T));
      }
      info_t = LAPACKE_dgesvd(
          102, 'N', 'N', (ptrdiff_t)x_size[0], (ptrdiff_t)x_size[1], &A_data[0],
          (ptrdiff_t)x_size[0], &s_data[0], NULL, (ptrdiff_t)1, NULL,
          (ptrdiff_t)1, &superb_data[0]);
      m = (int32_T)info_t;
      f_st.site = &p_emlrtRSI;
      if (m < 0) {
        if (m == -1010) {
          emlrtErrorWithMessageIdR2018a(&f_st, &l_emlrtRTEI, "MATLAB:nomem",
                                        "MATLAB:nomem", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(
              &f_st, &k_emlrtRTEI, "Coder:toolbox:LAPACKCallErrorInfo",
              "Coder:toolbox:LAPACKCallErrorInfo", 5, 4, 14, &fname[0], 12, m);
        }
      }
      if (m > 0) {
        emlrtErrorWithMessageIdR2018a(&c_st, &m_emlrtRTEI,
                                      "Coder:MATLAB:svd_NoConvergence",
                                      "Coder:MATLAB:svd_NoConvergence", 0);
      }
      y = s_data[0];
    }
  }
  return y;
}

/* End of code generation (norm.c) */
