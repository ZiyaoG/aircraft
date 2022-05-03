/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * RiemannEnergy1.c
 *
 * Code generation for function 'RiemannEnergy1'
 *
 */

/* Include files */
#include "RiemannEnergy1.h"
#include "RiemannEnergy1_data.h"
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "warning.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    18,               /* lineNo */
    "RiemannEnergy1", /* fcnName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\RiemannEnergy1.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    21,               /* lineNo */
    "RiemannEnergy1", /* fcnName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\RiemannEnergy1.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    22,               /* lineNo */
    "RiemannEnergy1", /* fcnName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\RiemannEnergy1.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    25,               /* lineNo */
    "RiemannEnergy1", /* fcnName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\RiemannEnergy1.m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI = {
    29,                  /* lineNo */
    "reshapeSizeChecks", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pathName */
};

static emlrtRSInfo f_emlrtRSI = {
    109,               /* lineNo */
    "computeDimsData", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pathName */
};

static emlrtRSInfo g_emlrtRSI =
    {
        91,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

static emlrtRSInfo h_emlrtRSI =
    {
        60,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

static emlrtRSInfo k_emlrtRSI = {
    2,        /* lineNo */
    "W_fcn1", /* fcnName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\aircraft\\metric\\W_fcn1.m" /* pathName
                                                                      */
};

static emlrtRSInfo l_emlrtRSI =
    {
        71,      /* lineNo */
        "power", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\ops\\power.m" /* pathName
                                                                          */
};

static emlrtRSInfo m_emlrtRSI = {
    20,         /* lineNo */
    "mldivide", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\ops\\mldivide.m" /* pathName
                                                                         */
};

static emlrtRSInfo n_emlrtRSI = {
    42,      /* lineNo */
    "mldiv", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\ops\\mldivide.m" /* pathName
                                                                         */
};

static emlrtRSInfo
    o_emlrtRSI =
        {
            61,        /* lineNo */
            "lusolve", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\lusolve.m" /* pathName */
};

static emlrtRSInfo
    p_emlrtRSI =
        {
            293,          /* lineNo */
            "lusolve3x3", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\lusolve.m" /* pathName */
};

static emlrtRSInfo
    q_emlrtRSI =
        {
            90,              /* lineNo */
            "warn_singular", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\lusolve.m" /* pathName */
};

static emlrtRTEInfo emlrtRTEI = {
    24,               /* lineNo */
    7,                /* colNo */
    "RiemannEnergy1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\RiemannEnergy1.m" /* pName */
};

static emlrtBCInfo emlrtBCI = {
    1,                /* iFirst */
    7,                /* iLast */
    25,               /* lineNo */
    21,               /* colNo */
    "gamma_s",        /* aName */
    "RiemannEnergy1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\RiemannEnergy1.m", /* pName */
    0                           /* checkKind */
};

static emlrtRTEInfo b_emlrtRTEI = {
    52,                  /* lineNo */
    13,                  /* colNo */
    "reshapeSizeChecks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pName */
};

static emlrtRTEInfo c_emlrtRTEI = {
    57,                  /* lineNo */
    23,                  /* colNo */
    "reshapeSizeChecks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pName */
};

static emlrtRTEInfo d_emlrtRTEI = {
    59,                  /* lineNo */
    23,                  /* colNo */
    "reshapeSizeChecks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pName */
};

static emlrtBCInfo b_emlrtBCI = {
    -1,       /* iFirst */
    -1,       /* iLast */
    2,        /* lineNo */
    45,       /* colNo */
    "x",      /* aName */
    "W_fcn1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\aircraft\\metric\\W_fcn1.m", /* pName
                                                                       */
    0 /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = {
    -1,       /* iFirst */
    -1,       /* iLast */
    2,        /* lineNo */
    65,       /* colNo */
    "x",      /* aName */
    "W_fcn1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\aircraft\\metric\\W_fcn1.m", /* pName
                                                                       */
    0 /* checkKind */
};

static emlrtRTEInfo e_emlrtRTEI =
    {
        135,                   /* lineNo */
        23,                    /* colNo */
        "dynamic_size_checks", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pName */
};

static emlrtRTEInfo f_emlrtRTEI =
    {
        130,                   /* lineNo */
        23,                    /* colNo */
        "dynamic_size_checks", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pName */
};

static emlrtRTEInfo g_emlrtRTEI = {
    16,         /* lineNo */
    19,         /* colNo */
    "mldivide", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\ops\\mldivide.m" /* pName
                                                                         */
};

static emlrtRTEInfo h_emlrtRTEI = {
    58,                   /* lineNo */
    23,                   /* colNo */
    "assertValidSizeArg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\assertValidSizeArg.m" /* pName */
};

static emlrtRTEInfo i_emlrtRTEI = {
    64,                   /* lineNo */
    15,                   /* colNo */
    "assertValidSizeArg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\assertValidSizeArg.m" /* pName */
};

/* Function Definitions */
real_T RiemannEnergy1(const emlrtStack *sp, const real_T c[12], real_T n,
                      real_T D, real_T N, const real_T T[28],
                      const real_T T_dot[28], const real_T w[7])
{
  static const real_T a[9] = {177.44,  104.08,  -208.72, 104.08, -411.9,
                              -327.72, -208.72, -327.72, -353.74};
  static const real_T b_a[9] = {
      -226.225074800958, -209.545736836326, 204.181601981005,
      -209.545736836326, 58.4069021659569,  284.644562077584,
      204.181601981005,  284.644562077584,  478.371768229265};
  static const real_T y[9] = {43.692,  94.638, -68.29,  94.638, 340.7,
                              -24.757, -68.29, -24.757, 289.13};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  real_T c_data[144];
  real_T gamma_data[84];
  real_T gamma_s_data[84];
  real_T b_gamma_s_data[12];
  real_T W[9];
  real_T E;
  real_T maxval;
  real_T ml_idx_1;
  real_T ml_idx_2;
  int32_T c_size[2];
  int32_T gamma_s_size[2];
  int32_T gamma_size[2];
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  int32_T i4;
  int32_T k;
  int32_T loop_ub;
  int32_T r1;
  int32_T r2;
  int32_T r3;
  int32_T rtemp;
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
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  /*  "1" means for code generation  */
  /*  persistent c_pre E_pre; */
  /*  if isempty(c_pre) % adding this may make the gradient calculatioin
   * inaccurate */
  /*      c_pre = zeros(n,D+1); */
  /*      E_pre = 0; */
  /*  end */
  /*  Compute the Riemann Energy using the pseudospectral method */
  /*      gamma = zeros(n,N+1); */
  /*      gamma_s = zeros(n,N+1); */
  /*      for i = 1:n    */
  /*         gamma(i,:) = c((i-1)*(D+1)+1:i*(D+1),:)'*T;       % gamma(i) is
   * 1*(N+1); the ith elment of gamma on all the (N+1) nodes */
  /*         gamma_s(i,:) = c((i-1)*(D+1)+1:i*(D+1),:)'*T_dot; */
  /*      end    */
  /*  vectorized format to improve computational efficiency */
  st.site = &emlrtRSI;
  b_st.site = &e_emlrtRSI;
  c_st.site = &f_emlrtRSI;
  if ((D + 1.0 != muDoubleScalarFloor(D + 1.0)) ||
      muDoubleScalarIsInf(D + 1.0) || (D + 1.0 < -2.147483648E+9) ||
      (D + 1.0 > 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &h_emlrtRTEI, "Coder:MATLAB:NonIntegerInput",
        "Coder:MATLAB:NonIntegerInput", 4, 12, MIN_int32_T, 12, MAX_int32_T);
  }
  if (D + 1.0 <= 0.0) {
    maxval = 0.0;
  } else {
    maxval = D + 1.0;
  }
  if (!(maxval <= 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &i_emlrtRTEI, "Coder:MATLAB:pmaxsize",
                                  "Coder:MATLAB:pmaxsize", 0);
  }
  c_st.site = &f_emlrtRSI;
  if ((n != muDoubleScalarFloor(n)) || muDoubleScalarIsInf(n) ||
      (n < -2.147483648E+9) || (n > 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &h_emlrtRTEI, "Coder:MATLAB:NonIntegerInput",
        "Coder:MATLAB:NonIntegerInput", 4, 12, MIN_int32_T, 12, MAX_int32_T);
  }
  if (n <= 0.0) {
    maxval = 0.0;
  } else {
    maxval = n;
  }
  if (!(maxval <= 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &i_emlrtRTEI, "Coder:MATLAB:pmaxsize",
                                  "Coder:MATLAB:pmaxsize", 0);
  }
  if ((int32_T)(D + 1.0) > 12) {
    emlrtErrorWithMessageIdR2018a(&st, &b_emlrtRTEI,
                                  "Coder:toolbox:reshape_emptyReshapeLimit",
                                  "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }
  if ((int32_T)n > 12) {
    emlrtErrorWithMessageIdR2018a(&st, &b_emlrtRTEI,
                                  "Coder:toolbox:reshape_emptyReshapeLimit",
                                  "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }
  if (((int32_T)(D + 1.0) < 0) || ((int32_T)n < 0)) {
    emlrtErrorWithMessageIdR2018a(&st, &c_emlrtRTEI,
                                  "MATLAB:checkDimCommon:nonnegativeSize",
                                  "MATLAB:checkDimCommon:nonnegativeSize", 0);
  }
  if ((int32_T)(D + 1.0) * (int32_T)n != 12) {
    emlrtErrorWithMessageIdR2018a(
        &st, &d_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
        "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }
  c_size[0] = (int32_T)n;
  c_size[1] = (int32_T)(D + 1.0);
  rtemp = (int32_T)(D + 1.0);
  for (i = 0; i < rtemp; i++) {
    loop_ub = (int32_T)n;
    for (r2 = 0; r2 < loop_ub; r2++) {
      c_data[r2 + (int32_T)n * i] = c[i + (int32_T)(D + 1.0) * r2];
    }
  }
  /*  the ith row corresponds to the ith element */
  /*  if norm(c-c_pre)> 1e-8 */
  st.site = &b_emlrtRSI;
  b_st.site = &h_emlrtRSI;
  if ((int32_T)(D + 1.0) != 4) {
    if (((int32_T)n == 1) && ((int32_T)(D + 1.0) == 1)) {
      emlrtErrorWithMessageIdR2018a(
          &b_st, &f_emlrtRTEI, "Coder:toolbox:mtimes_noDynamicScalarExpansion",
          "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(&b_st, &e_emlrtRTEI, "MATLAB:innerdim",
                                    "MATLAB:innerdim", 0);
    }
  }
  b_st.site = &g_emlrtRSI;
  mtimes(c_data, c_size, T, gamma_data, gamma_size);
  st.site = &c_emlrtRSI;
  b_st.site = &h_emlrtRSI;
  b_st.site = &g_emlrtRSI;
  mtimes(c_data, c_size, T_dot, gamma_s_data, gamma_s_size);
  E = 0.0;
  i = (int32_T)(N + 1.0);
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, N + 1.0, mxDOUBLE_CLASS,
                                (int32_T)(N + 1.0), &emlrtRTEI, (emlrtCTX)sp);
  if (0 <= (int32_T)(N + 1.0) - 1) {
    i1 = gamma_size[0];
    i2 = gamma_size[0];
    i3 = gamma_s_size[0];
    i4 = gamma_s_size[0];
    b_loop_ub = gamma_s_size[0];
  }
  for (k = 0; k < i; k++) {
    if ((k + 1 < 1) || (k + 1 > 7)) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, 7, &emlrtBCI, (emlrtCTX)sp);
    }
    st.site = &d_emlrtRSI;
    b_st.site = &d_emlrtRSI;
    if (2 > i1) {
      emlrtDynamicBoundsCheckR2012b(2, 1, i1, &b_emlrtBCI, &b_st);
    }
    if (2 > i2) {
      emlrtDynamicBoundsCheckR2012b(2, 1, i2, &c_emlrtBCI, &b_st);
    }
    c_st.site = &k_emlrtRSI;
    d_st.site = &l_emlrtRSI;
    c_st.site = &k_emlrtRSI;
    d_st.site = &l_emlrtRSI;
    maxval = gamma_data[gamma_size[0] * k + 1];
    ml_idx_2 = maxval * maxval;
    for (r2 = 0; r2 < 9; r2++) {
      W[r2] = (y[r2] + a[r2] * maxval) + b_a[r2] * ml_idx_2;
    }
    b_st.site = &d_emlrtRSI;
    if (i3 != 3) {
      emlrtErrorWithMessageIdR2018a(&b_st, &g_emlrtRTEI, "MATLAB:dimagree",
                                    "MATLAB:dimagree", 0);
    }
    c_st.site = &m_emlrtRSI;
    d_st.site = &n_emlrtRSI;
    e_st.site = &o_emlrtRSI;
    r1 = 0;
    r2 = 1;
    r3 = 2;
    maxval = muDoubleScalarAbs(W[0]);
    ml_idx_2 = muDoubleScalarAbs(W[1]);
    if (ml_idx_2 > maxval) {
      maxval = ml_idx_2;
      r1 = 1;
      r2 = 0;
    }
    if (muDoubleScalarAbs(W[2]) > maxval) {
      r1 = 2;
      r2 = 1;
      r3 = 0;
    }
    W[r2] /= W[r1];
    W[r3] /= W[r1];
    W[r2 + 3] -= W[r2] * W[r1 + 3];
    W[r3 + 3] -= W[r3] * W[r1 + 3];
    W[r2 + 6] -= W[r2] * W[r1 + 6];
    W[r3 + 6] -= W[r3] * W[r1 + 6];
    if (muDoubleScalarAbs(W[r3 + 3]) > muDoubleScalarAbs(W[r2 + 3])) {
      rtemp = r2;
      r2 = r3;
      r3 = rtemp;
    }
    W[r3 + 3] /= W[r2 + 3];
    W[r3 + 6] -= W[r3 + 3] * W[r2 + 6];
    if ((W[r1] == 0.0) || (W[r2 + 3] == 0.0) || (W[r3 + 6] == 0.0)) {
      f_st.site = &p_emlrtRSI;
      g_st.site = &q_emlrtRSI;
      warning(&g_st);
    }
    loop_ub = gamma_s_size[0] * k;
    maxval = gamma_s_data[r1 + loop_ub];
    ml_idx_1 = gamma_s_data[r2 + loop_ub] - maxval * W[r2];
    ml_idx_2 =
        ((gamma_s_data[r3 + loop_ub] - maxval * W[r3]) - ml_idx_1 * W[r3 + 3]) /
        W[r3 + 6];
    ml_idx_1 -= ml_idx_2 * W[r2 + 6];
    ml_idx_1 /= W[r2 + 3];
    b_st.site = &h_emlrtRSI;
    if (i4 != 3) {
      if (gamma_s_size[0] == 1) {
        emlrtErrorWithMessageIdR2018a(
            &b_st, &f_emlrtRTEI,
            "Coder:toolbox:mtimes_noDynamicScalarExpansion",
            "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(&b_st, &e_emlrtRTEI, "MATLAB:innerdim",
                                      "MATLAB:innerdim", 0);
      }
    }
    /*          if tmp<0 */
    /*              pause; */
    /*          end */
    for (r2 = 0; r2 < b_loop_ub; r2++) {
      b_gamma_s_data[r2] = gamma_s_data[r2 + gamma_s_size[0] * k];
    }
    E += ((b_gamma_s_data[0] * (((gamma_s_data[r1 + gamma_s_size[0] * k] -
                                  ml_idx_2 * W[r1 + 6]) -
                                 ml_idx_1 * W[r1 + 3]) /
                                W[r1]) +
           b_gamma_s_data[1] * ml_idx_1) +
          b_gamma_s_data[2] * ml_idx_2) *
         w[k];
    /*  noite that W_fcn needs to be selected for each specific example.  */
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtCTX)sp);
    }
  }
  /*      c_pre = c; */
  /*      E_pre = E; */
  /*  else */
  /*      E = E_pre; */
  /*  end */
  return E;
}

/* End of code generation (RiemannEnergy1.c) */
