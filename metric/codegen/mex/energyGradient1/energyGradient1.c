/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * energyGradient1.c
 *
 * Code generation for function 'energyGradient1'
 *
 */

/* Include files */
#include "energyGradient1.h"
#include "assertValidSizeArg.h"
#include "energyGradient1_data.h"
#include "energyGradient1_emxutil.h"
#include "energyGradient1_types.h"
#include "mldivide.h"
#include "mtimes.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emxArray_real_T *c_pre;

static boolean_T c_pre_not_empty;

static emxArray_real_T *g_pre;

static emlrtRSInfo emlrtRSI = {
    16,                /* lineNo */
    "energyGradient1", /* fcnName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    17,                /* lineNo */
    "energyGradient1", /* fcnName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    18,                /* lineNo */
    "energyGradient1", /* fcnName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    19,                /* lineNo */
    "energyGradient1", /* fcnName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI = {
    28,                /* lineNo */
    "energyGradient1", /* fcnName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pathName */
};

static emlrtRSInfo f_emlrtRSI = {
    30,                /* lineNo */
    "energyGradient1", /* fcnName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pathName */
};

static emlrtRSInfo g_emlrtRSI = {
    33,                /* lineNo */
    "energyGradient1", /* fcnName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pathName */
};

static emlrtRSInfo h_emlrtRSI = {
    35,                /* lineNo */
    "energyGradient1", /* fcnName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pathName */
};

static emlrtRSInfo i_emlrtRSI = {
    29,                  /* lineNo */
    "reshapeSizeChecks", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pathName */
};

static emlrtRSInfo j_emlrtRSI = {
    109,               /* lineNo */
    "computeDimsData", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pathName */
};

static emlrtRSInfo q_emlrtRSI =
    {
        91,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

static emlrtRSInfo r_emlrtRSI =
    {
        60,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

static emlrtRSInfo u_emlrtRSI = {
    2,        /* lineNo */
    "W_fcn1", /* fcnName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\aircraft\\metric\\W_fcn1.m" /* pathName
                                                                      */
};

static emlrtRSInfo cb_emlrtRSI = {
    4,             /* lineNo */
    "dW_dxi_fcn1", /* fcnName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\aircraft\\metric\\dW_dxi_fcn1.m" /* pathName
                                                                           */
};

static emlrtRSInfo eb_emlrtRSI = {
    24,    /* lineNo */
    "cat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pathName
                                                                          */
};

static emlrtRSInfo fb_emlrtRSI = {
    96,         /* lineNo */
    "cat_impl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pathName
                                                                          */
};

static emlrtRTEInfo emlrtRTEI =
    {
        130,                   /* lineNo */
        23,                    /* colNo */
        "dynamic_size_checks", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pName */
};

static emlrtRTEInfo b_emlrtRTEI =
    {
        135,                   /* lineNo */
        23,                    /* colNo */
        "dynamic_size_checks", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pName */
};

static emlrtECInfo emlrtECI = {
    2,                 /* nDims */
    17,                /* lineNo */
    9,                 /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

static emlrtRTEInfo c_emlrtRTEI = {
    24,                /* lineNo */
    9,                 /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

static emlrtBCInfo emlrtBCI = {
    1,                 /* iFirst */
    7,                 /* iLast */
    28,                /* lineNo */
    33,                /* colNo */
    "gamma",           /* aName */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    0                            /* checkKind */
};

static emlrtRTEInfo d_emlrtRTEI = {
    32,                /* lineNo */
    13,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

static emlrtDCInfo emlrtDCI = {
    35,                /* lineNo */
    83,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    4                            /* checkKind */
};

static emlrtDCInfo b_emlrtDCI = {
    35,                /* lineNo */
    83,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo c_emlrtDCI = {
    35,                /* lineNo */
    96,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo b_emlrtBCI = {
    1,                 /* iFirst */
    4,                 /* iLast */
    35,                /* lineNo */
    96,                /* colNo */
    "T_dot",           /* aName */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo d_emlrtDCI = {
    35,                /* lineNo */
    146,               /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = {
    1,                 /* iFirst */
    4,                 /* iLast */
    35,                /* lineNo */
    146,               /* colNo */
    "T",               /* aName */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    0                            /* checkKind */
};

static emlrtECInfo b_emlrtECI = {
    2,                 /* nDims */
    35,                /* lineNo */
    72,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

static emlrtECInfo c_emlrtECI = {
    2,                 /* nDims */
    35,                /* lineNo */
    34,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

static emlrtECInfo d_emlrtECI = {
    -1,                /* nDims */
    35,                /* lineNo */
    9,                 /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

static emlrtRTEInfo e_emlrtRTEI = {
    52,                  /* lineNo */
    13,                  /* colNo */
    "reshapeSizeChecks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pName */
};

static emlrtRTEInfo f_emlrtRTEI = {
    57,                  /* lineNo */
    23,                  /* colNo */
    "reshapeSizeChecks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pName */
};

static emlrtRTEInfo g_emlrtRTEI = {
    59,                  /* lineNo */
    23,                  /* colNo */
    "reshapeSizeChecks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pName */
};

static emlrtRTEInfo h_emlrtRTEI = {
    271,                   /* lineNo */
    27,                    /* colNo */
    "check_non_axis_size", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2021a\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pName
                                                                          */
};

static emlrtBCInfo d_emlrtBCI = {
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

static emlrtBCInfo
    e_emlrtBCI =
        {
            -1,              /* iFirst */
            -1,              /* iLast */
            2,               /* lineNo */
            72,              /* colNo */
            "x",             /* aName */
            "dW_dalpha_fcn", /* fName */
            "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\aircraft\\metric\\dW_dalpha_"
            "fcn.m", /* pName */
            0        /* checkKind */
};

static emlrtDCInfo e_emlrtDCI = {
    6,                 /* lineNo */
    19,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo f_emlrtDCI = {
    6,                 /* lineNo */
    19,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    4                            /* checkKind */
};

static emlrtDCInfo g_emlrtDCI = {
    6,                 /* lineNo */
    21,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo h_emlrtDCI = {
    6,                 /* lineNo */
    21,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    4                            /* checkKind */
};

static emlrtDCInfo i_emlrtDCI = {
    7,                 /* lineNo */
    21,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo j_emlrtDCI = {
    7,                 /* lineNo */
    21,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    4                            /* checkKind */
};

static emlrtDCInfo k_emlrtDCI = {
    20,                /* lineNo */
    13,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo l_emlrtDCI = {
    20,                /* lineNo */
    13,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    4                            /* checkKind */
};

static emlrtDCInfo m_emlrtDCI = {
    35,                /* lineNo */
    110,               /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo n_emlrtDCI = {
    35,                /* lineNo */
    110,               /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    4                            /* checkKind */
};

static emlrtDCInfo o_emlrtDCI = {
    35,                /* lineNo */
    114,               /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo p_emlrtDCI = {
    35,                /* lineNo */
    114,               /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    4                            /* checkKind */
};

static emlrtDCInfo q_emlrtDCI = {
    6,                 /* lineNo */
    5,                 /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo r_emlrtDCI = {
    6,                 /* lineNo */
    5,                 /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    4                            /* checkKind */
};

static emlrtDCInfo s_emlrtDCI = {
    7,                 /* lineNo */
    5,                 /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo t_emlrtDCI = {
    20,                /* lineNo */
    1,                 /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo u_emlrtDCI = {
    20,                /* lineNo */
    1,                 /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    4                            /* checkKind */
};

static emlrtDCInfo v_emlrtDCI = {
    35,                /* lineNo */
    104,               /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo w_emlrtDCI = {
    35,                /* lineNo */
    104,               /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    4                            /* checkKind */
};

static emlrtDCInfo x_emlrtDCI = {
    35,                /* lineNo */
    36,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    35,                /* lineNo */
    36,                /* colNo */
    "g",               /* aName */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo g_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    35,                /* lineNo */
    11,                /* colNo */
    "g",               /* aName */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m", /* pName */
    0                            /* checkKind */
};

static emlrtRTEInfo o_emlrtRTEI = {
    4,                 /* lineNo */
    18,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

static emlrtRTEInfo p_emlrtRTEI = {
    4,                 /* lineNo */
    12,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

static emlrtRTEInfo q_emlrtRTEI = {
    6,                 /* lineNo */
    5,                 /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

static emlrtRTEInfo r_emlrtRTEI = {
    7,                 /* lineNo */
    5,                 /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

static emlrtRTEInfo s_emlrtRTEI = {
    41,                /* lineNo */
    5,                 /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

static emlrtRTEInfo t_emlrtRTEI = {
    20,                /* lineNo */
    1,                 /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

static emlrtRTEInfo u_emlrtRTEI = {
    38,                /* lineNo */
    5,                 /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

static emlrtRTEInfo v_emlrtRTEI = {
    39,                /* lineNo */
    5,                 /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

static emlrtRTEInfo w_emlrtRTEI = {
    35,                /* lineNo */
    72,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

static emlrtRTEInfo x_emlrtRTEI =
    {
        28,      /* lineNo */
        9,       /* colNo */
        "colon", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\ops\\colon.m" /* pName
                                                                          */
};

static emlrtRTEInfo y_emlrtRTEI = {
    35,                /* lineNo */
    36,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

static emlrtRTEInfo ab_emlrtRTEI = {
    35,                /* lineNo */
    11,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

static emlrtRTEInfo bb_emlrtRTEI = {
    35,                /* lineNo */
    34,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

static emlrtRTEInfo cb_emlrtRTEI = {
    1,                 /* lineNo */
    14,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

static emlrtRTEInfo db_emlrtRTEI = {
    35,                /* lineNo */
    49,                /* colNo */
    "energyGradient1", /* fName */
    "D:\\Boran\\Dropbox\\Codes\\CCM-DNN\\ccm\\control_law_"
    "online\\energyGradient1.m" /* pName */
};

/* Function Definitions */
void energyGradient1(const emlrtStack *sp, const real_T c[12], real_T n,
                     real_T D, real_T N, const real_T T[28],
                     const real_T T_dot[28], const real_T w[7],
                     emxArray_real_T *g)
{
  static const real_T a[9] = {177.44,  104.08,  -208.72, 104.08, -411.9,
                              -327.72, -208.72, -327.72, -353.74};
  static const real_T b_a[9] = {
      -226.225074800958, -209.545736836326, 204.181601981005,
      -209.545736836326, 58.4069021659569,  284.644562077584,
      204.181601981005,  284.644562077584,  478.371768229265};
  static const real_T c_y[9] = {43.692,  94.638, -68.29,  94.638, 340.7,
                                -24.757, -68.29, -24.757, 289.13};
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_int32_T *r1;
  emxArray_real_T *b_g;
  emxArray_real_T *r;
  emxArray_real_T *y;
  real_T b_c_data[144];
  real_T c_data[144];
  real_T gamma_data[84];
  real_T gamma_s_data[84];
  real_T b_gamma_s_data[12];
  real_T b_y[9];
  real_T T_dot_data[4];
  real_T M_x_gamma_sk[3];
  real_T ml[3];
  real_T alpha1;
  real_T b_w;
  real_T beta1;
  real_T d;
  real_T d1;
  real_T d2;
  real_T d3;
  int32_T B_size[2];
  int32_T T_dot_size[2];
  int32_T b_c_size[2];
  int32_T c_size[2];
  int32_T gamma_s_size[2];
  int32_T iv[2];
  int32_T b_i;
  int32_T b_loop_ub;
  int32_T c_loop_ub;
  int32_T d_loop_ub;
  int32_T e_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  int32_T i4;
  int32_T input_sizes_idx_0_tmp;
  int32_T k;
  int32_T loop_ub;
  int32_T sizes_idx_0;
  char_T TRANSA1;
  char_T TRANSB1;
  int8_T input_sizes_idx_0;
  boolean_T c_a;
  boolean_T empty_non_axis_sizes;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtCTX)sp);
  /*  Compute the gradient of the Riemann Energy under the pseudospectral method
   */
  /*  for code generation */
  if (!c_pre_not_empty) {
    /*  adding this may make the gradient calculatioin inaccurate */
    if (!(n >= 0.0)) {
      emlrtNonNegativeCheckR2012b(n, &f_emlrtDCI, (emlrtCTX)sp);
    }
    beta1 = (int32_T)muDoubleScalarFloor(n);
    if (n != beta1) {
      emlrtIntegerCheckR2012b(n, &e_emlrtDCI, (emlrtCTX)sp);
    }
    i = c_pre->size[0] * c_pre->size[1];
    c_pre->size[0] = (int32_T)n;
    emxEnsureCapacity_real_T(sp, c_pre, i, &q_emlrtRTEI);
    if (!(D + 1.0 >= 0.0)) {
      emlrtNonNegativeCheckR2012b(D + 1.0, &h_emlrtDCI, (emlrtCTX)sp);
    }
    alpha1 = (int32_T)muDoubleScalarFloor(D + 1.0);
    if (D + 1.0 != alpha1) {
      emlrtIntegerCheckR2012b(D + 1.0, &g_emlrtDCI, (emlrtCTX)sp);
    }
    i = c_pre->size[0] * c_pre->size[1];
    c_pre->size[1] = (int32_T)(D + 1.0);
    emxEnsureCapacity_real_T(sp, c_pre, i, &q_emlrtRTEI);
    if (n != beta1) {
      emlrtIntegerCheckR2012b(n, &q_emlrtDCI, (emlrtCTX)sp);
    }
    if (!(D + 1.0 >= 0.0)) {
      emlrtNonNegativeCheckR2012b(D + 1.0, &r_emlrtDCI, (emlrtCTX)sp);
    }
    if (D + 1.0 != alpha1) {
      emlrtIntegerCheckR2012b(D + 1.0, &q_emlrtDCI, (emlrtCTX)sp);
    }
    loop_ub = (int32_T)n * (int32_T)(D + 1.0);
    for (i = 0; i < loop_ub; i++) {
      c_pre->data[i] = 0.0;
    }
    c_pre_not_empty = ((c_pre->size[0] != 0) && (c_pre->size[1] != 0));
    i = g_pre->size[0] * g_pre->size[1];
    g_pre->size[0] = 1;
    emxEnsureCapacity_real_T(sp, g_pre, i, &r_emlrtRTEI);
    beta1 = (D + 1.0) * n;
    if (!(beta1 >= 0.0)) {
      emlrtNonNegativeCheckR2012b(beta1, &j_emlrtDCI, (emlrtCTX)sp);
    }
    alpha1 = (int32_T)muDoubleScalarFloor(beta1);
    if (beta1 != alpha1) {
      emlrtIntegerCheckR2012b(beta1, &i_emlrtDCI, (emlrtCTX)sp);
    }
    i = g_pre->size[0] * g_pre->size[1];
    loop_ub = (int32_T)beta1;
    g_pre->size[1] = (int32_T)beta1;
    emxEnsureCapacity_real_T(sp, g_pre, i, &r_emlrtRTEI);
    if (beta1 != alpha1) {
      emlrtIntegerCheckR2012b(beta1, &s_emlrtDCI, (emlrtCTX)sp);
    }
    for (i = 0; i < loop_ub; i++) {
      g_pre->data[i] = 0.0;
    }
  }
  /*      gamma = zeros(n,N+1); */
  /*      gamma_s = zeros(n,N+1); */
  /*      for i = 1:n    */
  /*         gamma(i,:) = c((i-1)*(D+1)+1:i*(D+1),:)'*T;       % gamma(i) is
   * 1*(N+1); the ith elment of gamma on all the (N+1) nodes */
  /*         gamma_s(i,:) = c((i-1)*(D+1)+1:i*(D+1),:)'*T_dot; */
  /*      end  */
  st.site = &emlrtRSI;
  b_st.site = &i_emlrtRSI;
  c_st.site = &j_emlrtRSI;
  assertValidSizeArg(&c_st, D + 1.0);
  c_st.site = &j_emlrtRSI;
  assertValidSizeArg(&c_st, n);
  if ((int32_T)(D + 1.0) > 12) {
    emlrtErrorWithMessageIdR2018a(&st, &e_emlrtRTEI,
                                  "Coder:toolbox:reshape_emptyReshapeLimit",
                                  "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }
  if ((int32_T)n > 12) {
    emlrtErrorWithMessageIdR2018a(&st, &e_emlrtRTEI,
                                  "Coder:toolbox:reshape_emptyReshapeLimit",
                                  "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }
  if (((int32_T)(D + 1.0) < 0) || ((int32_T)n < 0)) {
    emlrtErrorWithMessageIdR2018a(&st, &f_emlrtRTEI,
                                  "MATLAB:checkDimCommon:nonnegativeSize",
                                  "MATLAB:checkDimCommon:nonnegativeSize", 0);
  }
  if ((int32_T)(D + 1.0) * (int32_T)n != 12) {
    emlrtErrorWithMessageIdR2018a(
        &st, &g_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
        "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }
  c_size[0] = (int32_T)n;
  c_size[1] = (int32_T)(D + 1.0);
  loop_ub = (int32_T)(D + 1.0);
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = (int32_T)n;
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      c_data[i1 + (int32_T)n * i] = c[i + (int32_T)(D + 1.0) * i1];
    }
  }
  /*  the ith row corresponds to the ith element */
  iv[0] = (*(int32_T(*)[2])c_pre->size)[0];
  iv[1] = (*(int32_T(*)[2])c_pre->size)[1];
  emlrtSizeEqCheckNDR2012b(&c_size[0], &iv[0], &emlrtECI, (emlrtCTX)sp);
  b_c_size[0] = (int32_T)n;
  b_c_size[1] = (int32_T)(D + 1.0);
  loop_ub = (int32_T)n * (int32_T)(D + 1.0);
  for (i = 0; i < loop_ub; i++) {
    b_c_data[i] = c_data[i] - c_pre->data[i];
  }
  st.site = &b_emlrtRSI;
  if (b_norm(&st, b_c_data, b_c_size) > 1.0E-5) {
    st.site = &c_emlrtRSI;
    b_st.site = &r_emlrtRSI;
    if ((int32_T)(D + 1.0) != 4) {
      if (((int32_T)n == 1) && ((int32_T)(D + 1.0) == 1)) {
        emlrtErrorWithMessageIdR2018a(
            &b_st, &emlrtRTEI, "Coder:toolbox:mtimes_noDynamicScalarExpansion",
            "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(&b_st, &b_emlrtRTEI, "MATLAB:innerdim",
                                      "MATLAB:innerdim", 0);
      }
    }
    b_st.site = &q_emlrtRSI;
    mtimes(c_data, c_size, T, gamma_data, b_c_size);
    st.site = &d_emlrtRSI;
    b_st.site = &r_emlrtRSI;
    b_st.site = &q_emlrtRSI;
    mtimes(c_data, c_size, T_dot, gamma_s_data, gamma_s_size);
    i = g->size[0] * g->size[1];
    g->size[0] = 1;
    emxEnsureCapacity_real_T(sp, g, i, &t_emlrtRTEI);
    beta1 = (D + 1.0) * n;
    if (!(beta1 >= 0.0)) {
      emlrtNonNegativeCheckR2012b(beta1, &l_emlrtDCI, (emlrtCTX)sp);
    }
    if (beta1 != (int32_T)muDoubleScalarFloor(beta1)) {
      emlrtIntegerCheckR2012b(beta1, &k_emlrtDCI, (emlrtCTX)sp);
    }
    i = g->size[0] * g->size[1];
    g->size[1] = (int32_T)beta1;
    emxEnsureCapacity_real_T(sp, g, i, &t_emlrtRTEI);
    beta1 = (D + 1.0) * n;
    if (!(beta1 >= 0.0)) {
      emlrtNonNegativeCheckR2012b(beta1, &u_emlrtDCI, (emlrtCTX)sp);
    }
    if (beta1 != (int32_T)muDoubleScalarFloor(beta1)) {
      emlrtIntegerCheckR2012b(beta1, &t_emlrtDCI, (emlrtCTX)sp);
    }
    loop_ub = (int32_T)beta1;
    for (i = 0; i < loop_ub; i++) {
      g->data[i] = 0.0;
    }
    /*      M_x_gamma_s = zeros(n,N+1); */
    /*  vectorized format */
    i = (int32_T)(N + 1.0);
    emlrtForLoopVectorCheckR2021a(1.0, 1.0, N + 1.0, mxDOUBLE_CLASS,
                                  (int32_T)(N + 1.0), &c_emlrtRTEI,
                                  (emlrtCTX)sp);
    emxInit_real_T(sp, &r, 2, &cb_emlrtRTEI, true);
    emxInit_int32_T(sp, &r1, 2, &cb_emlrtRTEI, true);
    emxInit_real_T(sp, &y, 2, &db_emlrtRTEI, true);
    emxInit_real_T(sp, &b_g, 1, &bb_emlrtRTEI, true);
    if (0 <= i - 1) {
      c_loop_ub = gamma_s_size[0];
      i2 = (int32_T)n;
    }
    for (k = 0; k < i; k++) {
      /*      if norm(gamma(:,k))> 10 */
      /*          disp('gamma norm is out of range'); */
      /*      end */
      st.site = &e_emlrtRSI;
      if ((k + 1 < 1) || (k + 1 > 7)) {
        emlrtDynamicBoundsCheckR2012b(k + 1, 1, 7, &emlrtBCI, &st);
      }
      if (2 > b_c_size[0]) {
        emlrtDynamicBoundsCheckR2012b(2, 1, b_c_size[0], &d_emlrtBCI, &st);
      }
      b_st.site = &u_emlrtRSI;
      b_st.site = &u_emlrtRSI;
      alpha1 = gamma_data[b_c_size[0] * k + 1];
      /*          M_x_gamma_sk = (W_fcn(gamma(:,k))\gamma_s(:,k)); */
      beta1 = alpha1 * alpha1;
      for (i1 = 0; i1 < 9; i1++) {
        b_y[i1] = (c_y[i1] + a[i1] * alpha1) + b_a[i1] * beta1;
      }
      for (i1 = 0; i1 < c_loop_ub; i1++) {
        b_gamma_s_data[i1] = gamma_s_data[i1 + gamma_s_size[0] * k];
      }
      st.site = &f_emlrtRSI;
      mldivide(&st, b_y, b_gamma_s_data, gamma_s_size[0], M_x_gamma_sk);
      /*          coder.extrinsic('dW_fcn'); */
      emlrtForLoopVectorCheckR2021a(1.0, 1.0, n, mxDOUBLE_CLASS, (int32_T)n,
                                    &d_emlrtRTEI, (emlrtCTX)sp);
      if (0 <= (int32_T)n - 1) {
        i3 = b_c_size[0];
        if (1.0 > D + 1.0) {
          d_loop_ub = 0;
          e_loop_ub = 0;
        } else {
          beta1 = (int32_T)muDoubleScalarFloor(D + 1.0);
          if (D + 1.0 != beta1) {
            emlrtIntegerCheckR2012b(D + 1.0, &c_emlrtDCI, (emlrtCTX)sp);
          }
          if (((int32_T)(D + 1.0) < 1) || ((int32_T)(D + 1.0) > 4)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)(D + 1.0), 1, 4, &b_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          d_loop_ub = (int32_T)(D + 1.0);
          if (D + 1.0 != beta1) {
            emlrtIntegerCheckR2012b(D + 1.0, &d_emlrtDCI, (emlrtCTX)sp);
          }
          if (((int32_T)(D + 1.0) < 1) || ((int32_T)(D + 1.0) > 4)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)(D + 1.0), 1, 4, &c_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          e_loop_ub = (int32_T)(D + 1.0);
        }
        b_w = w[k];
      }
      if (0 <= i2 - 1) {
        d = (int32_T)muDoubleScalarFloor(D + 1.0);
        d1 = M_x_gamma_sk[0];
        d2 = M_x_gamma_sk[1];
        d3 = M_x_gamma_sk[2];
        B_size[0] = 3;
        T_dot_size[0] = 1;
      }
      for (b_i = 0; b_i < i2; b_i++) {
        st.site = &g_emlrtRSI;
        c_a = (b_i + 1U == 2U);
        b_st.site = &cb_emlrtRSI;
        if (2 > i3) {
          emlrtDynamicBoundsCheckR2012b(2, 1, i3, &e_emlrtBCI, &b_st);
        }
        ml[2] = 2.0 * gamma_data[b_c_size[0] * k + 1];
        /*              dW_dxi = dW_fcn{i}(gamma(:,k));  */
        st.site = &h_emlrtRSI;
        if (!(D + 1.0 >= 0.0)) {
          emlrtNonNegativeCheckR2012b(D + 1.0, &emlrtDCI, &st);
        }
        if (D + 1.0 != d) {
          emlrtIntegerCheckR2012b(D + 1.0, &b_emlrtDCI, &st);
        }
        beta1 = n - ((real_T)b_i + 1.0);
        if (!(beta1 >= 0.0)) {
          emlrtNonNegativeCheckR2012b(beta1, &n_emlrtDCI, &st);
        }
        alpha1 = (int32_T)muDoubleScalarFloor(beta1);
        if (beta1 != alpha1) {
          emlrtIntegerCheckR2012b(beta1, &m_emlrtDCI, &st);
        }
        if (!(D + 1.0 >= 0.0)) {
          emlrtNonNegativeCheckR2012b(D + 1.0, &p_emlrtDCI, &st);
        }
        if (D + 1.0 != d) {
          emlrtIntegerCheckR2012b(D + 1.0, &o_emlrtDCI, &st);
        }
        if (beta1 != alpha1) {
          emlrtIntegerCheckR2012b(beta1, &v_emlrtDCI, &st);
        }
        if (!(D + 1.0 >= 0.0)) {
          emlrtNonNegativeCheckR2012b(D + 1.0, &w_emlrtDCI, &st);
        }
        if (D + 1.0 != d) {
          emlrtIntegerCheckR2012b(D + 1.0, &v_emlrtDCI, &st);
        }
        b_st.site = &eb_emlrtRSI;
        if ((b_i != 0) && ((int32_T)(D + 1.0) != 0)) {
          b_loop_ub = (int32_T)(D + 1.0);
        } else if ((int8_T)d_loop_ub != 0) {
          b_loop_ub = (int8_T)d_loop_ub;
        } else if (((int32_T)beta1 != 0) && ((int32_T)(D + 1.0) != 0)) {
          b_loop_ub = (int32_T)(D + 1.0);
        } else {
          i1 = (int32_T)(D + 1.0);
          b_loop_ub = muIntScalarMax_sint32(i1, 0);
          if ((int8_T)d_loop_ub > b_loop_ub) {
            b_loop_ub = (int8_T)d_loop_ub;
          }
          if ((int32_T)(D + 1.0) > b_loop_ub) {
            b_loop_ub = (int32_T)(D + 1.0);
          }
        }
        c_st.site = &fb_emlrtRSI;
        if (((int32_T)(D + 1.0) != b_loop_ub) &&
            ((b_i != 0) && ((int32_T)(D + 1.0) != 0))) {
          emlrtErrorWithMessageIdR2018a(
              &c_st, &h_emlrtRTEI, "MATLAB:catenate:matrixDimensionMismatch",
              "MATLAB:catenate:matrixDimensionMismatch", 0);
        }
        if (((int8_T)d_loop_ub != b_loop_ub) && ((int8_T)d_loop_ub != 0)) {
          emlrtErrorWithMessageIdR2018a(
              &c_st, &h_emlrtRTEI, "MATLAB:catenate:matrixDimensionMismatch",
              "MATLAB:catenate:matrixDimensionMismatch", 0);
        }
        if (((int32_T)(D + 1.0) != b_loop_ub) &&
            (((int32_T)(n - ((real_T)b_i + 1.0)) != 0) &&
             ((int32_T)(D + 1.0) != 0))) {
          emlrtErrorWithMessageIdR2018a(
              &c_st, &h_emlrtRTEI, "MATLAB:catenate:matrixDimensionMismatch",
              "MATLAB:catenate:matrixDimensionMismatch", 0);
        }
        empty_non_axis_sizes = (b_loop_ub == 0);
        if (empty_non_axis_sizes || ((b_i != 0) && ((int32_T)(D + 1.0) != 0))) {
          loop_ub = b_i;
        } else {
          loop_ub = 0;
        }
        if (empty_non_axis_sizes || ((int8_T)d_loop_ub != 0)) {
          input_sizes_idx_0 = 1;
        } else {
          input_sizes_idx_0 = 0;
        }
        if (empty_non_axis_sizes ||
            (((int32_T)(n - ((real_T)b_i + 1.0)) != 0) &&
             ((int32_T)(D + 1.0) != 0))) {
          sizes_idx_0 = (int32_T)(n - ((real_T)b_i + 1.0));
        } else {
          sizes_idx_0 = 0;
        }
        if (0 <= d_loop_ub - 1) {
          memcpy(&T_dot_data[0], &T_dot[k * 4], d_loop_ub * sizeof(real_T));
        }
        input_sizes_idx_0_tmp = input_sizes_idx_0;
        i1 = r->size[0] * r->size[1];
        r->size[0] = (loop_ub + input_sizes_idx_0) + sizes_idx_0;
        r->size[1] = b_loop_ub;
        emxEnsureCapacity_real_T(sp, r, i1, &w_emlrtRTEI);
        for (i1 = 0; i1 < b_loop_ub; i1++) {
          for (i4 = 0; i4 < loop_ub; i4++) {
            r->data[i4 + r->size[0] * i1] = 0.0;
          }
        }
        for (i1 = 0; i1 < b_loop_ub; i1++) {
          for (i4 = 0; i4 < input_sizes_idx_0_tmp; i4++) {
            r->data[loop_ub + r->size[0] * i1] =
                T_dot_data[input_sizes_idx_0 * i1] * 2.0;
          }
        }
        for (i1 = 0; i1 < b_loop_ub; i1++) {
          for (i4 = 0; i4 < sizes_idx_0; i4++) {
            r->data[((i4 + loop_ub) + input_sizes_idx_0) + r->size[0] * i1] =
                0.0;
          }
        }
        for (i1 = 0; i1 < 9; i1++) {
          b_y[i1] = (real_T)c_a * (a[i1] + b_a[i1] * ml[2]);
        }
        for (i1 = 0; i1 < 3; i1++) {
          ml[i1] = (b_y[i1] * d1 + b_y[i1 + 3] * d2) + b_y[i1 + 6] * d3;
        }
        B_size[1] = e_loop_ub;
        for (i1 = 0; i1 < e_loop_ub; i1++) {
          alpha1 = T[i1 + (k << 2)];
          b_gamma_s_data[3 * i1] = ml[0] * alpha1;
          b_gamma_s_data[3 * i1 + 1] = ml[1] * alpha1;
          b_gamma_s_data[3 * i1 + 2] = ml[2] * alpha1;
        }
        iv[0] = (*(int32_T(*)[2])r->size)[0];
        iv[1] = (*(int32_T(*)[2])r->size)[1];
        emlrtSizeEqCheckNDR2012b(&iv[0], &B_size[0], &b_emlrtECI, (emlrtCTX)sp);
        if (muDoubleScalarIsNaN(D + 1.0)) {
          i1 = y->size[0] * y->size[1];
          y->size[0] = 1;
          y->size[1] = 1;
          emxEnsureCapacity_real_T(sp, y, i1, &x_emlrtRTEI);
          y->data[0] = rtNaN;
        } else if (D + 1.0 < 1.0) {
          y->size[1] = 0;
        } else if (muDoubleScalarIsInf(D + 1.0) && (1.0 == D + 1.0)) {
          i1 = y->size[0] * y->size[1];
          y->size[0] = 1;
          y->size[1] = 1;
          emxEnsureCapacity_real_T(sp, y, i1, &x_emlrtRTEI);
          y->data[0] = rtNaN;
        } else {
          i1 = y->size[0] * y->size[1];
          y->size[0] = 1;
          loop_ub = (int32_T)muDoubleScalarFloor((D + 1.0) - 1.0);
          y->size[1] = loop_ub + 1;
          emxEnsureCapacity_real_T(sp, y, i1, &x_emlrtRTEI);
          for (i1 = 0; i1 <= loop_ub; i1++) {
            y->data[i1] = (real_T)i1 + 1.0;
          }
        }
        i1 = y->size[0] * y->size[1];
        y->size[0] = 1;
        emxEnsureCapacity_real_T(sp, y, i1, &y_emlrtRTEI);
        alpha1 = (((real_T)b_i + 1.0) - 1.0) * (D + 1.0);
        loop_ub = y->size[1] - 1;
        for (i1 = 0; i1 <= loop_ub; i1++) {
          beta1 = alpha1 + y->data[i1];
          if (beta1 != (int32_T)muDoubleScalarFloor(beta1)) {
            emlrtIntegerCheckR2012b(beta1, &x_emlrtDCI, (emlrtCTX)sp);
          }
          y->data[i1] = beta1;
        }
        loop_ub = y->size[1];
        for (i1 = 0; i1 < loop_ub; i1++) {
          i4 = (int32_T)y->data[i1];
          if ((i4 < 1) || (i4 > g->size[1])) {
            emlrtDynamicBoundsCheckR2012b(i4, 1, g->size[1], &f_emlrtBCI,
                                          (emlrtCTX)sp);
          }
        }
        st.site = &h_emlrtRSI;
        b_st.site = &q_emlrtRSI;
        loop_ub = r->size[0] * r->size[1];
        B_size[1] = r->size[1];
        for (i1 = 0; i1 < loop_ub; i1++) {
          b_gamma_s_data[i1] = r->data[i1] - b_gamma_s_data[i1];
        }
        if (B_size[1] == 0) {
          i1 = 0;
          T_dot_size[1] = 0;
        } else {
          TRANSB1 = 'N';
          TRANSA1 = 'T';
          alpha1 = 1.0;
          beta1 = 0.0;
          m_t = (ptrdiff_t)1;
          n_t = (ptrdiff_t)B_size[1];
          k_t = (ptrdiff_t)3;
          lda_t = (ptrdiff_t)3;
          ldb_t = (ptrdiff_t)3;
          ldc_t = (ptrdiff_t)1;
          i1 = B_size[1];
          T_dot_size[1] = B_size[1];
          dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &M_x_gamma_sk[0],
                &lda_t, &b_gamma_s_data[0], &ldb_t, &beta1, &T_dot_data[0],
                &ldc_t);
        }
        loop_ub = i1 - 1;
        for (i1 = 0; i1 <= loop_ub; i1++) {
          T_dot_data[i1] *= b_w;
        }
        iv[0] = (*(int32_T(*)[2])y->size)[0];
        iv[1] = (*(int32_T(*)[2])y->size)[1];
        emlrtSizeEqCheckNDR2012b(&iv[0], &T_dot_size[0], &c_emlrtECI,
                                 (emlrtCTX)sp);
        i1 = r1->size[0] * r1->size[1];
        r1->size[0] = 1;
        r1->size[1] = y->size[1];
        emxEnsureCapacity_int32_T(sp, r1, i1, &ab_emlrtRTEI);
        loop_ub = y->size[1];
        for (i1 = 0; i1 < loop_ub; i1++) {
          i4 = (int32_T)y->data[i1];
          if ((i4 < 1) || (i4 > g->size[1])) {
            emlrtDynamicBoundsCheckR2012b((int32_T)y->data[i1], 1, g->size[1],
                                          &g_emlrtBCI, (emlrtCTX)sp);
          }
          r1->data[i1] = i4;
        }
        if (r1->size[1] != y->size[1]) {
          emlrtSubAssignSizeCheck1dR2017a(r1->size[1], y->size[1], &d_emlrtECI,
                                          (emlrtCTX)sp);
        }
        loop_ub = y->size[1];
        i1 = b_g->size[0];
        b_g->size[0] = y->size[1];
        emxEnsureCapacity_real_T(sp, b_g, i1, &bb_emlrtRTEI);
        for (i1 = 0; i1 < loop_ub; i1++) {
          b_g->data[i1] = g->data[(int32_T)y->data[i1] - 1] + T_dot_data[i1];
        }
        loop_ub = b_g->size[0];
        for (i1 = 0; i1 < loop_ub; i1++) {
          g->data[r1->data[i1] - 1] = b_g->data[i1];
        }
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b((emlrtCTX)sp);
        }
      }
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b((emlrtCTX)sp);
      }
    }
    emxFree_real_T(&b_g);
    emxFree_real_T(&y);
    emxFree_int32_T(&r1);
    emxFree_real_T(&r);
    i = c_pre->size[0] * c_pre->size[1];
    c_pre->size[0] = (int32_T)n;
    c_pre->size[1] = 4;
    emxEnsureCapacity_real_T(sp, c_pre, i, &u_emlrtRTEI);
    loop_ub = (int32_T)n * 4;
    for (i = 0; i < loop_ub; i++) {
      c_pre->data[i] = c_data[i];
    }
    c_pre_not_empty = ((c_pre->size[0] != 0) && (c_pre->size[1] != 0));
    i = g_pre->size[0] * g_pre->size[1];
    g_pre->size[0] = 1;
    g_pre->size[1] = g->size[1];
    emxEnsureCapacity_real_T(sp, g_pre, i, &v_emlrtRTEI);
    loop_ub = g->size[1];
    for (i = 0; i < loop_ub; i++) {
      g_pre->data[i] = g->data[i];
    }
  } else {
    i = g->size[0] * g->size[1];
    g->size[0] = 1;
    g->size[1] = g_pre->size[1];
    emxEnsureCapacity_real_T(sp, g, i, &s_emlrtRTEI);
    loop_ub = g_pre->size[1];
    for (i = 0; i < loop_ub; i++) {
      g->data[i] = g_pre->data[i];
    }
  }
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtCTX)sp);
}

void energyGradient1_free(void)
{
  emxFree_real_T(&c_pre);
  emxFree_real_T(&g_pre);
}

void energyGradient1_init(const emlrtStack *sp)
{
  emxInit_real_T(sp, &g_pre, 2, &o_emlrtRTEI, false);
  emxInit_real_T(sp, &c_pre, 2, &p_emlrtRTEI, false);
  c_pre_not_empty = false;
}

/* End of code generation (energyGradient1.c) */
