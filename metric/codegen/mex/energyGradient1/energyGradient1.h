/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * energyGradient1.h
 *
 * Code generation for function 'energyGradient1'
 *
 */

#pragma once

/* Include files */
#include "energyGradient1_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void energyGradient1(const emlrtStack *sp, const real_T c[12], real_T n,
                     real_T D, real_T N, const real_T T[28],
                     const real_T T_dot[28], const real_T w[7],
                     emxArray_real_T *g);

void energyGradient1_free(void);

void energyGradient1_init(const emlrtStack *sp);

/* End of code generation (energyGradient1.h) */
