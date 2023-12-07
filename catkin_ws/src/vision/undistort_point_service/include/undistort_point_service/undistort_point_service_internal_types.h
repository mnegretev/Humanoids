//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: undistort_point_service_internal_types.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Dec-2023 09:29:35
//

#ifndef UNDISTORT_POINT_SERVICE_INTERNAL_TYPES_H
#define UNDISTORT_POINT_SERVICE_INTERNAL_TYPES_H

// Include Files
#include "anonymous_function.h"
#include "rtwtypes.h"
#include "undistort_point_service_types.h"
#include "coder_array.h"

// Type Definitions
struct d_struct_T {
  coder::c_anonymous_function nonlin;
  double f_1;
  coder::array<double, 1U> cEq_1;
  double f_2;
  coder::array<double, 1U> cEq_2;
  int nVar;
  int mIneq;
  int mEq;
  int numEvals;
  bool SpecifyObjectiveGradient;
  bool SpecifyConstraintGradient;
  bool isEmptyNonlcon;
  bool hasLB[2];
  bool hasUB[2];
  bool hasBounds;
  int FiniteDifferenceType;
};

#endif
//
// File trailer for undistort_point_service_internal_types.h
//
// [EOF]
//
