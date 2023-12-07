//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeFiniteDifferences.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Dec-2023 09:29:35
//

// Include Files
#include "computeFiniteDifferences.h"
#include "anonymous_function.h"
#include "finDiffEvalAndChkErr.h"
#include "rt_nonfinite.h"
#include "undistort_point_service_internal_types.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : d_struct_T &obj
//                const ::coder::array<double, 2U> &cEqCurrent
//                double xk[2]
//                ::coder::array<double, 2U> &JacCeqTrans
// Return Type  : bool
//
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace FiniteDifferences {
bool computeFiniteDifferences(d_struct_T &obj,
                              const ::coder::array<double, 2U> &cEqCurrent,
                              double xk[2],
                              ::coder::array<double, 2U> &JacCeqTrans)
{
  array<double, 1U> r;
  d_struct_T b_obj;
  int formulaType;
  bool evalOK;
  if (obj.isEmptyNonlcon) {
    evalOK = true;
  } else if (obj.FiniteDifferenceType == 0) {
    int idx;
    bool exitg1;
    evalOK = true;
    obj.numEvals = 0;
    idx = 0;
    exitg1 = false;
    while ((!exitg1) && (idx <= obj.nVar - 1)) {
      double delta1;
      double deltaX;
      int idx_col;
      int loop_ub;
      bool guard1{false};
      deltaX = 1.4901161193847656E-8 *
               (1.0 - 2.0 * static_cast<double>(xk[idx] < 0.0)) *
               std::fmax(std::abs(xk[idx]), 1.0);
      delta1 = obj.f_1;
      r.set_size(obj.cEq_1.size(0));
      loop_ub = obj.cEq_1.size(0);
      for (idx_col = 0; idx_col < loop_ub; idx_col++) {
        r[idx_col] = obj.cEq_1[idx_col];
      }
      evalOK = internal::finDiffEvalAndChkErr(obj.nonlin, obj.mEq,
                                              obj.SpecifyConstraintGradient, r,
                                              idx + 1, deltaX, xk);
      obj.f_1 = delta1;
      obj.cEq_1.set_size(r.size(0));
      loop_ub = r.size(0);
      for (idx_col = 0; idx_col < loop_ub; idx_col++) {
        obj.cEq_1[idx_col] = r[idx_col];
      }
      obj.numEvals++;
      guard1 = false;
      if (!evalOK) {
        deltaX = -deltaX;
        if (!obj.hasBounds) {
          evalOK = internal::finDiffEvalAndChkErr(obj.nonlin, obj.mEq,
                                                  obj.SpecifyConstraintGradient,
                                                  r, idx + 1, deltaX, xk);
          obj.f_1 = delta1;
          obj.cEq_1.set_size(r.size(0));
          loop_ub = r.size(0);
          for (idx_col = 0; idx_col < loop_ub; idx_col++) {
            obj.cEq_1[idx_col] = r[idx_col];
          }
          obj.numEvals++;
        }
        if (!evalOK) {
          exitg1 = true;
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
      if (guard1) {
        idx_col = obj.mEq;
        for (loop_ub = 0; loop_ub < idx_col; loop_ub++) {
          JacCeqTrans[idx + (loop_ub << 1)] =
              (obj.cEq_1[loop_ub] - cEqCurrent[loop_ub]) / deltaX;
        }
        idx++;
      }
    }
  } else {
    int idx;
    bool exitg1;
    b_obj = obj;
    evalOK = true;
    b_obj.numEvals = 0;
    idx = 0;
    exitg1 = false;
    while ((!exitg1) && (idx <= b_obj.nVar - 1)) {
      double delta1;
      double delta2;
      double deltaX;
      int loop_ub;
      deltaX = 1.4901161193847656E-8 * std::fmax(std::abs(xk[idx]), 1.0);
      if ((!b_obj.hasLB[idx]) && (!b_obj.hasUB[idx])) {
        formulaType = 0;
      }
      delta1 = -deltaX;
      delta2 = deltaX;
      int exitg2;
      do {
        exitg2 = 0;
        r.set_size(b_obj.cEq_1.size(0));
        loop_ub = b_obj.cEq_1.size(0);
        for (int idx_col{0}; idx_col < loop_ub; idx_col++) {
          r[idx_col] = b_obj.cEq_1[idx_col];
        }
        evalOK = internal::finDiffEvalAndChkErr(b_obj.nonlin, b_obj.mEq,
                                                b_obj.SpecifyConstraintGradient,
                                                r, idx + 1, delta1, xk);
        b_obj.cEq_1.set_size(r.size(0));
        loop_ub = r.size(0);
        for (int idx_col{0}; idx_col < loop_ub; idx_col++) {
          b_obj.cEq_1[idx_col] = r[idx_col];
        }
        b_obj.numEvals++;
        if (!evalOK) {
          if ((formulaType == 0) && (!b_obj.hasBounds)) {
            formulaType = 1;
            delta1 = deltaX;
            delta2 = 2.0 * deltaX;
          } else {
            exitg2 = 1;
          }
        } else {
          r.set_size(b_obj.cEq_2.size(0));
          loop_ub = b_obj.cEq_2.size(0);
          for (int idx_col{0}; idx_col < loop_ub; idx_col++) {
            r[idx_col] = b_obj.cEq_2[idx_col];
          }
          evalOK = internal::finDiffEvalAndChkErr(
              b_obj.nonlin, b_obj.mEq, b_obj.SpecifyConstraintGradient, r,
              idx + 1, delta2, xk);
          b_obj.cEq_2.set_size(r.size(0));
          loop_ub = r.size(0);
          for (int idx_col{0}; idx_col < loop_ub; idx_col++) {
            b_obj.cEq_2[idx_col] = r[idx_col];
          }
          b_obj.numEvals++;
          if ((!evalOK) && (formulaType == 0) && (!b_obj.hasBounds)) {
            formulaType = -1;
            delta1 = -2.0 * deltaX;
            delta2 = -deltaX;
          } else {
            exitg2 = 1;
          }
        }
      } while (exitg2 == 0);
      if (!evalOK) {
        exitg1 = true;
      } else {
        if ((!b_obj.SpecifyConstraintGradient) && (b_obj.mEq > 0)) {
          loop_ub = b_obj.mEq - 1;
          switch (formulaType) {
          case 0:
            for (int idx_col{0}; idx_col <= loop_ub; idx_col++) {
              JacCeqTrans[idx + (idx_col << 1)] =
                  (-b_obj.cEq_1[idx_col] + b_obj.cEq_2[idx_col]) /
                  (2.0 * deltaX);
            }
            break;
          case 1:
            for (int idx_col{0}; idx_col <= loop_ub; idx_col++) {
              JacCeqTrans[idx + (idx_col << 1)] =
                  ((-3.0 * cEqCurrent[idx_col] + 4.0 * b_obj.cEq_1[idx_col]) -
                   b_obj.cEq_2[idx_col]) /
                  (2.0 * deltaX);
            }
            break;
          default:
            for (int idx_col{0}; idx_col <= loop_ub; idx_col++) {
              JacCeqTrans[idx + (idx_col << 1)] =
                  ((b_obj.cEq_1[idx_col] - 4.0 * b_obj.cEq_2[idx_col]) +
                   3.0 * cEqCurrent[idx_col]) /
                  (2.0 * deltaX);
            }
            break;
          }
        }
        idx++;
      }
    }
    obj = b_obj;
  }
  return evalOK;
}

} // namespace FiniteDifferences
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for computeFiniteDifferences.cpp
//
// [EOF]
//
