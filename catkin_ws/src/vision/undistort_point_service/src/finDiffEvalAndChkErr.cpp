//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: finDiffEvalAndChkErr.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Dec-2023 09:29:35
//

// Include Files
#include "finDiffEvalAndChkErr.h"
#include "anonymous_function.h"
#include "anonymous_function1.h"
#include "anonymous_function11.h"
#include "lsqcurvefit.h"
#include "rt_nonfinite.h"
#include "undistort_point_service_internal_types1.h"
#include "undistort_point_service_internal_types11.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const c_anonymous_function obj_nonlin
//                int obj_mEq
//                bool obj_SpecifyConstraintGradient
//                ::coder::array<double, 1U> &cEqPlus
//                int dim
//                double delta
//                double xk[2]
// Return Type  : bool
//
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace FiniteDifferences {
namespace internal {
bool finDiffEvalAndChkErr(const c_anonymous_function obj_nonlin, int obj_mEq,
                          bool obj_SpecifyConstraintGradient,
                          ::coder::array<double, 1U> &cEqPlus, int dim,
                          double delta, double xk[2])
{
  array<double, 2U> r;
  double temp_tmp;
  bool evalOK;
  evalOK = true;
  temp_tmp = xk[dim - 1];
  xk[dim - 1] = temp_tmp + delta;
  if (!obj_SpecifyConstraintGradient) {
    int idx;
    lsqcurvefit_anonFcn1(obj_nonlin.workspace.fun.workspace.fun,
                         obj_nonlin.workspace.fun.workspace.ydata, xk, r);
    idx = r.size(0) << 1;
    cEqPlus.set_size(idx);
    for (int i{0}; i < idx; i++) {
      cEqPlus[i] = r[i];
    }
    idx = 0;
    while (evalOK && (idx + 1 <= obj_mEq)) {
      evalOK = ((!std::isinf(r[idx])) && (!std::isnan(r[idx])));
      idx++;
    }
  }
  xk[dim - 1] = temp_tmp;
  return evalOK;
}

} // namespace internal
} // namespace FiniteDifferences
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for finDiffEvalAndChkErr.cpp
//
// [EOF]
//
