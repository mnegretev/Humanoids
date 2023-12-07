//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: finDiffEvalAndChkErr.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Dec-2023 09:29:35
//

#ifndef FINDIFFEVALANDCHKERR_H
#define FINDIFFEVALANDCHKERR_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class c_anonymous_function;

}

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace FiniteDifferences {
namespace internal {
bool finDiffEvalAndChkErr(const c_anonymous_function obj_nonlin, int obj_mEq,
                          bool obj_SpecifyConstraintGradient,
                          ::coder::array<double, 1U> &cEqPlus, int dim,
                          double delta, double xk[2]);

}
} // namespace FiniteDifferences
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for finDiffEvalAndChkErr.h
//
// [EOF]
//
