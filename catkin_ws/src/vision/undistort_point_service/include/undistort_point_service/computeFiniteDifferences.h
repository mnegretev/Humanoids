//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeFiniteDifferences.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Dec-2023 09:29:35
//

#ifndef COMPUTEFINITEDIFFERENCES_H
#define COMPUTEFINITEDIFFERENCES_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct d_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace FiniteDifferences {
bool computeFiniteDifferences(d_struct_T &obj,
                              const ::coder::array<double, 2U> &cEqCurrent,
                              double xk[2],
                              ::coder::array<double, 2U> &JacCeqTrans);

}
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for computeFiniteDifferences.h
//
// [EOF]
//
