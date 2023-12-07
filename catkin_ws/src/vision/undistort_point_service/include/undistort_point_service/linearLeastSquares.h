//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: linearLeastSquares.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Dec-2023 09:29:35
//

#ifndef LINEARLEASTSQUARES_H
#define LINEARLEASTSQUARES_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace levenbergMarquardt {
void linearLeastSquares(::coder::array<double, 2U> &lhs,
                        ::coder::array<double, 1U> &rhs, double dx[2], int m);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for linearLeastSquares.h
//
// [EOF]
//
