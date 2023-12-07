//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: checkStoppingCriteria.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Dec-2023 09:29:35
//

#ifndef CHECKSTOPPINGCRITERIA_H
#define CHECKSTOPPINGCRITERIA_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace levenbergMarquardt {
int b_checkStoppingCriteria(const double gradf[2], double relFactor,
                            double funDiff, const double x[2],
                            const double dx[2], int funcCount,
                            bool stepSuccessful, int &iter);

int checkStoppingCriteria(const double gradf[2], double relFactor,
                          int funcCount);

} // namespace levenbergMarquardt
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for checkStoppingCriteria.h
//
// [EOF]
//
