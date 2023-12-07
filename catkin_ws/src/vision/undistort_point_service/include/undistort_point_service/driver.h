//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: driver.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Dec-2023 09:29:35
//

#ifndef DRIVER_H
#define DRIVER_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class b_anonymous_function;

}

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace levenbergMarquardt {
double driver(b_anonymous_function fun, double x0[2],
              ::coder::array<double, 2U> &fCurrent, char output_algorithm[19],
              double lambda_lower[2], double lambda_upper[2],
              ::coder::array<double, 2U> &jacobian, double &exitflag,
              double &output_iterations, double &output_funcCount,
              double &output_stepsize, double &output_firstorderopt);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for driver.h
//
// [EOF]
//
