//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: lsqcurvefit.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Dec-2023 09:29:35
//

#ifndef LSQCURVEFIT_H
#define LSQCURVEFIT_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class anonymous_function;

}

// Function Declarations
namespace coder {
void lsqcurvefit_anonFcn1(const anonymous_function fun, const double ydata[2],
                          const double x[2],
                          ::coder::array<double, 2U> &varargout_1);

}

#endif
//
// File trailer for lsqcurvefit.h
//
// [EOF]
//
