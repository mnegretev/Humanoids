//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: lsqcurvefit.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Dec-2023 09:29:35
//

// Include Files
#include "lsqcurvefit.h"
#include "anonymous_function11.h"
#include "cameraParameters.h"
#include "rt_nonfinite.h"
#include "undistort_point_service_internal_types111.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : const anonymous_function fun
//                const double ydata[2]
//                const double x[2]
//                ::coder::array<double, 2U> &varargout_1
// Return Type  : void
//
namespace coder {
void lsqcurvefit_anonFcn1(const anonymous_function fun, const double ydata[2],
                          const double x[2],
                          ::coder::array<double, 2U> &varargout_1)
{
  double b_r2_tmp;
  double center_idx_0;
  double center_idx_1;
  double k_idx_1;
  double r2;
  double r2_tmp;
  double xNorm;
  double yNorm;
  center_idx_0 = fun.workspace.b_this->K[6];
  center_idx_1 = fun.workspace.b_this->K[7];
  yNorm = (x[1] - center_idx_1) / fun.workspace.b_this->K[4];
  xNorm = ((x[0] - center_idx_0) - fun.workspace.b_this->K[3] * yNorm) /
          fun.workspace.b_this->K[0];
  r2_tmp = xNorm * xNorm;
  b_r2_tmp = yNorm * yNorm;
  r2 = r2_tmp + b_r2_tmp;
  center_idx_1 = r2 * r2;
  center_idx_0 = fun.workspace.b_this->RadialDistortion[0];
  k_idx_1 = fun.workspace.b_this->RadialDistortion[1];
  k_idx_1 = (center_idx_0 * r2 + k_idx_1 * center_idx_1) +
            fun.workspace.b_this->RadialDistortion[2] * (r2 * center_idx_1);
  center_idx_1 = xNorm * yNorm;
  center_idx_0 =
      2.0 * fun.workspace.b_this->TangentialDistortion[0] * center_idx_1 +
      fun.workspace.b_this->TangentialDistortion[1] * (r2 + 2.0 * r2_tmp);
  center_idx_1 =
      fun.workspace.b_this->TangentialDistortion[0] * (r2 + 2.0 * b_r2_tmp) +
      2.0 * fun.workspace.b_this->TangentialDistortion[1] * center_idx_1;
  center_idx_1 += yNorm + yNorm * k_idx_1;
  center_idx_0 =
      (((xNorm + xNorm * k_idx_1) + center_idx_0) * fun.workspace.b_this->K[0] +
       fun.workspace.b_this->K[6]) +
      center_idx_1 * fun.workspace.b_this->K[3];
  center_idx_1 =
      center_idx_1 * fun.workspace.b_this->K[4] + fun.workspace.b_this->K[7];
  varargout_1.set_size(1, 2);
  varargout_1[0] = center_idx_0 - ydata[0];
  varargout_1[varargout_1.size(0)] = center_idx_1 - ydata[1];
}

} // namespace coder

//
// File trailer for lsqcurvefit.cpp
//
// [EOF]
//
