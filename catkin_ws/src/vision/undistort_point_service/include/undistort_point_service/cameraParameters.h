//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: cameraParameters.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Dec-2023 09:29:35
//

#ifndef CAMERAPARAMETERS_H
#define CAMERAPARAMETERS_H

// Include Files
#include "ImageTransformer.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
class cameraParameters {
public:
  double RadialDistortion[3];
  double TangentialDistortion[2];
  double K[9];

protected:
  vision::internal::calibration::ImageTransformer UndistortMap;
};

} // namespace coder

#endif
//
// File trailer for cameraParameters.h
//
// [EOF]
//
