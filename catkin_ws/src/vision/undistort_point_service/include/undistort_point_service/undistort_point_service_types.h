//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: undistort_point_service_types.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Dec-2023 09:29:35
//

#ifndef UNDISTORT_POINT_SERVICE_TYPES_H
#define UNDISTORT_POINT_SERVICE_TYPES_H

// Include Files
#include "rtwtypes.h"

// Type Definitions
struct vision_msgs_and_srv_UndistortPointRequestStruct_T {
  char MessageType[41];
  double XDist;
  double YDist;
};

struct vision_msgs_and_srv_UndistortPointResponseStruct_T {
  char MessageType[42];
  double XUndis;
  double YUndis;
};

#endif
//
// File trailer for undistort_point_service_types.h
//
// [EOF]
//
