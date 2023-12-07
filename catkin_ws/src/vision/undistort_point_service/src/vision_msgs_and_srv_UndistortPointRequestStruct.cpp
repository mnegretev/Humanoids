//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: vision_msgs_and_srv_UndistortPointRequestStruct.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Dec-2023 09:29:35
//

// Include Files
#include "vision_msgs_and_srv_UndistortPointRequestStruct.h"
#include "rt_nonfinite.h"
#include "undistort_point_service_types.h"

// Function Definitions
//
// Message struct definition for vision_msgs_and_srv/UndistortPointRequest
//
// Arguments    : void
// Return Type  : vision_msgs_and_srv_UndistortPointRequestStruct_T
//
vision_msgs_and_srv_UndistortPointRequestStruct_T
vision_msgs_and_srv_UndistortPointRequestStruct()
{
  static const vision_msgs_and_srv_UndistortPointRequestStruct_T b_msg{
      {'v', 'i', 's', 'i', 'o', 'n', '_', 'm', 's', 'g', 's', '_', 'a', 'n',
       'd', '_', 's', 'r', 'v', '/', 'U', 'n', 'd', 'i', 's', 't', 'o', 'r',
       't', 'P', 'o', 'i', 'n', 't', 'R', 'e', 'q', 'u', 'e', 's', 't'}, // MessageType
      0.0, // XDist
      0.0  // YDist
  };
  vision_msgs_and_srv_UndistortPointRequestStruct_T msg;
  msg = b_msg;
  //(&b_msg);
  return msg;
}

//
// File trailer for vision_msgs_and_srv_UndistortPointRequestStruct.cpp
//
// [EOF]
//
