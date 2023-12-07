//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: undistort_point_service.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Dec-2023 09:29:35
//

// Include Files
#include "undistort_point_service.h"
#include "ServiceServer.h"
#include "rt_nonfinite.h"
#include "undistort_point_service_data.h"
#include "undistort_point_service_initialize.h"
#include "coder_posix_time.h"

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void undistort_point_service()
{
  coder::ros::ServiceServer testserver;
  coderTimespec b_timespec;
  if (!isInitialized_undistort_point_service) {
    undistort_point_service_initialize();
  }
  //  Instructions:
  //  1) run cd /home/andreslopez/Andres/Humanoids/catkin_ws/src/vision/
  //  2) run rosgenmsg
  //  3) run
  //  addpath('/home/andreslopez/Andres/Humanoids/catkin_ws/src/vision/matlab_msg_gen_ros1/glnxa64/install/m')
  //  4) run clear classes
  //  5) run rehash toolboxcache
  //  6) run rosmsg list
  //  Important: The step 4 needs the ros master to be running
  testserver.init();
  while (1) {
    if (pauseState == 0) {
      b_timespec.tv_sec = 0.0;
      b_timespec.tv_nsec = 1.0E+8;
      coderTimeSleep(&b_timespec);
    }
  }
}

//
// File trailer for undistort_point_service.cpp
//
// [EOF]
//
