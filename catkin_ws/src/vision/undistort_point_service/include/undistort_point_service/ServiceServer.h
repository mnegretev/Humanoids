//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ServiceServer.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Dec-2023 09:29:35
//

#ifndef SERVICESERVER_H
#define SERVICESERVER_H

// Include Files
#include "rtwtypes.h"
#include "undistort_point_service_types.h"
#include "mlroscpp_svcserver.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace ros {
class ServiceServer {
public:
  void callback();
  ServiceServer *init();
  char ServiceName[18];
  char RequestType[41];

private:
  vision_msgs_and_srv_UndistortPointRequestStruct_T ReqMsgStruct;
  vision_msgs_and_srv_UndistortPointResponseStruct_T RespMsgStruct;
  std::unique_ptr<
      MATLABSvcServer<vision_msgs_and_srv::UndistortPointRequest,
                      vision_msgs_and_srv::UndistortPointResponse,
                      vision_msgs_and_srv_UndistortPointRequestStruct_T,
                      vision_msgs_and_srv_UndistortPointResponseStruct_T>>
      SvcServerHelper;
  bool IsInitialized;
};

} // namespace ros
} // namespace coder

#endif
//
// File trailer for ServiceServer.h
//
// [EOF]
//
