#include "ros_structmsg_conversion.h"


// Conversions between vision_msgs_and_srv_UndistortPointRequestStruct_T and vision_msgs_and_srv::UndistortPointRequest

void struct2msg(vision_msgs_and_srv::UndistortPointRequest* msgPtr, vision_msgs_and_srv_UndistortPointRequestStruct_T const* structPtr)
{
  const std::string rosMessageType("vision_msgs_and_srv/UndistortPointRequest");

  msgPtr->x_dist =  structPtr->XDist;
  msgPtr->y_dist =  structPtr->YDist;
}

void msg2struct(vision_msgs_and_srv_UndistortPointRequestStruct_T* structPtr, vision_msgs_and_srv::UndistortPointRequest const* msgPtr)
{
  const std::string rosMessageType("vision_msgs_and_srv/UndistortPointRequest");

  structPtr->XDist =  msgPtr->x_dist;
  structPtr->YDist =  msgPtr->y_dist;
}


// Conversions between vision_msgs_and_srv_UndistortPointResponseStruct_T and vision_msgs_and_srv::UndistortPointResponse

void struct2msg(vision_msgs_and_srv::UndistortPointResponse* msgPtr, vision_msgs_and_srv_UndistortPointResponseStruct_T const* structPtr)
{
  const std::string rosMessageType("vision_msgs_and_srv/UndistortPointResponse");

  msgPtr->x_undis =  structPtr->XUndis;
  msgPtr->y_undis =  structPtr->YUndis;
}

void msg2struct(vision_msgs_and_srv_UndistortPointResponseStruct_T* structPtr, vision_msgs_and_srv::UndistortPointResponse const* msgPtr)
{
  const std::string rosMessageType("vision_msgs_and_srv/UndistortPointResponse");

  structPtr->XUndis =  msgPtr->x_undis;
  structPtr->YUndis =  msgPtr->y_undis;
}

