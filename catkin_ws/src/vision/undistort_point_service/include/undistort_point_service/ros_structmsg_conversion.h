#ifndef _ROS_STRUCTMSG_CONVERSION_H_
#define _ROS_STRUCTMSG_CONVERSION_H_

#include <ros/ros.h>
#include <vision_msgs_and_srv/UndistortPoint.h>
#include <vision_msgs_and_srv/UndistortPointRequest.h>
#include <vision_msgs_and_srv/UndistortPointResponse.h>
#include "undistort_point_service_types.h"
#include "mlroscpp_msgconvert_utils.h"


void struct2msg(vision_msgs_and_srv::UndistortPointRequest* msgPtr, vision_msgs_and_srv_UndistortPointRequestStruct_T const* structPtr);
void msg2struct(vision_msgs_and_srv_UndistortPointRequestStruct_T* structPtr, vision_msgs_and_srv::UndistortPointRequest const* msgPtr);

void struct2msg(vision_msgs_and_srv::UndistortPointResponse* msgPtr, vision_msgs_and_srv_UndistortPointResponseStruct_T const* structPtr);
void msg2struct(vision_msgs_and_srv_UndistortPointResponseStruct_T* structPtr, vision_msgs_and_srv::UndistortPointResponse const* msgPtr);


#endif
