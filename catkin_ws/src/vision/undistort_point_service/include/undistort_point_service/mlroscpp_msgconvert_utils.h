
//
// File mlroscpp_msgconvert_utils.h
//
// Code generated for MATLAB function 'undistort_point_service'.
//
// MATLAB Coder version         : 5.6 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Thu Dec 07 09:29:44 2023
#ifndef _MLROSCPP_MSGCONVERT_UTILS_H_
#define _MLROSCPP_MSGCONVERT_UTILS_H_
#include "undistort_point_service_types.h"
#include <algorithm>
/**
 * Convert data in a ROS C++ array message property to an array of struct property.
 *
 * This is a template specialization that requires the "structPtr" input to
 * be passed as a reference to an array. If "structPtr" points to a scalar,
 * see the other template specialization.
 *
 * @param[in] msgProp Array property in roscpp message
 * @param[in,out] structPtr Array property in Simulink bus
 * @param[in] idx Index in array to convert
 */
template <typename type, typename MsgType, size_t N>
inline void convertToStructInNested(const MsgType& msgProp, type (&structPtr)[N], int idx) {
        msg2struct(&structPtr[idx],&msgProp[idx]);
}
/**
 * Convert data in a ROS C++ array message property to a scalar struct property.
 *
 * This is a template specialization that requires the "structPtr" input to
 * be passed as a reference to a scalar value. If "structPtr" points to an array,
 * see the other template specialization.
 *
 * @param[in] msgProp Array property in roscpp message
 * @param[in,out] structPtr Scalar property in Simulink bus
 * @param[in] idx Index in array to convert
 */
template <typename type, typename MsgType>
inline void convertToStructInNested(const MsgType& msgProp, type& structPtr, int idx) {
        msg2struct(&structPtr,&msgProp[idx]);
}
/**
 * Convert data in an array of struct property to a ROS C++ array message property
 *
 * This is a template specialization that requires the "structPtr" input to
 * be passed as a reference to an array. If "structPtr" points to a scalar,
 * see the other template specialization.
 *
 * @param[in] structPtr Array property in Simulink bus
 * @param[in,out] msgProp Array property in roscpp message
 * @param[in] idx Index in array to convert
 */
template <typename type, typename MsgType, size_t N>
inline void convertFromStructInNested(const type(&structPtr)[N], MsgType& msgProp, int idx) {
        struct2msg(&msgProp[idx],&structPtr[idx]);
}
/**
 * Convert data in a scalar struct property to an ROS C++ array message property
 *
 * This is a template specialization that requires the "structPtr" input to
 * be passed as a reference to a scalar value. If "structPtr" points to an array,
 * see the other template specialization.
 *
 * @param[in] structPtr Scalar property in Simulink bus
 * @param[in,out] msgProp Array property in roscpp message
 * @param[in] idx Index in array to convert
 */
template <typename type, typename MsgType>
inline void convertFromStructInNested(const type& structPtr, MsgType& msgProp, int idx) {
        struct2msg(&msgProp[idx],&structPtr);
}
/**
 * Copy data from an scalar in ROS C++ message property to a primitive scalar 
 * For e.g., std::array<double, 1> msgPtr --> double structPtr [1]
 *
 * @param[in] msgPtr Array property in ROS C++ message
 * @param[in,out] structPtr primitive array in MATLAB passed by reference
 */
template <typename MsgType, typename type>
void convertToStructPrimitiveArray(type &structPtr, const MsgType& msgPtr) {
    std::copy(msgPtr.begin(), msgPtr.end(), &structPtr);
}
/**
 * Copy data from an array in ROS C++ message property to a primitive array
 * For e.g., mwboost::array<double, 100> msgPtr --> double structPtr [100]
 *
 * @param[in] msgPtr Array property in ROS C++ message
 * @param[in,out] structPtr primitive array in MATLAB passed by reference
 */
template <typename MsgType, typename type, int N>
void convertToStructPrimitiveArray(type (&structPtr)[N], const MsgType& msgPtr) {
    std::copy(msgPtr.begin(), msgPtr.end(), std::begin(structPtr));
}
/**
 * Copy data from a primitive scalar in MATLAB to a scalar in ROS C++ message 
 * property. For e.g., double structPtr[1] --> std::array<double, 1> msgPtr
 *
 * @param[in] structPtr primitive array in MATLAB passed by reference
 * @param[in,out] msgPtr Array property in ROS C++ message
 */
template <typename MsgType, typename type>
void convertFromStructPrimitiveArray(MsgType& msgPtr, const type &structPtr) {
    size_t numItems = msgPtr.size();
    std::copy(&structPtr, &structPtr + numItems, msgPtr.begin());
}
/**
 * Copy data from a primitive array in MATLAB to an array in ROS C++ message 
 * property. For e.g., double structPtr[100] --> std::array/vector<double, 100> msgPtr
 *
 * @param[in] structPtr primitive array in MATLAB passed by reference
 * @param[in,out] msgPtr Array property in ROS C++ message
 */
template <typename MsgType, typename type, int N>
void convertFromStructPrimitiveArray(MsgType& msgPtr, const type (&structPtr)[N]) {
    std::copy(std::begin(structPtr), std::end(structPtr), msgPtr.begin());
}
/**
 * Copy data from a ROS C++ scalar nested message property to
 * to scalar nested MATLAB struct message type
 *
 * @param[in] msgPtr array property in ROS C++ message (of type std::vector)
 * @param[in,out] structPtr coder::array property in MATLAB Structure 
 */
template <typename MsgType, typename type>
void convertToStructNestedArray(type &structPtr, const MsgType& msgPtr) {
    std::size_t  len = msgPtr.size();
    for (std::size_t k = 0; k < len; k++) {
        convertToStructInNested(msgPtr,structPtr,k);
    }
}
/**
 * 
 * Copy data from a MATLAB struct array property to an array
 * ROS C++ message property 
 *
 * @param[in] structPtr coder::array property in MATLAB Structure 
 * @param[in,out] msgPtr array property in ROS C++ message (of type std::vector)
 */
template <typename MsgType, typename type>
void convertFromStructNestedArray(MsgType& msgPtr, const type &structPtr) {
    std::size_t len = msgPtr.size();
    for (std::size_t k = 0; k < len; k++) {
        convertFromStructInNested(structPtr,msgPtr,k);
    }    
}
#endif
