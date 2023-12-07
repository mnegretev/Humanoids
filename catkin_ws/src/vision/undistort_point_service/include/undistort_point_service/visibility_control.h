#ifndef UNDISTORT_POINT_SERVICE__VISIBILITY_CONTROL_H_
#define UNDISTORT_POINT_SERVICE__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define UNDISTORT_POINT_SERVICE_EXPORT __attribute__ ((dllexport))
    #define UNDISTORT_POINT_SERVICE_IMPORT __attribute__ ((dllimport))
  #else
    #define UNDISTORT_POINT_SERVICE_EXPORT __declspec(dllexport)
    #define UNDISTORT_POINT_SERVICE_IMPORT __declspec(dllimport)
  #endif
  #ifdef UNDISTORT_POINT_SERVICE_BUILDING_LIBRARY
    #define UNDISTORT_POINT_SERVICE_PUBLIC UNDISTORT_POINT_SERVICE_EXPORT
  #else
    #define UNDISTORT_POINT_SERVICE_PUBLIC UNDISTORT_POINT_SERVICE_IMPORT
  #endif
  #define UNDISTORT_POINT_SERVICE_PUBLIC_TYPE UNDISTORT_POINT_SERVICE_PUBLIC
  #define UNDISTORT_POINT_SERVICE_LOCAL
#else
  #define UNDISTORT_POINT_SERVICE_EXPORT __attribute__ ((visibility("default")))
  #define UNDISTORT_POINT_SERVICE_IMPORT
  #if __GNUC__ >= 4
    #define UNDISTORT_POINT_SERVICE_PUBLIC __attribute__ ((visibility("default")))
    #define UNDISTORT_POINT_SERVICE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define UNDISTORT_POINT_SERVICE_PUBLIC
    #define UNDISTORT_POINT_SERVICE_LOCAL
  #endif
  #define UNDISTORT_POINT_SERVICE_PUBLIC_TYPE
#endif
#endif  // UNDISTORT_POINT_SERVICE__VISIBILITY_CONTROL_H_
// Generated 07-Dec-2023 09:29:46
// Copyright 2019-2020 The MathWorks, Inc.
