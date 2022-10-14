#ifndef SIAM_MAIN__VISIBILITY_CONTROL_H_
#define SIAM_MAIN__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SIAM_MAIN_EXPORT __attribute__ ((dllexport))
    #define SIAM_MAIN_IMPORT __attribute__ ((dllimport))
  #else
    #define SIAM_MAIN_EXPORT __declspec(dllexport)
    #define SIAM_MAIN_IMPORT __declspec(dllimport)
  #endif
  #ifdef SIAM_MAIN_BUILDING_LIBRARY
    #define SIAM_MAIN_PUBLIC SIAM_MAIN_EXPORT
  #else
    #define SIAM_MAIN_PUBLIC SIAM_MAIN_IMPORT
  #endif
  #define SIAM_MAIN_PUBLIC_TYPE SIAM_MAIN_PUBLIC
  #define SIAM_MAIN_LOCAL
#else
  #define SIAM_MAIN_EXPORT __attribute__ ((visibility("default")))
  #define SIAM_MAIN_IMPORT
  #if __GNUC__ >= 4
    #define SIAM_MAIN_PUBLIC __attribute__ ((visibility("default")))
    #define SIAM_MAIN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SIAM_MAIN_PUBLIC
    #define SIAM_MAIN_LOCAL
  #endif
  #define SIAM_MAIN_PUBLIC_TYPE
#endif
#endif  // SIAM_MAIN__VISIBILITY_CONTROL_H_
// Generated 14-Oct-2022 10:13:45
// Copyright 2019-2020 The MathWorks, Inc.
