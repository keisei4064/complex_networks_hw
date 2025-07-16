#ifndef CONTROLKO_CONTROLLERS__VISIBILITY_CONTROL_H_
#define CONTROLKO_CONTROLLERS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CONTROLKO_CONTROLLERS_EXPORT __attribute__ ((dllexport))
    #define CONTROLKO_CONTROLLERS_IMPORT __attribute__ ((dllimport))
  #else
    #define CONTROLKO_CONTROLLERS_EXPORT __declspec(dllexport)
    #define CONTROLKO_CONTROLLERS_IMPORT __declspec(dllimport)
  #endif
  #ifdef CONTROLKO_CONTROLLERS_BUILDING_LIBRARY
    #define CONTROLKO_CONTROLLERS_PUBLIC CONTROLKO_CONTROLLERS_EXPORT
  #else
    #define CONTROLKO_CONTROLLERS_PUBLIC CONTROLKO_CONTROLLERS_IMPORT
  #endif
  #define CONTROLKO_CONTROLLERS_PUBLIC_TYPE CONTROLKO_CONTROLLERS_PUBLIC
  #define CONTROLKO_CONTROLLERS_LOCAL
#else
  #define CONTROLKO_CONTROLLERS_EXPORT __attribute__ ((visibility("default")))
  #define CONTROLKO_CONTROLLERS_IMPORT
  #if __GNUC__ >= 4
    #define CONTROLKO_CONTROLLERS_PUBLIC __attribute__ ((visibility("default")))
    #define CONTROLKO_CONTROLLERS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CONTROLKO_CONTROLLERS_PUBLIC
    #define CONTROLKO_CONTROLLERS_LOCAL
  #endif
  #define CONTROLKO_CONTROLLERS_PUBLIC_TYPE
#endif

#endif  // CONTROLKO_CONTROLLERS__VISIBILITY_CONTROL_H_
