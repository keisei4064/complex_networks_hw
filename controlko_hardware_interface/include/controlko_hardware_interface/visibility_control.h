// ros2 pkg createで，--library_name を指定すると自動で生成される
// ライブラリを正しくビルド/公開するための「お作法的なヘッダ」

#ifndef CONTROLKO_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define CONTROLKO_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CONTROLKO_HARDWARE_INTERFACE__EXPORT __attribute__ ((dllexport))
    #define CONTROLKO_HARDWARE_INTERFACE__IMPORT __attribute__ ((dllimport))
  #else
    #define CONTROLKO_HARDWARE_INTERFACE__EXPORT __declspec(dllexport)
    #define CONTROLKO_HARDWARE_INTERFACE__IMPORT __declspec(dllimport)
  #endif
  #ifdef CONTROLKO_HARDWARE_INTERFACE__BUILDING_LIBRARY
    #define CONTROLKO_HARDWARE_INTERFACE__PUBLIC CONTROLKO_HARDWARE_INTERFACE__EXPORT
  #else
    #define CONTROLKO_HARDWARE_INTERFACE__PUBLIC CONTROLKO_HARDWARE_INTERFACE__IMPORT
  #endif
  #define CONTROLKO_HARDWARE_INTERFACE__PUBLIC_TYPE CONTROLKO_HARDWARE_INTERFACE__PUBLIC
  #define CONTROLKO_HARDWARE_INTERFACE__LOCAL
#else
  #define CONTROLKO_HARDWARE_INTERFACE__EXPORT __attribute__ ((visibility("default")))
  #define CONTROLKO_HARDWARE_INTERFACE__IMPORT
  #if __GNUC__ >= 4
    #define CONTROLKO_HARDWARE_INTERFACE__PUBLIC __attribute__ ((visibility("default")))
    #define CONTROLKO_HARDWARE_INTERFACE__LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CONTROLKO_HARDWARE_INTERFACE__PUBLIC
    #define CONTROLKO_HARDWARE_INTERFACE__LOCAL
  #endif
  #define CONTROLKO_HARDWARE_INTERFACE__PUBLIC_TYPE
#endif

#endif  // CONTROLKO_HARDWARE_INTERFACE___VISIBILITY_CONTROL_H_
