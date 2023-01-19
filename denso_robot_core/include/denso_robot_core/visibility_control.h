#ifndef DENSO_ROBOT_CORE__VISIBILITY_CONTROL_H__
#define DENSO_ROBOT_CORE__VISIBILITY_CONTROL_H__

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DENSO_ROBOT_CORE_EXPORT __attribute__((dllexport))
#define DENSO_ROBOT_CORE_IMPORT __attribute__((dllimport))
#else
#define DENSO_ROBOT_CORE_EXPORT __declspec(dllexport)
#define DENSO_ROBOT_CORE_IMPORT __declspec(dllimport)
#endif
#ifdef DENSO_ROBOT_CORE_BUILDING_DLL
#define DENSO_ROBOT_CORE_PUBLIC DENSO_ROBOT_CORE_EXPORT
#else
#define DENSO_ROBOT_CORE_PUBLIC DENSO_ROBOT_CORE_IMPORT
#endif
#define DENSO_ROBOT_CORE_PUBLIC_TYPE DENSO_ROBOT_CORE_PUBLIC
#define DENSO_ROBOT_CORE_LOCAL
#else
#define DENSO_ROBOT_CORE_EXPORT __attribute__((visibility("default")))
#define DENSO_ROBOT_CORE_IMPORT
#if __GNUC__ >= 4
#define DENSO_ROBOT_CORE_PUBLIC __attribute__((visibility("default")))
#define DENSO_ROBOT_CORE_LOCAL __attribute__((visibility("hidden")))
#else
#define DENSO_ROBOT_CORE_PUBLIC
#define DENSO_ROBOT_CORE_LOCAL
#endif
#define DENSO_ROBOT_CORE_PUBLIC_TYPE
#endif


#endif