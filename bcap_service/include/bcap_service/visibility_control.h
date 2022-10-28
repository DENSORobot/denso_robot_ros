#ifndef BCAP_SERVICE__VISIBILITY_CONTROL_H__
#define BCAP_SERVICE__VISIBILITY_CONTROL_H__

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define BCAP_SERVICE_EXPORT __attribute__((dllexport))
#define BCAP_SERVICE_IMPORT __attribute__((dllimport))
#else
#define BCAP_SERVICE_EXPORT __declspec(dllexport)
#define BCAP_SERVICE_IMPORT __declspec(dllimport)
#endif
#ifdef BCAP_SERVICE_BUILDING_DLL
#define BCAP_SERVICE_PUBLIC BCAP_SERVICE_EXPORT
#else
#define BCAP_SERVICE_PUBLIC BCAP_SERVICE_IMPORT
#endif
#define BCAP_SERVICE_PUBLIC_TYPE BCAP_SERVICE_PUBLIC
#define BCAP_SERVICE_LOCAL
#else
#define BCAP_SERVICE_EXPORT __attribute__((visibility("default")))
#define BCAP_SERVICE_IMPORT
#if __GNUC__ >= 4
#define BCAP_SERVICE_PUBLIC __attribute__((visibility("default")))
#define BCAP_SERVICE_LOCAL __attribute__((visibility("hidden")))
#else
#define BCAP_SERVICE_PUBLIC
#define BCAP_SERVICE_LOCAL
#endif
#define BCAP_SERVICE_PUBLIC_TYPE
#endif


#endif