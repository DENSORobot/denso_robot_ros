#ifndef BCAP_SRV__VISIBILITY_CONTROL_H__
#define BCAP_SRV__VISIBILITY_CONTROL_H__

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define BCAP_SRV_EXPORT __attribute__((dllexport))
#define BCAP_SRV_IMPORT __attribute__((dllimport))
#else
#define BCAP_SRV_EXPORT __declspec(dllexport)
#define BCAP_SRV_IMPORT __declspec(dllimport)
#endif
#ifdef BCAP_SRV_BUILDING_DLL
#define BCAP_SRV_PUBLIC BCAP_SRV_EXPORT
#else
#define BCAP_SRV_PUBLIC BCAP_SRV_IMPORT
#endif
#define BCAP_SRV_PUBLIC_TYPE BCAP_SRV_PUBLIC
#define BCAP_SRV_LOCAL
#else
#define BCAP_SRV_EXPORT __attribute__((visibility("default")))
#define BCAP_SRV_IMPORT
#if __GNUC__ >= 4
#define BCAP_SRV_PUBLIC __attribute__((visibility("default")))
#define BCAP_SRV_LOCAL __attribute__((visibility("hidden")))
#else
#define BCAP_SRV_PUBLIC
#define BCAP_SRV_LOCAL
#endif
#define BCAP_SRV_PUBLIC_TYPE
#endif


#endif