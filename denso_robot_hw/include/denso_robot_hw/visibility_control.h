#ifndef DENSO_HW__VISIBILITY_CONTROL_H__
#define DENSO_HW__VISIBILITY_CONTROL_H__

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DENSO_HW_EXPORT __attribute__((dllexport))
#define DENSO_HW_IMPORT __attribute__((dllimport))
#else
#define DENSO_HW_EXPORT __declspec(dllexport)
#define DENSO_HW_IMPORT __declspec(dllimport)
#endif
#ifdef DENSO_HW_BUILDING_DLL
#define DENSO_HW_PUBLIC DENSO_HW_EXPORT
#else
#define DENSO_HW_PUBLIC DENSO_HW_IMPORT
#endif
#define DENSO_HW_PUBLIC_TYPE DENSO_HW_PUBLIC
#define DENSO_HW_LOCAL
#else
#define DENSO_HW_EXPORT __attribute__((visibility("default")))
#define DENSO_HW_IMPORT
#if __GNUC__ >= 4
#define DENSO_HW_PUBLIC __attribute__((visibility("default")))
#define DENSO_HW_LOCAL __attribute__((visibility("hidden")))
#else
#define DENSO_HW_PUBLIC
#define DENSO_HW_LOCAL
#endif
#define DENSO_HW_PUBLIC_TYPE
#endif


#endif