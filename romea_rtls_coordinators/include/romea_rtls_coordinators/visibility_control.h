// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROMEA_RTLS_COORDINATORS__VISIBILITY_CONTROL_H_
#define ROMEA_RTLS_COORDINATORS__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROMEA_RTLS_COORDINATORS_EXPORT __attribute__ ((dllexport))
    #define ROMEA_RTLS_COORDINATORS_IMPORT __attribute__ ((dllimport))
  #else
    #define ROMEA_RTLS_COORDINATORS_EXPORT __declspec(dllexport)
    #define ROMEA_RTLS_COORDINATORS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROMEA_RTLS_COORDINATORS_BUILDING_DLL
    #define ROMEA_RTLS_COORDINATORS_PUBLIC ROMEA_RTLS_COORDINATORS_EXPORT
  #else
    #define ROMEA_RTLS_COORDINATORS_PUBLIC ROMEA_RTLS_COORDINATORS_IMPORT
  #endif
  #define ROMEA_RTLS_COORDINATORS_PUBLIC_TYPE ROMEA_RTLS_COORDINATORS_PUBLIC
  #define ROMEA_RTLS_COORDINATORS_LOCAL
#else
  #define ROMEA_RTLS_COORDINATORS_EXPORT __attribute__ ((visibility("default")))
  #define ROMEA_RTLS_COORDINATORS_IMPORT
  #if __GNUC__ >= 4
    #define ROMEA_RTLS_COORDINATORS_PUBLIC __attribute__ ((visibility("default")))
    #define ROMEA_RTLS_COORDINATORS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROMEA_RTLS_COORDINATORS_PUBLIC
    #define ROMEA_RTLS_COORDINATORS_LOCAL
  #endif
  #define ROMEA_RTLS_COORDINATORS_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ROMEA_RTLS_COORDINATORS__VISIBILITY_CONTROL_H_