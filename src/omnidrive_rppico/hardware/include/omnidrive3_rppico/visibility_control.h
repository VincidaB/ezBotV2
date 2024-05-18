// Copyright 2021 ros2_control Development Team
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

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef omnidrive3_RPPICO__VISIBILITY_CONTROL_H_
#define omnidrive3_RPPICO__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define omnidrive3_RPPICO_EXPORT __attribute__((dllexport))
#define omnidrive3_RPPICO_IMPORT __attribute__((dllimport))
#else
#define omnidrive3_RPPICO_EXPORT __declspec(dllexport)
#define omnidrive3_RPPICO_IMPORT __declspec(dllimport)
#endif
#ifdef omnidrive3_RPPICO_BUILDING_DLL
#define omnidrive3_RPPICO_PUBLIC omnidrive3_RPPICO_EXPORT
#else
#define omnidrive3_RPPICO_PUBLIC omnidrive3_RPPICO_IMPORT
#endif
#define omnidrive3_RPPICO_PUBLIC_TYPE omnidrive3_RPPICO_PUBLIC
#define omnidrive3_RPPICO_LOCAL
#else
#define omnidrive3_RPPICO_EXPORT __attribute__((visibility("default")))
#define omnidrive3_RPPICO_IMPORT
#if __GNUC__ >= 4
#define omnidrive3_RPPICO_PUBLIC __attribute__((visibility("default")))
#define omnidrive3_RPPICO_LOCAL __attribute__((visibility("hidden")))
#else
#define omnidrive3_RPPICO_PUBLIC
#define omnidrive3_RPPICO_LOCAL
#endif
#define omnidrive3_RPPICO_PUBLIC_TYPE
#endif

#endif  // omnidrive3_RPPICO__VISIBILITY_CONTROL_H_
