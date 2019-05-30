// Copyright 2018 Geoffrey Biggs, AIST
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

#if !defined AUTONOMOUS__AUTONOMOUS_COMPONENT_HPP_
#define AUTONOMOUS__AUTONOMOUS_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
    #define AUTONOMOUS_EXPORT __attribute__ ((dllexport))
    #define AUTONOMOUS_IMPORT __attribute__ ((dllimport))
  #else
    #define AUTONOMOUS_EXPORT __declspec(dllexport)
    #define AUTONOMOUS_IMPORT __declspec(dllimport)
  #endif
  #ifdef AUTONOMOUS_BUILDING_DLL
    #define AUTONOMOUS_PUBLIC AUTONOMOUS_EXPORT
  #else
    #define AUTONOMOUS_PUBLIC AUTONOMOUS_IMPORT
  #endif
  #define AUTONOMOUS_PUBLIC_TYPE AUTONOMOUS_PUBLIC
  #define AUTONOMOUS_LOCAL
#else
#define AUTONOMOUS_EXPORT __attribute__ ((visibility("default")))
#define AUTONOMOUS_IMPORT
#if __GNUC__ >= 4
#define AUTONOMOUS_PUBLIC __attribute__ ((visibility("default")))
    #define AUTONOMOUS_LOCAL  __attribute__ ((visibility("hidden")))
#else
#define AUTONOMOUS_PUBLIC
#define AUTONOMOUS_LOCAL
#endif
#define AUTONOMOUS_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>

namespace autonomous
{

    class Autonomous : public rclcpp::Node
    {
    public:
        AUTONOMOUS_PUBLIC Autonomous();

    private:
        rclcpp::TimerBase::SharedPtr timer_;

        void broadcast_greeting();
    };

} // namespace greeting

#endif // AUTONOMOUS__AUTONOMOUS_COMPONENT_HPP_