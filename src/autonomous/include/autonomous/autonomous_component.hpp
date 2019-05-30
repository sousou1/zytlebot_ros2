#if !defined AUTONOMOUS__AUTONOMOUS_NODE_HPP_
#define AUTONOMOUS__AUTONOMOUS_NODE_HPP_


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
#include <std_msgs/msg/string.hpp>

#include <fstream>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include "unistd.h"
#include <math.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/twist.h>
#include <string>
#include <cstdlib>
#include <typeinfo>

#include <boost/thread.hpp>

// pcam使用時
#include "std_msgs/msg/multi_array_layout.h"
#include "std_msgs/msg/multi_array_dimension.h"
#include "std_msgs/msg/u_int8_multi_array.h"
#include "std_msgs/msg/string.h"

// devmem
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/mman.h>

// JSON読み込み
#include <iostream>
#include <fstream>
#include <sstream>
#include <json_lib/json11.hpp>


namespace autonomous
{

    class AUTONOMOUS : public rclcpp::Node
    {
    public:
        AUTONOMOUS_PUBLIC AUTONOMOUS();

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        void autonomous();
    };

} // namespace autonomous_ros2_style

#endif // AUTONOMOUS__AUTONOMOUS_NODE_HPP_