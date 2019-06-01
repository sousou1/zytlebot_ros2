#if !defined PCAM__PCAM_COMPONENT_HPP_
#define PCAM__PCAM_COMPONENT_HPP_


#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
    #define PCAM_EXPORT __attribute__ ((dllexport))
    #define PCAM_IMPORT __attribute__ ((dllimport))
  #else
    #define PCAM_EXPORT __declspec(dllexport)
    #define PCAM_IMPORT __declspec(dllimport)
  #endif
  #ifdef PCAM_BUILDING_DLL
    #define PCAM_PUBLIC PCAM_EXPORT
  #else
    #define PCAM_PUBLIC PCAM_IMPORT
  #endif
  #define PCAM_PUBLIC_TYPE PCAM_PUBLIC
  #define PCAM_LOCAL
#else
#define PCAM_EXPORT __attribute__ ((visibility("default")))
#define PCAM_IMPORT
#if __GNUC__ >= 4
#define PCAM_PUBLIC __attribute__ ((visibility("default")))
    #define PCAM_LOCAL  __attribute__ ((visibility("hidden")))
#else
#define PCAM_PUBLIC
#define PCAM_LOCAL
#endif
#define PCAM_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <fstream>
#include <iostream>
#include <chrono>
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
#include <string>
#include <cstdlib>
#include <typeinfo>

// #include <boost/thread.hpp>

// pcam使用時
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

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
#include <sys/ioctl.h>

// JSON読み込み
#include <iostream>
#include <fstream>
#include <sstream>

#include <linux/videodev2.h>

using namespace std::chrono;
using namespace std;
using namespace cv;

using std::placeholders::_1;

#define FMT_NUM_PLANES 3
#define WIDTH 640
#define HEIGHT 480


struct buffer_addr_struct{
    void *start[FMT_NUM_PLANES];
    size_t length[FMT_NUM_PLANES];
} *buffers;

static int xioctl(int fd, int request, void *arg){
    int r;
    do {
        r = ioctl (fd, request, arg);
        if (request == VIDIOC_DQBUF) {
            std::cout << "r : " << r << std::endl;
        }
    } while (-1 == r && EINTR == errno);
    return r;
}

namespace pcam
{
    class Pcam : public rclcpp::Node
    {
    public:
        PCAM_PUBLIC Pcam();

    private:
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr image_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        void setInit();
        void reset();
} // namespace pcam

#endif // PCAM__PCAM_COMPONENT_HPP_