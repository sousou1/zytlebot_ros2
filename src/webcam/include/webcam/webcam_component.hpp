#if !defined WEBCAM__WEBCAM_COMPONENT_HPP_
#define WEBCAM__WEBCAM_COMPONENT_HPP_


#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
    #define WEBCAM_EXPORT __attribute__ ((dllexport))
    #define WEBCAM_IMPORT __attribute__ ((dllimport))
  #else
    #define WEBCAM_EXPORT __declspec(dllexport)
    #define WEBCAM_IMPORT __declspec(dllimport)
  #endif
  #ifdef WEBCAM_BUILDING_DLL
    #define WEBCAM_PUBLIC WEBCAM_EXPORT
  #else
    #define WEBCAM_PUBLIC WEBCAM_IMPORT
  #endif
  #define WEBCAM_PUBLIC_TYPE WEBCAM_PUBLIC
  #define WEBCAM_LOCAL
#else
#define WEBCAM_EXPORT __attribute__ ((visibility("default")))
#define WEBCAM_IMPORT
#if __GNUC__ >= 4
#define WEBCAM_PUBLIC __attribute__ ((visibility("default")))
    #define WEBCAM_LOCAL  __attribute__ ((visibility("hidden")))
#else
#define WEBCAM_PUBLIC
#define WEBCAM_LOCAL
#endif
#define WEBCAM_PUBLIC_TYPE
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

// webcam使用時
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

namespace webcam
{
    class Webcam : public rclcpp::Node {
    public:
        WEBCAM_PUBLIC Webcam();

    private:
        unsigned char *buffers[REQUEST_BUFFER_NUM];

        int fd;
        struct v4l2_capability caps;
        int got_buffer_num;
        int usbcam_frame;
        std::chrono::system_clock::time_point  t1, t2, t3, t4, t5, t6, t7;

        std_msgs::UInt8MultiArray::SharedPtr camdata;
        struct 	v4l2_buffer buf;


        bool CbFlag;

        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr image_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        void setInit();

        void reset();

        void get_image();
        };
} // namespace webcam

#endif // WEBCAM__WEBCAM_COMPONENT_HPP_