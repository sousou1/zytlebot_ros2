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
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>


// #include <boost/thread.hpp>

// pcam使用時
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <string.h>
#include <iostream>
#include <linux/videodev2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "pcam_lib/cam.h"

using namespace std::chrono;
using namespace std;
using namespace cv;

using std::placeholders::_1;

#define NUM_BUFFER 2

#define WIDTH  640
#define HEIGHT 480


namespace pcam
{
    class Pcam : public rclcpp::Node {
    public:
        PCAM_PUBLIC Pcam();

    private:
        static int v4l2_fd;
        static void *v4l2_user_frame[NUM_BUFFER];

        int rc;
        int w = WIDTH, h = HEIGHT;
        unsigned char *buf;
        rc = v4l2init(w, h, V4L2_PIX_FMT_RGB24);
        cv::Mat frame(h, w, CV_8UC3);

        int v4l2init(int w, int h, __u32 pixelformat);
        int v4l2end(void);
        int v4l2grab(unsigned char **frame);
        int v4l2release(int buf_idx);

        /*
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

        unsigned char *buffer;
        int fd;
        struct v4l2_capability caps;

        int num_planes;
        struct v4l2_requestbuffers reqbuf;
        int MAX_BUF_COUNT;

        int pcam_frame;

        std_msgs::msg::UInt8MultiArray::SharedPtr camdata;
        struct v4l2_buffer buf;

        struct v4l2_plane planes[FMT_NUM_PLANES];

        bool CbFlag;

        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr image_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        void setInit();

        void reset();

        void get_image();
         */
        };
} // namespace pcam

#endif // PCAM__PCAM_COMPONENT_HPP_