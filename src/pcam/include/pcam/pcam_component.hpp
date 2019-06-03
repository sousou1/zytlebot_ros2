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

#include "sensor_msgs/msg/image.hpp"


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

        static int xioctl(int fd, int request, void *arg){
                int rc;
                do rc = ioctl(fd, request, arg);
                while (-1 == rc && EINTR == errno);
                return rc;
        }

        void get_image();
        int v4l2init(int w, int h, __u32 pixelformat);
        int v4l2end(void);
        int v4l2grab(unsigned char **frame);
        int v4l2release(int buf_idx);
        std::string mat_type2encoding(int mat_type);

        void convert_frame_to_message(
                const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image::SharedPtr msg);

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        };
} // namespace pcam

#endif // PCAM__PCAM_COMPONENT_HPP_