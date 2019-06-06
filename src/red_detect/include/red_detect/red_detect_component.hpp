#if !defined REDDETECT__REDDETECT_COMPONENT_HPP_
#define REDDETECT__REDDETECT_COMPONENT_HPP_


#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
    #define REDDETECT_EXPORT __attribute__ ((dllexport))
    #define REDDETECT_IMPORT __attribute__ ((dllimport))
  #else
    #define REDDETECT_EXPORT __declspec(dllexport)
    #define REDDETECT_IMPORT __declspec(dllimport)
  #endif
  #ifdef REDDETECT_BUILDING_DLL
    #define REDDETECT_PUBLIC REDDETECT_EXPORT
  #else
    #define REDDETECT_PUBLIC REDDETECT_IMPORT
  #endif
  #define REDDETECT_PUBLIC_TYPE REDDETECT_PUBLIC
  #define REDDETECT_LOCAL
#else
#define REDDETECT_EXPORT __attribute__ ((visibility("default")))
#define REDDETECT_IMPORT
#if __GNUC__ >= 4
#define REDDETECT_PUBLIC __attribute__ ((visibility("default")))
    #define REDDETECT_LOCAL  __attribute__ ((visibility("hidden")))
#else
#define REDDETECT_PUBLIC
#define REDDETECT_LOCAL
#endif
#define REDDETECT_PUBLIC_TYPE
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

// red_detect使用時
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

#include "signal_lib/feature.h"
#include "signal_lib/forest.h"
#include "signal_lib/window_candidate.h"
#include "signal_lib/hw.h"

#include <json_lib/json11.hpp>

#include "signal_lib/dma_simple.h"
#include <vector>
#include <iomanip>
#include <iterator>
#include <cmath>

using namespace std::chrono;
using namespace std;
using namespace cv;

using std::placeholders::_1;


struct window_rect{
    int sy;
    int sx;
    int ey;
    int ex;
};


namespace red_detect
{
    class RedDetect : public rclcpp::Node {
    public:
        REDDETECT_PUBLIC RedDetect();

    private:
        int how_search;
        bool find_flag;

        void *dma_regs;
        void *hls_regs;
        struct udmabuf intake_buf;
        struct udmabuf outlet_buf;

        float THRESH;
        float BIAS;
        bool LOG_MODE;

        struct window_rect{
            int sy;
            int sx;
            int ey;
            int ex;
        };

        std::chrono::system_clock::time_point  t1, t2;


        window_rect shukai_waku, cross_waku;

        unsigned int *assignToPhysicalUInt(unsigned long address,unsigned int size);
        int hw_setup();

        void writebram(unsigned int* target, string array_name, json11::Json json, unsigned int fixed_val = 0, bool zeroflag=false);

        int json_setup();

        cv::Mat getShrinkFrame(cv::Mat original_img, int window_height);

        vector<pair<pair<int,int>,int>> predictRectFrame(cv::Mat inputimg, int window_height, window_rect waku, int sy = 0, int sx = 0);

        vector<pair<pair<int,int>,int>> processRectFrame(cv::Mat original_img, int window_height, window_rect waku, int sy = 0, int sx = 0);

        void image_cb(const sensor_msgs::msg::Image::SharedPtr msg);
        void signalSearchCb(const std_msgs::msg::String::SharedPtr msg);

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr red_pub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_ ;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr signal_search_;
    };
} // namespace red_detect

#endif // REDDETECT__REDDETECT_COMPONENT_HPP_