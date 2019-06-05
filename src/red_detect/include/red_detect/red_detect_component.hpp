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

using namespace std::chrono;
using namespace std;
using namespace cv;

using std::placeholders::_1;

map<int, vector<pair<int, int> > > mp[2];
map<int, vector<pair<int, int> > > original_mp[2];
set<int> widthkind[2];

int sx_min[2] = {999, 999};
int sy_min[2] = {999, 999};
int ex_max[2] = {-1, -1};
int ey_max[2] = {-1, -1};
#define WINDOW_WIDTH 64
#define WINDOW_HEIGHT 32
bool imgout = false;
float proba_thresh = 0.65;
#define hwmode true
#define checkmode false
#define showdetailtime false
#define FEATURE_SIZE 672

unsigned short sw_feature[FEATURE_SIZE*4] = {0};
unsigned short hw_feature[FEATURE_SIZE*4] = {0};


namespace red_detect
{
    class RedDetect : public rclcpp::Node {
    public:
        REDDETECT_PUBLIC RedDetect();

    private:
        int window_mode;
        int how_search; // -1で探さない 0で外周 1で交差点
        bool find_flag;
        int find_count;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr red_pub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_ ;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr signal_search_;

        void check_window();
        void check_window2(std::string project_folder);
        void signalSearchCb(const std_msgs::msg::String::SharedPtr msg);
        void image_cb(const sensor_msgs::msg::Image msg);
        bool hwresultcheck(unsigned short* sw_feature, unsigned short* hw_feature, int start, int end);
        void test_four_window(float* result, int num, Mat rgb[4], Mat hls[4], Mat gray[4], double* time0, double* time1, double* time2, double *time3);
        vector<pair<vector<int>, float>> test_one_frame(Mat frame, int mode);


        };
} // namespace red_detect

#endif // REDDETECT__REDDETECT_COMPONENT_HPP_