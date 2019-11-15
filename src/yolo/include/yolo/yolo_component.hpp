#if !defined YOLO__YOLO_COMPONENT_HPP_
#define YOLO__YOLO_COMPONENT_HPP_


#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
    #define YOLO_EXPORT __attribute__ ((dllexport))
    #define YOLO_IMPORT __attribute__ ((dllimport))
  #else
    #define YOLO_EXPORT __declspec(dllexport)
    #define YOLO_IMPORT __declspec(dllimport)
  #endif
  #ifdef YOLO_BUILDING_DLL
    #define YOLO_PUBLIC YOLO_EXPORT
  #else
    #define YOLO_PUBLIC YOLO_IMPORT
  #endif
  #define YOLO_PUBLIC_TYPE YOLO_PUBLIC
  #define YOLO_LOCAL
#else
#define YOLO_EXPORT __attribute__ ((visibility("default")))
#define YOLO_IMPORT
#if __GNUC__ >= 4
#define YOLO_PUBLIC __attribute__ ((visibility("default")))
    #define YOLO_LOCAL  __attribute__ ((visibility("hidden")))
#else
#define YOLO_PUBLIC
#define YOLO_LOCAL
#endif
#define YOLO_PUBLIC_TYPE
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

// yolo使用時
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include <algorithm>
#include <vector>
#include <atomic>
#include <queue>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <mutex>
#include <zconf.h>
#include <thread>
#include <sys/stat.h>
#include <dirent.h>
#include <functional>

#include <dnndk/dnndk.h>

#include "utils.h"

using namespace std;
using namespace cv;
using namespace std::chrono;


#define INPUT_NODE "layer0_conv"

typedef pair<int, Mat> imagePair;

class paircomp {
 public:
  bool operator()(const imagePair &n1, const imagePair &n2) const {
    if (n1.first == n2.first) {
        return (n1.first > n2.first);
      }

    return n1.first > n2.first;
  }
};

namespace yolo
{
    class Yolo : public rclcpp::Node {
    public:
        YOLO_PUBLIC Yolo();

    private:

      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr object_search_result_;
      rclcpp::TimerBase::SharedPtr timer_;


      int idxInputImage = 0;  // frame index of input video
      int idxShowImage = 0;   // next frame index to be displayed
      bool bReading = true;   // flag of reding input frame
      chrono::system_clock::time_point start_time;

      cv::VideoCapture cap;

      int is_video=0;

      // mutex for protection of input frames queue
      mutex mtxQueueInput;
      // mutex for protection of display frmaes queue
      mutex mtxQueueShow;
      // input frames queue
      queue<pair<int, Mat>> queueInput;
      // display frames queue
      priority_queue<imagePair, vector<imagePair>, paircomp> queueShow;

      void image_cb();

      void setInputImageForYOLO(DPUTask* task, const Mat& frame, float* mean);

      void readFrame(const char *fileName);

      void displayFrame();

      void postProcess(DPUTask* task, Mat& frame, int sWidth, int sHeight);

      void runYOLO(DPUTask* task, Mat& img);

      void runYOLO_video(DPUTask* task);
      
      };
} // namespace yolo

#endif // YOLO__YOLO_COMPONENT_HPP_