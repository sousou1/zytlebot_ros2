#include "autonomous/autonomous_component.hpp"

#include <class_loader/register_macro.hpp>
#include <chrono>
#include <memory>


#define PI 3.141592653589793

#define DEBUG false

#define FATAL do { fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", \
						   __LINE__, __FILE__, errno, strerror(errno)); exit(1); } while(0)

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

using namespace std;
using namespace cv;

using std::placeholders::_1;

using namespace std::chrono;


/*
now_phaseについて
0なら直線検出モード
1なら右カーブ検出決め打ち移動状態
2なら左カーブ検出
3ならカーブ終了処理中直線検出以降状態（緩やかにカーブをやめて直線モードに移行）
4なら障害物検知状態
5なら信号検知状態
*/

/*
 * TODO
 * 右カーブ直後の横断歩道の認識が苦手なため、afterCurveSkipフラグによってスキップさせている
 */

namespace autonomous
{

    Autonomous::Autonomous()
            : Node("autonomous")
    {
        // init end

        twist_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        signal_search_ = create_publisher<std_msgs::msg::String>("/signal_search_type", 1);
/*

#if DEBUG

        image_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
    "/image_array", std::bind(&Autonomous::image_cb, this, _1));

#else
        image_sub_ = this->create_subscription<std_msgs::msg:::UInt8MultiArrayPtr>(
                "/pcam/image_array", std::bind(&Autonomous::image_cb, this, _1));

#endif
        red_pub_ = this->create_subscription<std_msgs::msg::String>(
                "/red_flag", std::bind(&Autonomous::redFlagUpdate, this, _1));
*/
    }

} // namespace autorace

CLASS_LOADER_REGISTER_CLASS(autonomous::Autonomous, rclcpp::Node)