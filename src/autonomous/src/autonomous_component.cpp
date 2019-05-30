#include <fstream>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include "unistd.h"
#include <math.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>
#include <string>
#include <cstdlib>
#include <typeinfo>

#include <boost/thread.hpp>

// pcam使用時
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/String.h"

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

using namespace std::chrono;
inline double get_time_sec(void){
    return static_cast<double>(duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count())/1000000000;
}
typedef struct object {
public:
    // オブジェクトの種類
    // obstacle, intersection, people
    std::string objType;
    int beforeX;
    int beforeY;
    int findCnt;
    ros::Time timeStamp;
} OBJECT;


// 直線を中点、傾き、長さで表す
typedef struct straight {
    cv::Point2f middle;
    double degree;
    double length;
} STRAIGHT;

// map_data[y][x][0]がタイルの種類
// map_data[y][x][1]が向きを表している
// 向きは1がデータ画像のとおりで、0~3で右回りに表現されている
int map_data[7][5][2] = {{{3, 0}, {4, 0}, {7, 2}, {4, 0}, {3, 1}},
                         {{6, 1}, {0, 0}, {1, 1}, {0, 0}, {6, 1}},
                         {{4, 1}, {0, 0}, {5, 1}, {0, 0}, {4, 1}},
                         {{7, 1}, {2, 0}, {8, 0}, {2, 2}, {7, 3}},
                         {{4, 1}, {0, 0}, {5, 3}, {0, 0}, {4, 1}},
                         {{6, 1}, {0, 0}, {1, 3}, {0, 0}, {6, 1}},
                         {{3, 3}, {4, 0}, {7, 0}, {4, 0}, {3, 2}}};

int intersectionDir[100] = {0};

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
        pub_ = create_publisher<greeting_msg::msg::Greeting>("greeting");

        sub_ = this->create_subscription<std_msgs::msg::String>(
                "autonomous", std::bind(&Autonomous::display_greeting, this, _1));


        ros::Timer led_timer;

        bool red_flag;

        // 定数宣言
        int BIRDSEYE_LENGTH, CAMERA_WIDTH, CAMERA_HEIGHT;

        double BURGER_MAX_LIN_VEL, BURGER_MAX_ANG_VEL, RIGHT_CURVE_START_LOST_LINE_TIME, LEFT_CURVE_START_LOST_LINE_TIME, RIGHT_CURVE_END_MARGIN_TIME, RIGHT_CURVE_END_TIME,
                RIGHT_CURVE_VEL , RIGHT_CURVE_ROT , LEFT_CURVE_END_TIME , LEFT_CURVE_END_MARGIN_TIME , LEFT_CURVE_VEL , LEFT_CURVE_ROT , LEFT_CURVE_AFTER_ROT ,
                AVOID_OBSTACLE_VEL , AVOID_OBSTACLE_ROT , AVOID_ROT_TIME , AVOID_ROT_STRAIGHT , AVOID_STRAIGHT_TIME , AVOID_BEFORE_STRAIGHT_MARGIN_TIME , INTERSECTION_PREDICTION_TIME_RATIO , DETECT_TEMPLATE_RATE,
                CROSSWALK_UNDER_MARGIN, RIGHT_CURVE_UNDER_MARGIN , INTERSECTION_PREDICTION_UNDER_MARGIN , INTERSECTION_CURVE_START_FLAG_RATIO , RUN_LINE , RUN_LINE_MARGIN , WIDTH_RATIO , HEIGHT_H , HEIGHT_L, INTERSECTION_STRAIGHT_TIME;


        int Hue_l, Hue_h, Saturation_l, Saturation_h, Lightness_l, Lightness_h;

        cv::Mat camera_mtx;
        cv::Mat camera_dist;

        geometry_msgs::Twist twist;

        ros::Publisher twist_pub;
        // cv::Mat curve_image;
        int line_lost_cnt;
        int curve_detect_cnt;
        int find_T;
        int find_X;

        bool find_left_line;

        std::string now_phase;
        std::string nextSearchObject;

        bool RED_OBJ_SEARCH;
        int RED_DETECT_NUM;

        bool FIGURE_SEARCH;

        int figure_search_limit;
        int figure_search_cnt;
        bool figure_search_phase_limit;

        // 発見したオブジェクト（交差点、障害物）のリスト
        std::list <OBJECT> objects;


        PHASE_INFO_BACKUP backupInfo;

        // 次のtileを保存
        int next_tile_x;
        int next_tile_y;
        int now_dir;

        // 検出された直線のx座標
        int detected_line_x;

        ros::Time phaseStartTime;
        ros::Time tileUpdatedTime;
        ros::Time line_lost_time;
        ros::Time cycleTime;

        // change phaseで初期化
        // bottomにオブジェクトが到達したかどうか
        bool reachBottomRightLaneRightT;
        bool reachBottomRightLaneLeftT;
        bool reachBottomLeftLaneLeftT;
        bool reachBottomLeftLaneStraightEnd;

        // 交差点の挙動決定のための配列の位置
        int nowIntersectionCount;

        // BIRDS_EYE_LENGTHの3/4に右のT字路が到達したかどうか
        bool intersectionDetectionFlag;

        // 横断歩道の（停止線）の位置に来た時trueになる
        bool crosswalkFlag;

        bool rightcurveFlag;

        // 人形を見つけているかどうか
        bool findFigureFlag;


        // カーブの次が横断歩道の場合、カーブ終了後横断歩道を認識するまで少しストップ
        bool curveAfterCrosswalk;
        bool intersectionAfterCrosswalk;

        string SW_CHANGE_PHASE;

        double mileage;
        double phaseRunMileage;
        double detected_angle;


        // 加速するかしないか
        bool acceleration;

        // テンプレートマッチングで探す形
        std::string searchType;

        XmlRpc::XmlRpcValue params;

        cv::Mat template_right_T;
        cv::Mat template_left_T;
        cv::Mat template_under_T;
        cv::Mat template_crosswalk;
        cv::Mat template_right_curve;
        cv::Mat template_intersection;

        cv::Mat aroundDebug;

        // BackgroundSubtractorMOG2
        cv::Ptr<cv::BackgroundSubtractorMOG2> bgs;
        cv::Mat bgmask, out_frame;

        // 歪補正に使う
        cv::Mat MapX, MapY, mapR;

        std::string project_folder;

        // 赤色検出 //////////////
        int RED_HIGH_H;
        int RED_HIGH_S;
        int RED_HIGH_V;

        int RED_LOW_H;
        int RED_LOW_S;
        int RED_LOW_V;

        // 肌色検出 //////////////
        int SKIN_HIGH_H;
        int SKIN_HIGH_S;
        int SKIN_HIGH_V;

        int SKIN_LOW_H;
        int SKIN_LOW_S;
        int SKIN_LOW_V;

        //////////////////

        //////////////// LEDのためのフラグ/////////////////
        bool find_intersection;
        bool do_curve;

        // 点滅させるか
        bool Left_LED;
        bool Right_LED;
        bool Brake_LED;

        // 点滅させるために、前状態を持っておくための変数
        bool Left_LED_before;
        bool Right_LED_before;

        double before_twist_x;

        //////////


#if !DEBUG
        // devmem
        int fd;
        void* map_base;
        void* virt_addr;
        void* virt_addr2;
        bool sw1_flag;
        bool sw2_flag;
        bool sw3_flag;

        // LED
        void* map_base2;
#endif


#if !DEBUG
        off_t physical_address = 0x41210000;

        // Switch

        //initialize
        if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) FATAL;
        map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, physical_address & ~MAP_MASK);
        if(map_base == (void *) -1) FATAL;
        virt_addr = map_base + (physical_address & MAP_MASK);
        sw1_flag = false;
        sw2_flag = false;
        sw3_flag = false;


        off_t physical_address2 = 0x41240000;
        // LED

        //initialize
        map_base2 = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, physical_address2 & ~MAP_MASK);
        if(map_base2 == (void *) -1) FATAL;
        virt_addr2 = map_base2 + (physical_address2 & MAP_MASK);
#endif

        cout << "Start nodelet" << endl;


        // init start
        // キャリブレーションファイル読み込み
        cv::FileStorage fs((std::string) params["project_folder"] + "/calibration.yml", cv::FileStorage::READ);
        fs["mtx"] >> camera_mtx;
        fs["dist"] >> camera_dist;
        fs.release();

        // init end

        twist_pub = create_publisher<geometry_msgs::Twist>("/cmd_vel", 1);
        signal_search_ = create_publisher<std_msgs::String>(("/signal_search_type", 1);


        // パラメータセット
        setParam();

#if DEBUG

        image_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
    "/image_array", std::bind(&Autonomous::image_cb, this, _1));

#else
        image_sub_ = this->create_subscription<std_msgs::msg:::UInt8MultiArrayPtr>(
                "/pcam/image_array", std::bind(&Autonomous::image_cb, this, _1));

#endif
        red_pub_ = this->create_subscription<std_msgs::msg::String>(
                "/red_flag", std::bind(&Autonomous::redFlagUpdate, this, _1));

        //  処理した挙動をパブリッシュ


#if !DEBUG
        // LEDの点灯
        led_timer = nh_.createTimer(ros::Duration(0.5), boost::bind(&NodeletAutorace::ledCb, this, _1));
#endif
        //
    }

    void Autonomous::red_flag_update(const std_msgs::String &msg) {
        if (msg.data == "true") {
            red_flag = true;
        } else {
            red_flag = false;
        }

        cout << msg.data << endl;
        cout << red_flag << endl;
    }

    void Autonomous::image_cb(const std_msgs::msg:::UInt8MultiArrayPtr msg)
    {
        cout << "doing" << endl;
    }

} // namespace autorace

CLASS_LOADER_REGISTER_CLASS(autonomous::Autonomous, rclcpp::Node)