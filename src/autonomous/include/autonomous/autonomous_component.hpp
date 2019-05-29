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

namespace autonomous
{

    class AUTONOMOUS : public rclcpp::Node
    {
    public:
        AUTONOMOUS_PUBLIC AUTONOMOUS();

        ros::Timer led_timer;

        bool red_flag;

        // 定数宣言
        int BIRDSEYE_LENGTH, CAMERA_WIDTH, CAMERA_HEIGHT;

        double BURGER_MAX_LIN_VEL, BURGER_MAX_ANG_VEL, RIGHT_CURVE_START_LOST_LINE_TIME, LEFT_CURVE_START_LOST_LINE_TIME, RIGHT_CURVE_END_MARGIN_TIME, RIGHT_CURVE_END_TIME,
                RIGHT_CURVE_VEL , RIGHT_CURVE_ROT , LEFT_CURVE_END_TIME , LEFT_CURVE_END_MARGIN_TIME , LEFT_CURVE_VEL , LEFT_CURVE_ROT , LEFT_CURVE_AFTER_ROT ,
                AVOID_OBSTACLE_VEL , AVOID_OBSTACLE_ROT , AVOID_ROT_TIME , AVOID_ROT_STRAIGHT , AVOID_STRAIGHT_TIME , AVOID_BEFORE_STRAIGHT_MARGIN_TIME , INTERSECTION_PREDICTION_TIME_RATIO , DETECT_TEMPLATE_RATE,
                CROSSWALK_UNDER_MARGIN, RIGHT_CURVE_UNDER_MARGIN , INTERSECTION_PREDICTION_UNDER_MARGIN , INTERSECTION_CURVE_START_FLAG_RATIO , RUN_LINE , RUN_LINE_MARGIN , WIDTH_RATIO , HEIGHT_H , HEIGHT_L, INTERSECTION_STRAIGHT_TIME;


        int Hue_l, Hue_h, Saturation_l, Saturation_h, Lightness_l, Lightness_h;





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

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        void broadcast_autonomous();
    };

} // namespace autonomous_ros2_style

#endif // AUTONOMOUS__AUTONOMOUS_NODE_HPP_