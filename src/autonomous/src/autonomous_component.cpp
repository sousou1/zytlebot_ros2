#include "autonomous/autonomous_component.hpp"

#include <class_loader/register_macro.hpp>
#include <chrono>
#include <memory>


#define PI 3.141592653589793



#define FATAL do { fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", \
                           __LINE__, __FILE__, errno, strerror(errno)); exit(1); } while(0)

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)





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

namespace autonomous {

    Autonomous::Autonomous()
            : Node("autonomous") {
#if !DEBUG
        off_t physical_address = 0x41210000;

        // Switch

        //initialize
        if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) FATAL;
        map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, physical_address & ~MAP_MASK);
        if (map_base == (void *) -1) FATAL;
        virt_addr = map_base + (physical_address & MAP_MASK);
        sw1_flag = false;
        sw2_flag = false;
        sw3_flag = false;


        off_t physical_address2 = 0x41240000;
        // LED

        //initialize
        map_base2 = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, physical_address2 & ~MAP_MASK);
        if (map_base2 == (void *) -1) FATAL;
        virt_addr2 = map_base2 + (physical_address2 & MAP_MASK);
#endif

        cout << "Start autonomous" << endl;


        // init start
        // TODO キャリブレーションファイル読み込み
        // cv::FileStorage fs((std::string) params["project_folder"] + "/calibration.yml", cv::FileStorage::READ);
        cv::FileStorage fs("/calibration.yml", cv::FileStorage::READ);
        fs["mtx"] >> camera_mtx;
        fs["dist"] >> camera_dist;
        fs.release();
        // init end

        twist_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        signal_search_ = create_publisher<std_msgs::msg::String>("/signal_search_type", 1);


        // パラメータセット
        set_param();

#if SIM

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw", std::bind(&Autonomous::image_cb, this, _1));

#else
        image_sub_ = this->create_subscription<std_msgs::msg:::UInt8MultiArray>(
                "/pcam/image_array", std::bind(&Autonomous::image_cb, this, _1));

#endif
        red_pub_ = this->create_subscription<std_msgs::msg::String>(
                "/red_flag", std::bind(&Autonomous::red_flag_update, this, _1));

    }

    void Autonomous::red_flag_update(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "true") {
            red_flag = true;
        } else {
            red_flag = false;
        }

        cout << msg->data << endl;
        cout << red_flag << endl;
    }
#ifdef SIM
    void Autonomous::image_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg){
#else
    void Autonomous::image_cb(const std_msgs::msg::UInt8MultiArray::SharedPtr msg){
#endif

#ifdef SIM
        cv_bridge::CvImagePtr cv_ptr;
        try {
            // ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch (cv_bridge::Exception &e) {
            cout << "error image" << endl;
            return;
        }
        cv::Mat caliblated = cv_ptr->image;
        cv::imshow("image", caliblated);
#else
        cv::Mat base_image(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC2);
        cv::Mat dstimg(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC2);
        memcpy(base_image.data, &msg.data[0], CAMERA_WIDTH * CAMERA_HEIGHT * 2);
        cv::cvtColor(base_image, dstimg, cv::COLOR_YUV2BGR_YUYV);

        cv::Mat caliblated;
        cv::remap(dstimg, caliblated, MapX, MapY, cv::INTER_LINEAR);
#endif
        cout << "doing" << endl;
    }

    void Autonomous::set_param(){
        // 進行方向も初期化読み込み
        std::ifstream ifs("/home/honsen_dir.txt");
        std::string str;
        if (ifs.fail()) {
            std::cerr << "text file load fail" << std::endl;
        }
        int cnt = 0;
        while (getline(ifs, str)) {
            // std::cout << "[" << str << "]" << std::endl;
            int num = std::atoi(str.c_str());
            // std::cout << num << std::endl;
            intersectionDir[cnt++] = num;
        }
        for (int i = 0; i < cnt; i++) {
            std::cout << intersectionDir[i] << std::endl;
        }

        cout << "json before load" << endl;
        ifstream fin("/home/autorace.json" );
        if( !fin ){
            cout << "json load failed" << endl;
            return;
        }

        cout << "json loaded" << endl;

        stringstream strstream;
        strstream << fin.rdbuf();
        fin.close();
        string jsonstr(strstream.str());

        string err;
        auto json = json11::Json::parse(jsonstr, err);
        auto autorace = json["autorace"];

        /*
        int next_y = autorace["next_y"].int_value();
        bool red_obj_search = autorace["red_obj_search"].bool_value();
        double burger_max_lin_vel = autorace["burger_max_lin_vel"].number_value();
         */

        // params set
        red_flag = false;

        // 定数をセット

        cout << "json parse start" << endl;

        Hue_l = autorace["hue_l"].int_value();
        Hue_h = autorace["hue_h"].int_value();
        Saturation_l = autorace["saturation_l"].int_value();
        Saturation_h = autorace["saturation_h"].int_value();
        Lightness_l = autorace["lightness_l"].int_value();
        Lightness_h = autorace["lightness_h"].int_value();
        line_lost_cnt = 0;
        next_tile_x = autorace["next_x"].int_value();
        next_tile_y = autorace["next_y"].int_value();
        now_dir = autorace["start_dir"].int_value();


        cout << "json parse 2" << endl;

        BURGER_MAX_LIN_VEL = autorace["burger_max_lin_vel"].number_value();
        BURGER_MAX_ANG_VEL = autorace["burger_max_ang_vel"].number_value();
        INTERSECTION_STRAIGHT_TIME = autorace["intersection_straight_time"].number_value();

        RIGHT_CURVE_START_LOST_LINE_TIME = autorace["right_curve_start_lost_line_time"].number_value();
        LEFT_CURVE_START_LOST_LINE_TIME = autorace["left_curve_start_lost_line_time"].number_value();
        RIGHT_CURVE_END_MARGIN_TIME = autorace["right_curve_end_margin_time"].number_value();
        RIGHT_CURVE_END_TIME = autorace["right_curve_end_time"].number_value();

        RIGHT_CURVE_VEL = autorace["right_curve_vel"].number_value();
        RIGHT_CURVE_ROT = autorace["right_curve_rot"].number_value();

        LEFT_CURVE_END_TIME = autorace["left_curve_end_time"].number_value();
        LEFT_CURVE_END_MARGIN_TIME = autorace["left_curve_end_margin_time"].number_value();

        LEFT_CURVE_VEL = autorace["left_curve_vel"].number_value();
        LEFT_CURVE_ROT = autorace["left_curve_rot"].number_value();
        LEFT_CURVE_AFTER_ROT = autorace["left_curve_after_rot"].number_value();
        AVOID_OBSTACLE_VEL = autorace["avoid_obstacle_vel"].number_value();
        AVOID_OBSTACLE_ROT = autorace["avoid_obstacle_rot"].number_value();
        AVOID_ROT_TIME = autorace["avoid_rot_time"].number_value();

        RED_OBJ_SEARCH = autorace["red_obj_search"].bool_value();
        FIGURE_SEARCH = autorace["figure_search"].bool_value();

        cout << "json parse 3" << endl;

        AVOID_ROT_STRAIGHT = autorace["avoid_rot_straight"].number_value();
        AVOID_STRAIGHT_TIME = autorace["avoid_straight_time"].number_value();
        AVOID_BEFORE_STRAIGHT_MARGIN_TIME = autorace["avoid_before_straight_margin_time"].number_value();
        INTERSECTION_PREDICTION_TIME_RATIO = autorace["intersection_prediction_time_ratio"].number_value();
        INTERSECTION_CURVE_START_FLAG_RATIO = autorace["intersection_curve_start_flag_ratio"].number_value();
        CROSSWALK_UNDER_MARGIN = autorace["crosswalk_under_margin"].number_value();
        RIGHT_CURVE_UNDER_MARGIN = autorace["right_curve_under_margin"].number_value();
        INTERSECTION_PREDICTION_UNDER_MARGIN = autorace["intersection_prediction_under_margin"].number_value();
        RUN_LINE = autorace["run_line"].number_value();
        RUN_LINE_MARGIN = autorace["run_line_margin"].number_value();
        WIDTH_RATIO = autorace["width_ratio"].number_value();
        HEIGHT_H = autorace["height_h"].number_value();;
        HEIGHT_L = autorace["height_l"].number_value();;
        DETECT_TEMPLATE_RATE = autorace["detect_template_rate"].number_value();;

        BIRDSEYE_LENGTH = autorace["birdseye_length"].int_value();
        CAMERA_WIDTH = autorace["camera_width"].int_value();
        CAMERA_HEIGHT = autorace["camera_height"].int_value();

        SW_CHANGE_PHASE = autorace["sw_change_phase"].string_value();

        RED_DETECT_NUM = autorace["red_detect_num"].int_value();
        // 赤色検出
        RED_HIGH_H = autorace["red_high_h"].int_value();
        RED_HIGH_S = autorace["red_high_s"].int_value();
        RED_HIGH_V = autorace["red_high_v"].int_value();

        RED_LOW_H = autorace["red_low_h"].int_value();
        RED_LOW_S = autorace["red_low_s"].int_value();
        RED_LOW_V = autorace["red_low_v"].int_value();

        // 肌色検出
        SKIN_HIGH_H = autorace["skin_high_h"].int_value();
        SKIN_HIGH_S = autorace["skin_high_s"].int_value();
        SKIN_HIGH_V = autorace["skin_high_v"].int_value();

        SKIN_LOW_H = autorace["skin_low_h"].int_value();
        SKIN_LOW_S = autorace["skin_low_s"].int_value();
        SKIN_LOW_V = autorace["skin_low_v"].int_value();

        cout << "json parse end" << endl;

        /* TODO IMAGE folder
        template_right_T = cv::imread((std::string) params["project_folder"] + "/image/right_T.png", 1);
        template_left_T = cv::imread((std::string) params["project_folder"] + "/image/left_T.png", 1);
        template_under_T = cv::imread((std::string) params["project_folder"] + "/image/under_T.png", 1);
        template_crosswalk = cv::imread((std::string) params["project_folder"] + "/image/crosswalk.png", 1);
        template_right_curve = cv::imread((std::string) params["project_folder"] + "/image/right_curve.png", 1);
        template_intersection = cv::imread((std::string) params["project_folder"] + "/image/intersection.png", 1);
         */

        find_left_line = false;


        detected_line_x = 0;

        // start時間を初期化
        phaseStartTime = get_time_sec();
        line_lost_time = get_time_sec();
        tileUpdatedTime = get_time_sec();
        cycleTime = get_time_sec();

        now_phase = "straight";


        reachBottomRightLaneRightT = false;
        reachBottomRightLaneLeftT = false;
        reachBottomLeftLaneLeftT = false;
        reachBottomLeftLaneStraightEnd = false;
        nowIntersectionCount = 0;
        phaseRunMileage = 0;
        detected_angle = 0;
        intersectionDetectionFlag = false;
        curveAfterCrosswalk = false;
        intersectionAfterCrosswalk = false;
        crosswalkFlag = false;
        rightcurveFlag = false;
        findFigureFlag = false;

        searchType == "";

        acceleration = false;
        // 歪補正の前計算
        mapR = cv::Mat::eye(3, 3, CV_64F);
        cv::initUndistortRectifyMap(camera_mtx, camera_dist, mapR, camera_mtx, cv::Size(640, 480), CV_32FC1, MapX,
                                    MapY);


        // BGS
        bgs = cv::createBackgroundSubtractorMOG2();
        bgs->setVarThreshold(10);

        // 人形のフラグ
        figure_search_phase_limit = false;
        figure_search_cnt = 0;
        figure_search_limit = 2;


        // LEDのためのフラグ
        find_intersection = false;
        do_curve = false ;

        Left_LED = false;
        Right_LED = false;
        Brake_LED = false;

        Left_LED_before = false;
        Right_LED_before = false;
        before_twist_x = 0.0;
        ////////////////

        //image_pub_ = it_.advertise("/image_topic", 1);

        // twist初期化
        //geometry_msgs::Twist twist;
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;
        //limitedTwistPub();


        // TODO valid setSearchType();

        // param set end
        cout << "Hue_h" << Hue_h << endl;
        cout << "red_obj_search" << RED_OBJ_SEARCH << endl;
        cout << "burager_max_lin_vel" << BURGER_MAX_LIN_VEL << endl;
    }


    } // namespace autorace

CLASS_LOADER_REGISTER_CLASS(autonomous::Autonomous, rclcpp::Node)