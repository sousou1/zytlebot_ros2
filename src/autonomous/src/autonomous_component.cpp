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

inline double get_time_sec(void) {
    return static_cast<double>(duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count()) / 1000000000;
}

typedef struct object {
public:
    // オブジェクトの種類
    // obstacle, intersection, people
    std::string objType;
    int beforeX;
    int beforeY;
    int findCnt;
    //TODO ros::Time timeStamp;
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

        cout << "Start nodelet" << endl;


        // init start
        // TODO キャリブレーションファイル読み込み
        // cv::FileStorage fs((std::string) params["project_folder"] + "/calibration.yml", cv::FileStorage::READ);
        cv::FileStorage fs(("/calibration.yml", cv::FileStorage::READ);
        fs["mtx"] >> camera_mtx;
        fs["dist"] >> camera_dist;
        fs.release();
        // init end
        
        twist_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        signal_search_ = create_publisher<std_msgs::msg::String>("/signal_search_type", 1);


        // パラメータセット
        // TODO setParam();

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

    void Autonomous::red_flag_update(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "true") {
            red_flag = true;
        } else {
            red_flag = false;
        }

        cout << msg->data << endl;
        cout << red_flag << endl;
    }

    void Autonomous::image_cb(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        cout << "doing" << endl;
    }

} // namespace autorace

CLASS_LOADER_REGISTER_CLASS(autonomous::Autonomous, rclcpp::Node)