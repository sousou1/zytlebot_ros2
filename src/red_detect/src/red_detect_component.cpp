#include "red_detect/red_detect_component.hpp"

#include <class_loader/register_macro.hpp>
#include <chrono>
#include <memory>

using namespace std;
using namespace cv;

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/mman.h>


namespace red_detect {

    RedDetect::RedDetect()
            : Node("red_detect") {
        cout << "start reddetect" << endl;

        find_flag = false;
        find_count = 0;

        if(json_setup() < 0){
            cout << "json setup failed" << endl;
        }
        if(hw_setup() < 0){
            cout << "hw_setup() failed" << endl;
        }
        cout << "hw_setup() finished" << endl;

        cap.open(0);
        if(!cap.isOpened()){
            cout << "failed" << endl;
        }
        signal_search_ = this->create_subscription<std_msgs::msg::String>(
                "/signal_search_type", std::bind(&RedDetect::signalSearchCb, this, _1));

        timer_ = create_wall_timer(100ms, std::bind(&RedDetect::image_cb, this));

        red_pub_ = create_publisher<std_msgs::msg::String>("/red_flag", 1);

        /*
        window_mode = 0;
        how_search = 0;
        cout << "signal start" << endl;


        check_window();
        check_window2("/home/fpga/zytlebot_ros2/src/autonomous");


        // TODO subscribe先はtopicに応じて変更
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/webcam/image_array", std::bind(&RedDetect::image_cb, this, _1));

        signal_search_ = this->create_subscription<std_msgs::msg::String>(
            "/signal_search_type", std::bind(&RedDetect::signalSearchCb, this, _1));

        red_pub_ = create_publisher<std_msgs::msg::String>("/red_flag", 1);

        if(hwmode) hw_setup();
        cout << "nodelet hw setup completed" << endl;

        find_flag = false;
        find_count = 0;
         */
    }

    float RedDetect::rf_test_one_image(Mat img){
        Mat resized_img, gray;
        cv::resize(img, resized_img, cv::Size(64 ,32), INTER_NEAREST);
        cv::Mat resized_hls;
        cv::cvtColor(resized_img, resized_hls, CV_BGR2HSV);
        ​
        cv::Size spatial_size(8, 8);
        Mat spatial_rgb, spatial_hls;
        cv::resize(resized_img, spatial_rgb, spatial_size, INTER_LINEAR);
        cv::resize(resized_hls, spatial_hls, spatial_size, INTER_LINEAR);

        unsigned short feature[672] = {0};
        rf_ravel(spatial_hls, feature);
        rf_ravel(spatial_rgb, feature + 192);
        // hist(resized_hls, feature + 192 * 2);
        // hist(resized_img, feature + 192 * 2 + 36);
        ​
        cv::cvtColor(resized_img, gray, CV_BGR2GRAY);
        rf_lite_hog(gray, feature + 192 * 2);
        ​
        float proba = randomforest_classifier(feature);
        // float red_proba = (float)res.red / (res.not_red + res.red);
        return proba;
    }

    unsigned int* RedDetect::assignToPhysicalUInt(unsigned long address,unsigned int size){
        int devmem = open("/dev/mem", O_RDWR | O_SYNC);
        off_t PageOffset = (off_t) address % getpagesize();
        off_t PageAddress = (off_t) (address - PageOffset);
        return (unsigned int *) mmap(0, size*sizeof(unsigned int), PROT_READ|PROT_WRITE, MAP_SHARED, devmem, PageAddress);
    }

    int RedDetect::hw_setup(){
        int uio1_fd = open("/dev/uio1", O_RDWR);
        hls_regs = mmap(NULL, 0x10000, PROT_READ|PROT_WRITE, MAP_SHARED, uio1_fd, 0);

        int uio2_fd = open("/dev/uio2", O_RDWR);
        dma_regs = mmap(NULL, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, uio2_fd, 0);
        printf("mmap end\n");

        if(udmabuf_open(&intake_buf, "udmabuf0") == -1){
            printf("udmabuf_open failed\n");
            return -1;
        }
        if(udmabuf_open(&outlet_buf, "udmabuf1") == -1){
            printf("udmabuf_open failed\n");
            return -1;
        }
        printf("open end / reset start\n");
        dma_reset(dma_regs);
        printf("dma reset end\n");
        regs_write32(hls_regs, 0x81);
        return 1;
    }

    void RedDetect::putRectangle(vector<pair<pair<int,int>,int> > window_candidates, Mat frame_image){
        for(int i = 0; i < window_candidates.size(); i++){
            int sy = window_candidates[i].first.first;
            int sx = window_candidates[i].first.second;
            int window_height = window_candidates[i].second;
            rectangle(frame_image, Point(sx, sy), Point(sx + window_height*2, sy + window_height), Scalar(0,0,200), 2); //x,y //Scaler = B,G,R
        }
    }

    void RedDetect::signalSearchCb(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "-1") {
            how_search = -1;
        } else if (msg->data == "0") {
            how_search = 0;
        } else {
            how_search = 1;
        }
    }

    void RedDetect::image_cb() {
        if(LOG_MODE) cout << "search signal Type = " << how_search << "!!!!!!" << endl;


        if (how_search != -1) {
            if(LOG_MODE) cout << "searching signal!!!!" << endl;

            cv::Mat frame;
            cap >> frame; // get a new frame from camera
            //cv::Mat res = frame.clone();
            window_rect waku;
            // 0なら周回
            // それ以外　交差点
            if(how_search == 0) waku = shukai_waku;
            else waku = cross_waku;

            // auto candidates = processRectFrame(frame, 32, waku, 240, 160);
            auto candidates2 = processRectFrame(frame, 80, waku);
            //auto candidates3 = processRectFrame(frame, 70, waku);
            auto candidates4 = processRectFrame(frame, 60, waku);
            auto candidates5 = processRectFrame(frame, 50, waku);

            /*putRectangle(candidates, res);
             * putRectangle(candidates2, res);
                putRectangle(candidates4, res);
                putRectangle(candidates5, res);
                cv::imshow("result", res);
                cv::waitKey(1);
                */
            auto find = std::make_shared<std_msgs::msg::String>();

            bool now_find = (candidates2.size() > 0 || candidates4.size() > 0 || candidates5.size() > 0);


            if (now_find) { // 見つかった場合
                if (find_count >= 0) { // find_countが0以上の場合
                    find_count++;
                } else {
                    find_count = 1; // find_countが負の値の場合
                }
            } else { //　見つからなかった場合
                if (find_count <= 0) {
                    find_count--; // find_countが負の場合、-1
                } else {
                    find_count = -1; // find_countが正の値の場合
                }
            }

            // 3回連続で見つけたり見失ったらフラグを変更
            if (find_count >= 3) {
                find_flag = true;
            } else if (find_count <= -3) {
                find_flag = false;
            }

            if (find_flag) {
                find->data = "true";
            } else {
                find->data = "false";
            }


            t2 = std::chrono::system_clock::now();
            double elapsed = (double)std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
            if(LOG_MODE) cout << "elapsed:" << elapsed << "[milisec]" << endl;
            if(LOG_MODE) cout << "fps:" << 1000.0/elapsed << "[fps]" << endl;

            red_pub_->publish(find);
        }
    }

    void RedDetect::writebram(unsigned int* target, string array_name, json11::Json json, unsigned int fixed_val = 0, bool zeroflag=false){
        int cnt = 0;
        for(auto &k: json[array_name].array_items()){
            if(LOG_MODE) cout << cnt << " ";
            if(fixed_val == 0 && zeroflag == false) target[cnt++] = k.number_value();
            else target[cnt++] = fixed_val;
        }
    }
    int RedDetect::json_setup(){
        unsigned int* bgrhsv_w1 = assignToPhysicalUInt(0xB0014000, 24*4);
        unsigned int* bgrhsv_w2 = assignToPhysicalUInt(0xB0018000, 24*4);
        unsigned int* bgrhsv_w3 = assignToPhysicalUInt(0xB001C000, 24*4);
        unsigned int* bgrhsv_w4 = assignToPhysicalUInt(0xB0020000, 24*4);
        unsigned int* hog_w1 = assignToPhysicalUInt(0xB0024000, 63*4);
        unsigned int* hog_w2 = assignToPhysicalUInt(0xB0028000, 63*4);
        unsigned int* hog_w3 = assignToPhysicalUInt(0xB002C000, 63*4);

        ifstream ifs("/home/fpga/zytlebot_ros2/src/red_detect/weights.json");
        if(ifs.fail()){
            cerr << "json file load failed" << endl;
            return -1;
        }
        std::istreambuf_iterator<char> it(ifs);
        std::istreambuf_iterator<char> last;
        std::string str(it, last);
        string err;
        auto json = json11::Json::parse(str, err);
        if(err != ""){
            cerr << "json file parse error: " << err << endl;
            return -1;
        }

        writebram(hog_w1, "hog_w1", json);
        writebram(hog_w2, "hog_w2", json);
        writebram(hog_w3, "hog_w3", json);
        writebram(bgrhsv_w1, "bgrhsv_w1", json);
        writebram(bgrhsv_w2, "bgrhsv_w2", json);
        writebram(bgrhsv_w3, "bgrhsv_w3", json);
        writebram(bgrhsv_w4, "bgrhsv_w4", json);

        THRESH = json["thresh"].number_value();
        BIAS = json["bias"].number_value();
        LOG_MODE = (json["log_mode"].int_value() == 1);

        shukai_waku.sy = json["shukai_sy"].int_value();
        shukai_waku.sx = json["shukai_sx"].int_value();
        shukai_waku.ey = json["shukai_ey"].int_value();
        shukai_waku.ex = json["shukai_ex"].int_value();

        cross_waku.sy = json["cross_sy"].int_value();
        cross_waku.sx = json["cross_sx"].int_value();
        cross_waku.ey = json["cross_ey"].int_value();
        cross_waku.ex = json["cross_ex"].int_value();

        return 1;
    }

    cv::Mat RedDetect::getShrinkFrame(cv::Mat original_img, int window_height){
        //shrink whole frame
        float scaling = 32.0/window_height;
        // cv::Mat resized_img(cv::Size(640.0*scaling, 480.0*scaling), CV_8UC3);
        cv::Mat resized_img;
        resize(original_img, resized_img, cv::Size(), scaling, scaling);
        //pase scaled image to 320x240 canvas
        cv::Mat scaled_frame(cv::Size(320, 240), CV_8UC3);
        cv::Mat roi_dst = scaled_frame(cv::Rect(0, 0, 640*scaling, 480*scaling));
        resized_img.copyTo(roi_dst);
        return scaled_frame;
    }

    int RedDetect::encoding2mat_type(const std::string & encoding)
    {
        if (encoding == "mono8") {
            return CV_8UC1;
        } else if (encoding == "bgr8") {
            return CV_8UC3;
        } else if (encoding == "mono16") {
            return CV_16SC1;
        } else if (encoding == "rgba8") {
            return CV_8UC4;
        } else if (encoding == "bgra8") {
            return CV_8UC4;
        } else if (encoding == "32FC1") {
            return CV_32FC1;
        } else if (encoding == "rgb8") {
            return CV_8UC3;
        } else {
            throw std::runtime_error("Unsupported encoding type");
        }
    }

    void RedDetect::convert_frame_to_message(
            const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image::SharedPtr msg)
    {
        // copy cv information into ros message
        msg->height = frame.rows;
        msg->width = frame.cols;
        msg->encoding = mat_type2encoding(frame.type());
        msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
        size_t size = frame.step * frame.rows;
        msg->data.resize(size);
        memcpy(&msg->data[0], frame.data, size);
        msg->header.frame_id = std::to_string(frame_id);
    }

    std::string RedDetect::mat_type2encoding(int mat_type)
    {
        switch (mat_type) {
            case CV_8UC1:
                return "mono8";
            case CV_8UC3:
                return "bgr8";
            case CV_16SC1:
                return "mono16";
            case CV_8UC4:
                return "rgba8";
            default:
                throw std::runtime_error("Unsupported encoding type");
        }
    }

    //input: 320x240pix
    //output: ((sy,sx), window_height)
    vector<pair<pair<int,int>,int>> RedDetect::predictRectFrame(cv::Mat inputimg, int window_height, window_rect waku, int sy = 0, int sx = 0){
        vector<pair<pair<int,int>,int>> rst;

        cv::Mat bgra_img;
        cv::cvtColor(inputimg, bgra_img, CV_BGR2BGRA);
        //copy image pixel value to input buffer
        for(int y = 0; y < 240; y++)  {
        memcpy(intake_buf.buf + 320 * y * 4, bgra_img.data + bgra_img.step * y, bgra_img.cols * 4 * sizeof(unsigned char));
    }

    dma_setup(dma_regs, intake_buf.phys_addr, outlet_buf.phys_addr);
    dma_outlet_start(dma_regs, 27*33*4);
    dma_intake_start(dma_regs, 320*240*4);
    dma_wait_irq(dma_regs);
    dma_clear_status(dma_regs);

int removecnt = 0;
for(int y = 0; y < 27; y++){
for(int x = 0; x < 33; x++){
int index = y * 33 + x;
float hwrst = ((float*) (outlet_buf.buf))[index];
hwrst += 1.7700042; //must be done
hwrst += BIAS;
// float proba = 1.0/(1.0+exp(-rst));
if(hwrst > THRESH){
Mat window_bgr_img = inputimg(cv::Rect(x*8, y*8, 64, 32));
float rf_proba = rf_test_one_image(window_bgr_img);
// cout << "rf_proba : " << rf_proba << endl;
if(rf_proba < 0.50){
removecnt++;
continue;
}
//inside window rect
int detected_y = (int)((float)y * 8 * window_height / 32.0) + sy;
int detected_x = (int)((float)x * 8 * window_height / 32.0) + sx;
if(waku.sy <= detected_y && waku.sx <= detected_x && detected_y + window_height <= waku.ey && detected_x + window_height*2 <= waku.ex){
float proba = 1.0/(1.0+exp(-hwrst));
if(LOG_MODE) cout << window_height << " " << proba << " " << detected_y << " " << detected_x << endl;
rst.push_back(make_pair(make_pair(detected_y, detected_x), window_height));
}
}
}
}
cout << "removecnt :" << removecnt << endl;
return rst;

    vector<pair<pair<int,int>,int>> RedDetect::processRectFrame(cv::Mat original_img, int window_height, window_rect waku, int sy = 0, int sx = 0){
    if(window_height <= 32){
    //crop and predict
    Mat cropped_img = original_img(cv::Rect(sx, sy, 320, 240));
    return predictRectFrame(cropped_img, window_height, waku, sy, sx);
    }else if(window_height <= 64){
    cv::Mat resized_img;
    float scaling = 32.0/window_height;
    resize(original_img, resized_img, cv::Size(), scaling, scaling);
    Mat cropped_img = resized_img(cv::Rect(0, 0, 320, 240));
    return predictRectFrame(cropped_img, window_height, waku);
    }else {
    Mat shrinked_img = getShrinkFrame(original_img, window_height);
    return predictRectFrame(shrinked_img, window_height, waku);
    }
    }



} // namespace autorace

CLASS_LOADER_REGISTER_CLASS(red_detect::RedDetect, rclcpp::Node)