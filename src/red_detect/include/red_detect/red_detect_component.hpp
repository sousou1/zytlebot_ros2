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

#include <json_lib/json11.hpp>

#include <vector>
#include <iomanip>
#include <iterator>
#include <cmath>


#define DMA_INTAKE_DMACR  (0x0000)
#define DMA_INTAKE_DMASR  (0x0004)
#define DMA_INTAKE_SA     (0x0018)
#define DMA_INTAKE_LENGTH (0x0028)

#define DMA_OUTLET_DMACR  (0x0030)
#define DMA_OUTLET_DMASR  (0x0034)
#define DMA_OUTLET_DA     (0x0048)
#define DMA_OUTLET_LENGTH (0x0058)

#define DMA_CR_RS         (1u<<0)
#define DMA_CR_RESET      (1u<<2)

#define DMA_SR_HALTED     (1u<<0)
#define DMA_SR_IDLE       (1u<<1)
#define DMA_SR_IOC_Irq    (1u<<12)
#define DMA_SR_ERR_Irq    (1u<<14)

static inline uint32_t regs_read32(void* addr){
    volatile uint32_t* regs_addr = (uint32_t*)(addr);
    return *regs_addr;
}

static inline void regs_write32(void* addr, uint32_t data){
    volatile uint32_t* regs_addr = (uint32_t*)(addr);
    *regs_addr = data;
}

static inline void dma_reset(void* regs){
    regs_write32(regs + DMA_INTAKE_DMACR, DMA_CR_RESET);
    while(regs_read32(regs + DMA_INTAKE_DMACR) & DMA_CR_RESET);
    regs_write32(regs + DMA_OUTLET_DMASR, DMA_CR_RESET);
    while(regs_read32(regs + DMA_OUTLET_DMACR) & DMA_CR_RESET);
}

static inline void dma_setup(void* regs, unsigned long src_addr, unsigned long dst_addr){
    regs_write32(regs + DMA_OUTLET_DMACR, DMA_CR_RS);
    regs_write32(regs + DMA_OUTLET_DA, dst_addr);
    regs_write32(regs + DMA_INTAKE_DMACR, DMA_CR_RS);
    regs_write32(regs + DMA_INTAKE_SA, src_addr);
}

static inline void dma_intake_start(void* regs, unsigned int xfer_size){
    regs_write32(regs + DMA_INTAKE_LENGTH, xfer_size);
}

static inline void dma_outlet_start(void* regs, unsigned int xfer_size){
    regs_write32(regs + DMA_OUTLET_LENGTH, xfer_size);
}

static inline void dma_wait_irq(void* regs){
    while(~regs_read32(regs + DMA_INTAKE_DMASR) & DMA_SR_IOC_Irq);
    while(~regs_read32(regs + DMA_OUTLET_DMASR) & DMA_SR_IOC_Irq);
}

static inline void dma_clear_status(void* regs){
    regs_write32(regs + DMA_INTAKE_DMASR, DMA_SR_IOC_Irq | DMA_SR_ERR_Irq);
    regs_write32(regs + DMA_OUTLET_DMASR, DMA_SR_IOC_Irq | DMA_SR_ERR_Irq);
}

struct udmabuf{
    char name[128];
    int file;
    unsigned char* buf;
    unsigned int buf_size;
    unsigned long phys_addr;
    unsigned long debug_vma;
    unsigned long sync_mode;
};


int udmabuf_open(struct udmabuf* udmabuf, const char* name){
    char file_name[1024];
    int fd;
    unsigned char attr[1024];

    strcpy(udmabuf->name, name);
    udmabuf->file = -1;
    sprintf(file_name, "/sys/class/udmabuf/%s/phys_addr", name);
    if ((fd = open(file_name, O_RDONLY)) == -1){
        printf("Can not open %s\n", file_name);
        return(-1);
    }
    read(fd, (void*)attr, 1024);
    sscanf(attr, "%x", &udmabuf->phys_addr);
    close(fd);

    sprintf(file_name, "/sys/class/udmabuf/%s/size", name);
    if((fd = open(file_name, O_RDONLY)) == -1){
        printf("Can not open %s\n", file_name);
        return(-1);
    }
    read(fd, (void*)attr, 1024);
    sscanf(attr, "%d", &udmabuf->buf_size);
    close(fd);
    sprintf(file_name, "/dev/%s", name);
    if((udmabuf->file = open(file_name, O_RDWR|O_SYNC)) == -1){
        printf("Can not open %s\n", file_name);
        return(-1);
    }
    udmabuf->buf = mmap(NULL, udmabuf->buf_size, PROT_READ|PROT_WRITE, MAP_SHARED, udmabuf->file, 0);
    udmabuf->debug_vma = 0;
    udmabuf->sync_mode = 1;

    return 0;
}


int udmabuf_close(struct udmabuf* udmabuf){
    if(udmabuf->file < 0) return -1;

    close(udmabuf->file);
    udmabuf->file = -1;
    return 0;
}



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
        int find_count;

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

        cv::VideoCapture cap;

        cv::Mat getShrinkFrame(cv::Mat original_img, int window_height);

        vector<pair<pair<int,int>,int>> predictRectFrame(cv::Mat inputimg, int window_height, window_rect waku, int sy = 0, int sx = 0);

        vector<pair<pair<int,int>,int>> processRectFrame(cv::Mat original_img, int window_height, window_rect waku, int sy = 0, int sx = 0);

        void image_cb();
        void signalSearchCb(const std_msgs::msg::String::SharedPtr msg);

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr red_pub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr signal_search_;

        int encoding2mat_type(const std::string & encoding);

        std::string mat_type2encoding(int mat_type);

        void convert_frame_to_message(
                const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image::SharedPtr msg);

        rclcpp::TimerBase::SharedPtr timer_;

    };
} // namespace red_detect

#endif // REDDETECT__REDDETECT_COMPONENT_HPP_