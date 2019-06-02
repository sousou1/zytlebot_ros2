#include "pcam/pcam_component.hpp"

#include <class_loader/register_macro.hpp>
#include <chrono>
#include <memory>


namespace pcam {

    Pcam::Pcam()
            : Node("pcam") {
        image_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/pcam/image_array", 1);

        setInit();

        // TODO 時間設定
        timer_ = create_wall_timer(1s, std::bind(&Pcam::get_image, this));
    }

    void Pcam::get_image() {
        // 7. Capture Image
        if (CbFlag) {
            cout << "pcam Cb fail" << endl;
        } else
        {
            CbFlag = true;
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(fd, &fds);
            struct timeval tv = {0};
            tv.tv_sec = 2;
            int r = select(fd + 1, &fds, NULL, NULL, &tv);

            if (-1 == r) {
                std::cout << "Waiting for Frame" << std::endl;
                return;
            }



            memset(&buf, 0, sizeof(buf));
            memset(planes, 0, sizeof(planes));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.m.planes = planes;
            buf.length = FMT_NUM_PLANES;

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
                std::cout << "Pcam Retrieving Frame" << std::endl;
                reset();
                return;
            }

            // 8. Store Image in OpenCV Data Type
            for (int j = 0; j < num_planes; j++) {
                memcpy(&(camdata->data[0]), buffers[buf.index].start[j], WIDTH * HEIGHT * 2);
                image_pub_->publish(camdata);
                std::cout << "pcam publish" << std::endl;
            }

            std::cout << "buf.index " << buf.index << std::endl;
            // Connect buffer to queue for next capture.
            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) {
                std::cout << "VIDIOC_QBUF" << std::endl;
            }

            CbFlag = false;
        }
    }

    void Pcam::setInit() {

        CbFlag = false;

        // 1. Open Video Device.
        fd = open("/dev/video0", O_RDWR, 0);
        if (fd == -1){
            std::cout << "Failed to open video device." << std::endl;
            return;
        }

        // 2. Querying video capabilities.
        memset(&caps, 0, sizeof(caps));
        if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &caps)){
            std::cout << "Failed to query capabilities." << std::endl;
            return;
        }
        std::cout << "bus_info	: " << caps.bus_info << std::endl;
        std::cout << "card		: " << caps.card << std::endl;
        std::cout << "driver	: " << caps.driver << std::endl;
        std::cout << "version	: " << caps.version << std::endl;

        // 3. Format Specification.
        {
            struct v4l2_format fmt;
            memset(&(fmt), 0, sizeof(fmt));

            fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            fmt.fmt.pix_mp.width = WIDTH;
            fmt.fmt.pix_mp.height = HEIGHT;
            fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_YUYV;
            fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;

            if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)){
                std::cout << "Failed to set pixel format." << std::endl;
                return;
            }
        }

        MAX_BUF_COUNT = 3;/*we want at least 3 buffers*/

        // 4. Request Buffer
        {
            memset(&(reqbuf), 0, sizeof(reqbuf));
            reqbuf.count = FMT_NUM_PLANES;
            reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            reqbuf.memory = V4L2_MEMORY_MMAP;

            if (-1 == xioctl(fd, VIDIOC_REQBUFS, &reqbuf)){
                std::cout << "Failed to request buffer." << std::endl;
                return;
            }
            if (reqbuf.count < MAX_BUF_COUNT){
                std::cout << "Not enought buffer memory." << std::endl;
                return;
            }
            std::cout << "reqbuf.count : " << reqbuf.count << std::endl;

            buffers = (buffer_addr_struct*) calloc(reqbuf.count, sizeof(*buffers));
            assert(buffers != NULL);

        }

        // 5. Query Buffer
        {
            for(int i = 0; i < reqbuf.count; i++){

                struct  v4l2_plane planes[FMT_NUM_PLANES];
                struct  v4l2_buffer buf;
                memset(&(buf), 0, sizeof(buf));
                memset(planes, 0, sizeof(planes));
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.m.planes = planes;
                buf.length = FMT_NUM_PLANES;
                buf.index = i;
                if(-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf)){
                    std::cout << "Failed to query buffer." << std::endl;
                    return;
                }
                num_planes = buf.length;
                std::cout << "buf.length : " << buf.length << std::endl;
                std::cout << "buf.m.offset : " << buf.m.offset << std::endl;

                for(int j = 0; j < num_planes; j++){
                    buffers[i].length[j] = buf.m.planes[j].length;
                    std::cout << "buf.m.planes[j].length : " << buf.m.planes[j].length << std::endl;
                    buffers[i].start[j] = mmap(NULL, buf.m.planes[j].length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.planes[j].m.mem_offset);
                    if(MAP_FAILED == buffers[i].start[j]){
                        std::cout << "mmap error" << std::endl;
                    }
                    std::cout << "buffers[i].start[j] : " << buffers[i].start[j] << std::endl;

                }
            }
        }

        //5.5 QBUF Request
        {
            for (int i = 0; i < reqbuf.count; ++i) {
                struct v4l2_buffer buf;
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = i;
                struct v4l2_plane planes[FMT_NUM_PLANES];
                buf.m.planes = planes;
                buf.length = FMT_NUM_PLANES;
                if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                    std::cerr << "VIDIOC_QBUF" << std::endl;
            }
        }

        cout << "pcam 5.5 end" << endl;

        // 6. Start Streaming
        {
            struct 	v4l2_buffer buf;
            memset(&(buf), 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            if(-1 == xioctl(fd, VIDIOC_STREAMON, &buf.type))
            {
                std::cout << "Fail to start Capture" << std::endl;
                return;
            }
        }

        std::vector<uint8_t> vec = std::vector<uint8_t>(WIDTH*HEIGHT*2);

        std_msgs::msg::UInt8MultiArray::SharedPtr camdatatemp(new std_msgs::msg::UInt8MultiArray);
        camdatatemp->data = vec;

        camdata = camdatatemp;




        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = 0;
        buf.m.planes = planes;
        buf.length = FMT_NUM_PLANES;
    }
    void Pcam::reset() {
        if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &buf.type)) {
            std::cout << "VIDIOC_STREAMOFF" << std::endl;
        }

        for(int i = 0; i < 3; i++){
            if(munmap(buffers[i].start[0], 480 * 640 * 2) != 0){
                cerr << "pcam munmap failed" << endl;
            }else{
                cout << "pcam munmap success" << endl;
            }
        }
        close(fd);

        setInit();
        cout << "--------------------pcam reset-------------------" << endl;
    }
} // namespace autorace

CLASS_LOADER_REGISTER_CLASS(pcam::Pcam, rclcpp::Node)