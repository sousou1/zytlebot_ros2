#include "webcam/webcam_component.hpp"

#include <class_loader/register_macro.hpp>
#include <chrono>
#include <memory>


namespace webcam {

    Webcam::Webcam()
            : Node("webcam") {
        image_pub_ = create_publisher<sensor_msgs::msg::Image>("/webcam/image_raw", 1);

        set_Init();

        timer_ = create_wall_timer(100ms, std::bind(&Webcam::get_image, this));
    }

    void Webcam::get_image() {
        t1 = std::chrono::system_clock::now();
        // 7. Capture Image
        // 8. Store Image in OpenCV Data Type
        if (CbFlag) {
            cout << "usb Cb fail" << endl;
        } else {
            CbFlag = true;
            cout << "usb Cb start" << endl;
            {
                struct v4l2_buffer buf;
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                fd_set fds;
                FD_ZERO(&fds);
                FD_SET(fd, &fds);
                struct timeval tv = {0};
                tv.tv_sec = 2;
                int r = select(fd+1, &fds, NULL, NULL, &tv);

                if(-1 == r){
                    std::cout << "Waiting for Frame" << std::endl;
                    return;
                }

                memset(&(buf), 0, sizeof(buf));
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;

                if(-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
                {
                    std::cout << "USB Retrieving Frame" << std::endl;
                    reset();
                    return;
                }

                cv::Mat frame(h, w, CV_8UC3);
                auto msg = std::make_shared<sensor_msgs::msg::Image>();

                memcpy(frame.data, buffers[buf.index], 640 * 480 * 2);

                convert_frame_to_message(frame, 1, msg);

                image_pub_->publish(msg);

                // Connect buffer to queue for next capture.
                if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) {
                    std::cout << "VIDIOC_QBUF" << std::endl;
                }

            }
            CbFlag = false;
            cout << "CbFlag end" << endl;
        }

        t2 = std::chrono::system_clock::now();
    }

    void Webcam::reset() {
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &buf.type)) {
            std::cout << "VIDIOC_STREAMOFF" << std::endl;
        }

        for(int i = 0; i < got_buffer_num; i++){
            if(munmap(buffers[i], 480 * 640 * 2) != 0){
                cerr << "munmap failed" << endl;
            }else{
                cout << "munmap success" << endl;
            }
        }
        close(fd);

        cout << "restart  USB!!! " << endl;
        set_Init();
    }

    void Webcam::set_Init() {
        CbFlag = false;
        fd = open("/dev/video0", O_RDWR, 0);
        if (fd == -1)
        {
            std::cout << "Failed to open video device." << std::endl;
            return;
        }

        // 2. Querying video capabilities.
        memset(&caps, 0, sizeof(caps));
        if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &caps))
        {
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

            fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            fmt.fmt.pix.width = 640;
            fmt.fmt.pix.height = 480;
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
            fmt.fmt.pix.field = V4L2_FIELD_NONE;

            if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
            {
                std::cout << "Failed to set pixel format." << std::endl;
                return;
            }
        }

        // 4. Request Buffer
        {
            struct v4l2_requestbuffers req;
            memset(&(req), 0, sizeof(req));
            req.count = REQUEST_BUFFER_NUM;
            req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            req.memory = V4L2_MEMORY_MMAP;

            if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
            {
                std::cout << "Failed to request buffer." << std::endl;
                return;
            }
            cout << "we could get " << req.count  << " buffers. " << endl;
            got_buffer_num = req.count;
        }

        // 5. Query Buffer
        {
            for(int bufferindex = 0; bufferindex < got_buffer_num; bufferindex++){
                struct 	v4l2_buffer buf;
                memset(&(buf), 0, sizeof(buf));
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = bufferindex;
                if(-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
                {
                    std::cout << "Failed to query buffer." << std::endl;
                    return;
                }

                std::cout << "buf.length : " << buf.length << std::endl;
                std::cout << "buf.m.offset : " << buf.m.offset << std::endl;

                buffers[bufferindex] = (unsigned char*)mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
                if (MAP_FAILED == buffers[bufferindex])
                    cerr << "mmap" << endl;
            }
        }


        // 5.5 QBUF Request
        {
            for (int i = 0; i < got_buffer_num; ++i) {
                struct v4l2_buffer buf;
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = i;
                if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                    cerr << "VIDIOC_QBUF" << endl;
            }
        }

        cout << "5.5 end" << endl;
        // 6. Start Streaming
        {
            struct 	v4l2_buffer buf;
            memset(&(buf), 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if(-1 == xioctl(fd, VIDIOC_STREAMON, &buf.type))
            {
                std::cout << "Start Capture" << std::endl;
                return;
            }
        }



        std::vector<uint8_t> vec = std::vector<uint8_t>(WIDTH*HEIGHT*2);
    }

    void Webcam::convert_frame_to_message(
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

    std::string Webcam::mat_type2encoding(int mat_type)
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
} // namespace autorace

CLASS_LOADER_REGISTER_CLASS(webcam::Webcam, rclcpp::Node)