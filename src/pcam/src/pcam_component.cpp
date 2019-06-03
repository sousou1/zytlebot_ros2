#include "pcam/pcam_component.hpp"

#include <class_loader/register_macro.hpp>
#include <chrono>
#include <memory>


namespace pcam {

    Pcam::Pcam()
            : Node("pcam") {
        rc = v4l2init(w, h, V4L2_PIX_FMT_RGB24);
        if (rc < 0) {
            fprintf(stderr, "v4l2init = %d\n", rc);
        }
        image_pub_ = create_publisher<sensor_msgs::msg::Image>("/pcam/image_raw", 1);

        // TODO 時間設定
        timer_ = create_wall_timer(1s, std::bind(&Pcam::get_image, this));
    }

    void Pcam::get_image() {
        cv::Mat frame(h, w, CV_8UC3);
        auto msg = std::make_shared<sensor_msgs::msg::Image>();
        rc = v4l2grab(&buf);
        if (rc < 0) {
            fprintf(stderr, "v4l2grab = %d\n", rc);
        }
        frame.data = buf;
        convert_frame_to_message(frame, 1, msg);

        image_pub_->publish(msg);
    }


    int Pcam::v4l2init(int w, int h, __u32 pixelformat)
    {
        int rc;
        struct v4l2_format fmt;
        struct v4l2_requestbuffers req;
        struct v4l2_buffer buf;
        enum v4l2_buf_type type;

        Pcam::v4l2_fd = open("/dev/video0", O_RDWR);
        if (v4l2_fd < 0) {
            fprintf(stderr, "open = %d, errno = %d\n", v4l2_fd, errno);
            return -1;
        }

        std::cout << v4l2_fd << std::endl;

        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = w;
        fmt.fmt.pix.height = h;
        fmt.fmt.pix.pixelformat = pixelformat;
        fmt.fmt.pix.field = V4L2_FIELD_ANY;
        rc = xioctl(v4l2_fd, VIDIOC_S_FMT, &fmt);
        if (rc < 0) {
            fprintf(stderr, "VIDIOC_S_FMT: errno = %d\n", errno);
            return -1;
        }

        memset(&req, 0, sizeof(req));
        req.count = NUM_BUFFER;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        rc = xioctl(v4l2_fd, VIDIOC_REQBUFS, &req);
        if (rc < 0) {
            fprintf(stderr, "VIDIOC_REQBUFS: errno = %d\n", errno);
            return -1;
        }

        memset(&buf, 0, sizeof(buf));
        buf.memory = V4L2_MEMORY_MMAP;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        for (int i = 0; i < NUM_BUFFER; i++) {
            buf.index = i;
            rc = xioctl(v4l2_fd, VIDIOC_QUERYBUF, &buf);
            if (rc < 0) {
                fprintf(stderr, "VIDIOC_QUERYBUF: errno = %d\n", errno);
                return -1;
            }
            Pcam::v4l2_user_frame[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, v4l2_fd, buf.m.offset);
            if (!v4l2_user_frame[i] || v4l2_user_frame[i] == (void *)-1) {
                fprintf(stderr, "mmap: errno = %d\n", errno);
                return -1;
            }
            rc = xioctl(v4l2_fd, VIDIOC_QBUF, &buf);
            if (rc < 0) {
                fprintf(stderr, "VIDIOC_QBUF: errno = %d\n", errno);
                return -1;
            }
        }

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        rc = xioctl(v4l2_fd, VIDIOC_STREAMON, &type);
        if (rc < 0) {
            fprintf(stderr, "VIDIOC_STREAMON: errno = %d\n", errno);
            return -1;
        }

        return 0;
    }

    int Pcam::v4l2end(void)
    {
        int rc;
        enum v4l2_buf_type type;

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        rc = xioctl(v4l2_fd, VIDIOC_STREAMOFF, &type);
        if (rc < 0) {
            fprintf(stderr, "VIDIOC_STREAMOFF: errno = %d\n", errno);
            return -1;
        }

        close(v4l2_fd);

        return 0;
    }

    int Pcam::v4l2grab(unsigned char **frame)
    {
        int rc;
        struct v4l2_buffer buf;
        fd_set fds;
        struct timeval tv;

        memset(&buf, 0, sizeof(buf));
        buf.memory = V4L2_MEMORY_MMAP;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        std::cout << v4l2_fd << std::endl;

        FD_ZERO(&fds);
        FD_SET(v4l2_fd, &fds);
        tv.tv_sec = 2;
        tv.tv_usec = 0;
        select(v4l2_fd + 1, &fds, NULL, NULL, &tv);
        if (FD_ISSET(v4l2_fd, &fds)) {
            rc = xioctl(v4l2_fd, VIDIOC_DQBUF, &buf);
            if (rc < 0) {
                fprintf(stderr, "VIDIOC_DQBUF: errno = %d\n", errno);
                return -1;
            }
            if (buf.index < NUM_BUFFER) {
                *frame = (unsigned char *)v4l2_user_frame[buf.index];
                return buf.index;
            } else {
                fprintf(stderr, "VIDIOC_DQBUF: buf.index = %d\n", errno);
                return -1;
            }
        } else {
            fprintf(stderr, "select: errno = %d\n", errno);
            return -1;
        }
    }

    int Pcam::v4l2release(int buf_idx)
    {
        int rc;
        struct v4l2_buffer buf;

        memset(&buf, 0, sizeof(buf));
        buf.memory = V4L2_MEMORY_MMAP;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (buf_idx < NUM_BUFFER) {
            buf.index = buf_idx;
            rc = xioctl(v4l2_fd, VIDIOC_QBUF, &buf);
            if (rc < 0) {
                fprintf(stderr, "VIDIOC_QBUF: errno = %d\n", errno);
                return -1;
            }
        }

        return 0;
    }

    void Pcam::convert_frame_to_message(
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

    std::string Pcam::mat_type2encoding(int mat_type)
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

    int Pcam::xioctl(int fd, int request, void *arg){
        int rc;
        do rc = ioctl(fd, request, arg);
        while (-1 == rc && EINTR == errno);
        return rc;
    }

} // namespace pcam

CLASS_LOADER_REGISTER_CLASS(pcam::Pcam, rclcpp::Node)