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
            return -1;
        }

        image_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/pcam/image_array", 1);

        // TODO 時間設定
        timer_ = create_wall_timer(1s, std::bind(&Pcam::get_image, this));
    }

    void Pcam::get_image() {
        rc = v4l2grab(&buf);
        if (rc < 0) {
            fprintf(stderr, "v4l2grab = %d\n", rc);
            return -1;
        }
        frame.data = buf;

        image_pub_->publish(frame);
    }


    int Pcam::v4l2init(int w, int h, __u32 pixelformat)
    {
        int rc;
        struct v4l2_format fmt;
        struct v4l2_requestbuffers req;
        struct v4l2_buffer buf;
        enum v4l2_buf_type type;

        v4l2_fd = open("/dev/video0", O_RDWR);
        if (v4l2_fd < 0) {
            fprintf(stderr, "open = %d, errno = %d\n", v4l2_fd, errno);
            return -1;
        }

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
            v4l2_user_frame[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, v4l2_fd, buf.m.offset);
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
} // namespace pcam

CLASS_LOADER_REGISTER_CLASS(pcam::Pcam, rclcpp::Node)