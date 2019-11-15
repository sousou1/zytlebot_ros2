#include "yolo/yolo_component.hpp"

#include <class_loader/register_macro.hpp>

namespace yolo {

    Yolo::Yolo()
            : Node("yolo") {
        cout << "start Yolo" << endl;

        idxInputImage = 0;  // frame index of input video
        idxShowImage = 0;   // next frame index to be displayed
        bReading = true;   // flag of reding input frameint idxInputImage = 0;  // frame index of input video
        idxShowImage = 0;   // next frame index to be displayed
        bReading = true;   // flag of reding input frame

        is_video= 0 ;

        dpuOpen();

        cap.open(0);
        if(!cap.isOpened()){
            cout << "failed" << endl;
        }

      object_search_result_ = create_publisher<std_msgs::msg::String>("/object_search_result", 1);

      timer_ = create_wall_timer(100ms, std::bind(&Yolo::image_cb, this));

    }

    void Yolo::image_cb() {
      cout << "searching object!!!!" << endl;

      cv::Mat frame;
      cap >> frame; // get a new frame from camera

      DPUKernel *kernel = dpuLoadKernel("yolo");
      DPUTask* task = dpuCreateTask(kernel, 0);


    }

    /**
 * @brief Feed input frame into DPU for process
 *
 * @param task - pointer to DPU Task for YOLO-v3 network
 * @param frame - pointer to input frame
 * @param mean - mean value for YOLO-v3 network
 *
 * @return none
 */
    void Yolo::setInputImageForYOLO(DPUTask* task, const Mat& frame, float* mean) {
      Mat img_copy;
      int height = dpuGetInputTensorHeight(task, INPUT_NODE);
      int width = dpuGetInputTensorWidth(task, INPUT_NODE);
      int size = dpuGetInputTensorSize(task, INPUT_NODE);
      int8_t* data = dpuGetInputTensorAddress(task, INPUT_NODE);

      image img_new = load_image_cv(frame);
      image img_yolo = letterbox_image(img_new, width, height);

      vector<float> bb(size);
      for(int b = 0; b < height; ++b) {
          for(int c = 0; c < width; ++c) {
              for(int a = 0; a < 3; ++a) {
                  bb[b*width*3 + c*3 + a] = img_yolo.data[a*height*width + b*width + c];
                }
            }
        }

      float scale = dpuGetInputTensorScale(task, INPUT_NODE);

      for(int i = 0; i < size; ++i) {
          data[i] = int(bb.data()[i]*scale);
          if(data[i] < 0) data[i] = 127;
        }

      free_image(img_new);
      free_image(img_yolo);
    }

/**
 * @brief Thread entry for reading image frame from the input video file
 *
 * @param fileName - pointer to video file name
 *
 * @return none
 */
    void Yolo::readFrame(const char *fileName) {
      static int loop = 3;
      VideoCapture video;
      string videoFile = fileName;
      start_time = chrono::system_clock::now();

      while (loop>0) {
          loop--;
          if (!video.open(videoFile)) {
              cout<<"Fail to open specified video file:" << videoFile << endl;
              exit(-1);
            }

          while (true) {
              usleep(20000);
              Mat img;
              if (queueInput.size() < 30) {
                  if (!video.read(img) ) {
                      break;
                    }

                  mtxQueueInput.lock();
                  queueInput.push(make_pair(idxInputImage++, img));
                  mtxQueueInput.unlock();
                } else {
                  usleep(10);
                }
            }

          video.release();
        }

      exit(0);
    }

/**
 * @brief Thread entry for displaying image frames
 *
 * @param  none
 * @return none
 *
 */
    void Yolo::displayFrame() {
      Mat frame;

      while (true) {
          mtxQueueShow.lock();

          if (queueShow.empty()) {
              mtxQueueShow.unlock();
              usleep(10);
            } else if (idxShowImage == queueShow.top().first) {
              auto show_time = chrono::system_clock::now();
              stringstream buffer;
              frame = queueShow.top().second;

              auto dura = (duration_cast<microseconds>(show_time - start_time)).count();
              buffer << fixed << setprecision(1)
                     << (float)queueShow.top().first / (dura / 1000000.f);
              string a = buffer.str() + " FPS";
              cv::putText(frame, a, cv::Point(10, 15), 1, 1, cv::Scalar{240, 240, 240},1);
              cv::imshow("Yolo@Xilinx DPU", frame);

              idxShowImage++;
              queueShow.pop();
              mtxQueueShow.unlock();
              if (waitKey(1) == 'q') {
                  bReading = false;
                  exit(0);
                }
            } else {
              mtxQueueShow.unlock();
            }
        }
    }

/**
 * @brief Post process after the runing of DPU for YOLO-v3 network
 *
 * @param task - pointer to DPU task for running YOLO-v3
 * @param frame
 * @param sWidth
 * @param sHeight
 *
 * @return none
 */
    void Yolo::postProcess(DPUTask* task, Mat& frame, int sWidth, int sHeight){

      /*output nodes of YOLO-v3 */
      const vector<string> outputs_node = {"layer81_conv", "layer93_conv", "layer105_conv"};

      vector<vector<float>> boxes;
      for(size_t i = 0; i < outputs_node.size(); i++){
          string output_node = outputs_node[i];
          int channel = dpuGetOutputTensorChannel(task, output_node.c_str());
          int width = dpuGetOutputTensorWidth(task, output_node.c_str());
          int height = dpuGetOutputTensorHeight(task, output_node.c_str());

          int sizeOut = dpuGetOutputTensorSize(task, output_node.c_str());
          int8_t* dpuOut = dpuGetOutputTensorAddress(task, output_node.c_str());
          float scale = dpuGetOutputTensorScale(task, output_node.c_str());
          vector<float> result(sizeOut);
          boxes.reserve(sizeOut);

          /* Store every output node results */
          get_output(dpuOut, sizeOut, scale, channel, height, width, result);

          /* Store the object detection frames as coordinate information  */
          detect(boxes, result, channel, height, width, i, sHeight, sWidth);
        }

      /* Restore the correct coordinate frame of the original image */
      correct_region_boxes(boxes, boxes.size(), frame.cols, frame.rows, sWidth, sHeight);

      /* Apply the computation for NMS */
      cout << "boxes size: " << boxes.size() << endl;
      vector<vector<float>> res = applyNMS(boxes, classificationCnt, NMS_THRESHOLD);

      float h = frame.rows;
      float w = frame.cols;
      for(size_t i = 0; i < res.size(); ++i) {
          float xmin = (res[i][0] - res[i][2]/2.0) * w + 1.0;
          float ymin = (res[i][1] - res[i][3]/2.0) * h + 1.0;
          float xmax = (res[i][0] + res[i][2]/2.0) * w + 1.0;
          float ymax = (res[i][1] + res[i][3]/2.0) * h + 1.0;

          cout<<res[i][res[i][4] + 6]<<" ";
          cout<<xmin<<" "<<ymin<<" "<<xmax<<" "<<ymax<<endl;


          if(res[i][res[i][4] + 6] > CONF ) {
              int type = res[i][4];

              if (type==0) {
                  rectangle(frame, cvPoint(xmin, ymin), cvPoint(xmax, ymax), Scalar(0, 0, 255), 6, 1, 0);
                }
              else if (type==1) {
                  rectangle(frame, cvPoint(xmin, ymin), cvPoint(xmax, ymax), Scalar(255, 0, 0), 6, 1, 0);
                }
              else {
                  rectangle(frame, cvPoint(xmin, ymin), cvPoint(xmax, ymax), Scalar(0 ,255, 255), 6, 1, 0);
                }
            }
        }
    }

/**
 * @brief Thread entry for running YOLO-v3 network on DPU for acceleration
 *
 * @param task - pointer to DPU task for running YOLO-v3
 * @param img
 *
 * @return none
 */
    void Yolo::runYOLO(DPUTask* task, Mat& img) {
      /* mean values for YOLO-v3 */
      float mean[3] = {0.0f, 0.0f, 0.0f};

      int height = dpuGetInputTensorHeight(task, INPUT_NODE);
      int width = dpuGetInputTensorWidth(task, INPUT_NODE);


      /* feed input frame into DPU Task with mean value */
      setInputImageForYOLO(task, img, mean);

      /* invoke the running of DPU for YOLO-v3 */
      dpuRunTask(task);
      postProcess(task, img, width, height);


    }


/**
 * @brief Thread entry for running YOLO-v3 network on DPU for acceleration
 *
 * @param task - pointer to DPU task for running YOLO-v3
 *
 * @return none
 */
    void Yolo::runYOLO_video(DPUTask* task) {
      /* mean values for YOLO-v3 */
      float mean[3] = {0.0f, 0.0f, 0.0f};

      int height = dpuGetInputTensorHeight(task, INPUT_NODE);
      int width = dpuGetInputTensorWidth(task, INPUT_NODE);

      while (true) {
          pair<int, Mat> pairIndexImage;

          mtxQueueInput.lock();
          if (queueInput.empty()) {
              mtxQueueInput.unlock();
              if (bReading)
                {
                  continue;
                } else {
                  break;
                }
            } else {
              /* get an input frame from input frames queue */
              pairIndexImage = queueInput.front();
              queueInput.pop();
              mtxQueueInput.unlock();
            }
          vector<vector<float>> res;
          /* feed input frame into DPU Task with mean value */
          setInputImageForYOLO(task, pairIndexImage.second, mean);

          /* invoke the running of DPU for YOLO-v3 */
          dpuRunTask(task);

          postProcess(task, pairIndexImage.second, width, height);
          mtxQueueShow.lock();

          /* push the image into display frame queue */
          queueShow.push(pairIndexImage);
          mtxQueueShow.unlock();
      }
    }
} // namespace yolo

CLASS_LOADER_REGISTER_CLASS(yolo::Yolo, rclcpp::Node)