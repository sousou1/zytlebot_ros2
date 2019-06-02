// Copyright 2018 Geoffrey Biggs, AIST
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include <autonomous/autonomous_component.hpp>
#include <pcam/pcam_component.hpp>
#include <red_detect/red_detect_component.hpp>
#include <webcam/webcam_component.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // タイマーコールバック、トピックコールバック等を行うexecutor
  rclcpp::executors::SingleThreadedExecutor exec;

  // Greeterコンポーネントノードのインスタンスを作成しexecutorに登録する
  auto autonomous = std::make_shared<autonomous::Autonomous>();
  exec.add_node(autonomous);
  // Displayerコンポーネントノードのインスタンスを作成しexecutorに登録する
  auto pcam = std::make_shared<pcam::Pcam>();
  exec.add_node(pcam);
  auto red_detect = std::make_shared<red_detect::RedDetect>();
  exec.add_node(red_detect);
  auto webcam = std::make_shared<webcam::Webcam>();
  exec.add_node(webcam);

  // Ctrl-Cが押されるまでexecutorを実行する
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
