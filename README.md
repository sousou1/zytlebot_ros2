# zytlebot_ros2

Autonomous Mobile Robot Software for HEART 2019(https://wwp.shizuoka.ac.jp/fpt-design-contest/heart2019/)

## Platform
Robot body：TurtleBot3

Board：Ultra96, ZyboZ7 , Raspberry Pi 3(without Red light detection
)

ROS2 distribution:Crystal

Requires Ubuntu PC and Raspberry Pi 3 for setup, but not required for execution.

## Setup
### ボードに対応したOSのインストール
こちらを参考に、Ubuntu18.04をインストールし、ssh接続できることを確認してください。

### ROS2 and TurtleBot3のためのsetup
Ultra96にsshで接続し、ROS2 Crystalをインストール(https://index.ros.org/doc/ros2/Installation/)してください。（base packageだけでいいです。）

そのまま続けて、TurtleBot3のsetupもこちらのページを参考に行います。
(http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/#ros2)

TurtleBot3を動かすためには、以下の3つを行う必要があります。

1.1.3 Install TurtleBot3 ROS2 Packages

1.2 SBC setup

1.3 OpenCR setup


#### 1.1.3 Install TurtleBot3 ROS2 Packages

現在、Turtlebot3のリポジトリがcrystalからDashingへの移行途中なので、ビルドが失敗する可能性があります。その場合は

1.1.3 [Install TurtleBot3 ROS2 Packages]の

``wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos``

↓

``wget https://raw.githubusercontent.com/sousou1/turtlebot3/ros2/turtlebot3.repos``

に置き換えてください。

また、OpenCRと接続する際はROS2の環境を読み込んでいると失敗するので、
``echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
``
は行わず、ROS2を起動する前に
``source ~/turtlebot3_ws/install/setup.bash``
を行ってください

#### SBC setup
続けてUltra96で行います。

``source ~/turtlebot3_ws/install/setup.bash``

を**していないこと**を確認して、Raspberry Pi 3と同様に行います。

#### OpenCR setup
Ultra96ではなく、**Raspberry Pi 3**で行ってください。
(Ultra96やPCでは成功したように見えて動きません)

## ZytleBotパッケージのビルド

```
git clone https://github.com/sousou1/zytlebot_ros2.git
cd zytlebot_ros2
colcon build --parallel-workers 1
```
parallel-workersオプションがない場合、メモリが足らずに失敗することがあります。
## 実行方法
Ultra96とsshしたターミナルを2つ立ち上げてください
### ターミナル1(OpenCRとの接続)
cd ~/turtlebot3 && MicroXRCEAgent serial /dev/ttyACM0
### ターミナル2(ROS2 packageの起動)
