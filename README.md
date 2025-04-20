# KU2024 ROS1 프로젝트 🤖

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![ROS](https://img.shields.io/badge/ROS-Melodic-brightgreen)](http://wiki.ros.org/melodic)
[![Python](https://img.shields.io/badge/Python-2.7-blue)](https://www.python.org/)

## 프로젝트 소개 📝

KU2024는 ROS1 Melodic 기반의 로봇 제어 및 센서 데이터 처리 프로젝트입니다. Arduino와 연동하여 하드웨어 제어가 가능하며, 다양한 센서 데이터를 처리하고 시각화할 수 있습니다.

## 주요 기능 ⚡

- Arduino 시리얼 통신 및 하드웨어 제어
- ROS 토픽/서비스 기반 센서 데이터 처리
- RViz를 통한 실시간 데이터 시각화
- 동적 파라미터 재구성
- 사용자 정의 메시지 및 서비스 타입

## 시스템 요구사항 🔧

- Ubuntu 18.04 LTS
- ROS Melodic
- Python 2.7
- Arduino IDE 1.8.x
- pyserial 3.4 이상

## 프로젝트 구조 📂

```
ku2024/
├── Arduino/          # Arduino 스케치 및 라이브러리
│   └── 211/         # 메인 Arduino 코드
├── data/            # 캘리브레이션 및 설정 데이터
├── launch/          # ROS 런치 파일
├── msg/             # 사용자 정의 메시지
│   ├── SensorData.msg
│   └── ControlCommand.msg
├── srv/             # 사용자 정의 서비스
│   └── SetParameter.srv
├── params/          # 파라미터 YAML 파일
├── rviz/            # RViz 설정 파일
└── src/             # Python 소스 코드
    ├── nodes/       # ROS 노드
    ├── lib/         # 유틸리티 라이브러리
    └── tests/       # 단위 테스트
```

## 주요 ROS 토픽 📡

### 센서 토픽
| 토픽 이름 | 타입 | 설명 |
|-----------|------|------|
| `/imu/data` | `sensor_msgs/Imu` | IMU 센서 데이터 |
| `/imu/mag` | `sensor_msgs/MagneticField` | 자기장 센서 데이터 |
| `/heading` | `std_msgs/Float64` | 선박의 헤딩 각도 |
| `/enu_position` | `geometry_msgs/Point` | 선박의 ENU 좌표계 위치 |
| `/pointcloud/scan_data` | `sensor_msgs/LaserScan` | LiDAR 스캔 데이터 |
| `/usb_cam/image_raw` | `sensor_msgs/Image` | 카메라 이미지 |

### 제어 토픽
| 토픽 이름 | 타입 | 설명 |
|-----------|------|------|
| `/thrusterL` | `std_msgs/UInt16` | 좌측 추진기 제어 |
| `/thrusterR` | `std_msgs/UInt16` | 우측 추진기 제어 |
| `/servo` | `std_msgs/UInt16` | 서보 모터 제어 |

### 장애물 및 시각화 토픽
| 토픽 이름 | 타입 | 설명 |
|-----------|------|------|
| `/obstacles` | `ku2023/ObstacleList` | 감지된 장애물 목록 |
| `/visual_rviz` | `visualization_msgs/MarkerArray` | RViz 시각화 마커 |
| `/angles_rviz` | `visualization_msgs/MarkerArray` | 각도 시각화 |
| `/points_rviz` | `visualization_msgs/MarkerArray` | 포인트 시각화 |
| `/goal_rviz` | `visualization_msgs/Marker` | 목표점 시각화 |
| `/traj_rviz` | `visualization_msgs/Marker` | 경로 시각화 |

## 설치 방법 💻

1. ROS Melodic 설치
```bash
# ROS Melodic 설치 (Ubuntu 18.04)
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full
```

2. ROS 환경 설정
```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

3. 의존성 패키지 설치
```bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
```

4. 작업 공간 설정
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/yeodonghyeon1/ku2024.git
cd ..
catkin_make
source devel/setup.bash
```

5. Python 의존성 설치
```bash
pip install -r requirements.txt
```

## Arduino 설정 ⚡

1. Arduino IDE 설치
```bash
sudo apt install arduino
```

2. 시리얼 포트 권한 설정
```bash
sudo usermod -a -G dialout $USER
sudo chmod a+rw /dev/ttyACM0  # Arduino 포트에 따라 변경
```

3. Arduino 코드 업로드
```bash
cd ~/catkin_ws/src/ku2024/Arduino/211
# Arduino IDE를 통해 코드 업로드
```

## 실행 방법 🚀

1. ROS 마스터 실행
```bash
roscore
```

2. Arduino 노드 실행
```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
```

3. 센서 데이터 처리 노드 실행
```bash
roslaunch ku2024 sensor_processing.launch
```

4. 시각화 실행
```bash
roslaunch ku2024 visualization.launch
```

## 주요 코드 예제 💻

1. 센서 데이터 구독 예제
```python
#!/usr/bin/env python2
import rospy
from std_msgs.msg import Float64

def heading_callback(data):
    heading = data.data  # 선박의 헤딩 각도
    rospy.loginfo("Current heading: %f", heading)

def main():
    rospy.init_node('heading_subscriber', anonymous=True)
    rospy.Subscriber('/heading', Float64, heading_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

2. 제어 명령 발행 예제
```python
#!/usr/bin/env python2
import rospy
from std_msgs.msg import UInt16

def control_thrusters():
    thrusterL_pub = rospy.Publisher('/thrusterL', UInt16, queue_size=10)
    thrusterR_pub = rospy.Publisher('/thrusterR', UInt16, queue_size=10)
    rospy.init_node('thruster_controller', anonymous=True)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # 추진기 제어 (0-255)
        thrusterL_pub.publish(128)  # 50% 출력
        thrusterR_pub.publish(128)  # 50% 출력
        rate.sleep()

if __name__ == '__main__':
    try:
        control_thrusters()
    except rospy.ROSInterruptException:
        pass
```

## 문제 해결 🔍

1. 시리얼 포트 연결 오류
```bash
# 시리얼 포트 확인
ls -l /dev/ttyACM*
# 권한 부여
sudo chmod 666 /dev/ttyACM0
```

2. ROS 노드 실행 오류
```bash
# catkin 작업 공간 재빌드
cd ~/catkin_ws
catkin_make clean
catkin_make
source devel/setup.bash
```

## 라이선스 📜

이 프로젝트는 [GNU General Public License v3.0](LICENSE) 라이선스 하에 배포됩니다.

## 기여 방법 🤝

1. 이 저장소를 포크합니다.
2. 새로운 브랜치를 생성합니다: `git checkout -b feature/새기능`
3. 변경사항을 커밋합니다: `git commit -am '새로운 기능 추가'`
4. 브랜치를 푸시합니다: `git push origin feature/새기능`
5. Pull Request를 생성합니다.

## 문의사항 💌

프로젝트에 대한 문의사항이 있으시면 Issues를 통해 문의해주세요.