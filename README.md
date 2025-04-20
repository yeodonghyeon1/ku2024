# KU2024 ROS2 프로젝트 🤖

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-brightgreen)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.8%2B-blue)](https://www.python.org/)

## 프로젝트 소개 📝

KU2024는 ROS2 기반의 로봇 제어 및 센서 데이터 처리 프로젝트입니다. Arduino와 연동하여 하드웨어 제어가 가능하며, 다양한 센서 데이터를 처리하고 시각화할 수 있습니다.

## 주요 기능 ⚡

- Arduino 하드웨어 통합
- ROS2 노드 기반 센서 데이터 처리
- RViz를 통한 데이터 시각화
- 파라미터 기반 시스템 설정
- 커스텀 메시지 타입 지원

## 프로젝트 구조 📂

```
ku2024/
├── Arduino/          # Arduino 관련 코드
├── data/            # 데이터 파일
├── launch/          # ROS2 런치 파일
├── msg/             # 커스텀 메시지 정의
├── params/          # 파라미터 설정 파일
├── rviz/            # RViz 설정 파일
└── src/             # 메인 소스 코드
```

## 설치 방법 💻

1. ROS2 Humble 설치
```bash
# ROS2 Humble 설치 (Ubuntu 22.04)
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
```

2. 의존성 패키지 설치
```bash
pip install -r requirements.txt
```

3. 프로젝트 빌드
```bash
colcon build
source install/setup.bash
```

## 실행 방법 🚀

1. Arduino 코드 업로드
```bash
cd Arduino/211
# Arduino IDE를 통해 코드 업로드
```

2. ROS2 노드 실행
```bash
ros2 launch ku2024 main.launch.py
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