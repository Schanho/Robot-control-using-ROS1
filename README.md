# Robot-control-using-ROS1

Ubuntu 20.04와 ROS Noetic 기반의 로봇 제어용 ROS1 프로젝트입니다. 다양한 로봇 제어 시나리오에 맞게 모듈형 코드와 구성 파일을 포함하고 있어, 하드웨어 구성 및 로봇 작업에 맞게 유연하게 활용할 수 있습니다.

## 개요

이 레포지토리는 ROS1 Noetic에서 로봇을 제어하기 위한 코드, 설정 파일, 문서를 포함하고 있습니다. 기본적인 설정 및 구성을 제공하여, 다양한 로봇 플랫폼과 센서와의 통합이 가능합니다.

## 주요 기능

- **호환성**: Ubuntu 20.04 LTS 및 ROS Noetic에 최적화되어 있습니다.
- **모듈형 코드베이스**: 다양한 로봇 플랫폼과 센서 통합을 위한 유연한 구조 제공
- **간편한 설정**: ROS Noetic 환경 설정 및 워크스페이스 구축에 대한 단계별 가이드 제공
- **맞춤형 제어 모듈**: 로봇의 움직임 및 센서 통합을 위한 스크립트와 구성 포함

## 요구 사항

- **운영체제**: Ubuntu 20.04 LTS
- **ROS 배포판**: ROS Noetic Ninjemys
- **패키지**:
  - [기본 ROS 패키지](http://wiki.ros.org/noetic/Installation/Ubuntu)
  - `requirements.txt`에 나열된 추가 의존 패키지

---

## **📜 프로젝트 배경**
### 필요성
- **학교 내 안전과 효율적인 물품 전달**  
  - 시간적 여유가 부족하거나 긴급한 상황에서 물품을 대신 전달할 수 있는 시스템 구축이 필요합니다.
- **새로운 도입 및 활용**  
  - 실내 공간에 첨단 기술을 도입하여 다양한 방식으로 활용이 가능하며, 최신 기술을 체험할 수 있는 기회를 제공합니다.
- **장기 비용 절감**  
  - 인건비 절감과 더불어 서비스 효율성을 증대하여 장기적인 비용 절감 효과를 기대할 수 있습니다.

### 기존 해결책의 문제점
- 반복적이고 단순한 작업에 대한 인력 비용이 지속적으로 증가하고 있습니다.
- 실외 자율주행 로봇은 날씨나 외부 환경의 영향을 크게 받습니다.

---

## **🛠 System Design**
### 시스템 요구 사항
- **SBC** : Jetson Orin Nano Development Kit
- **Controller** : OpenCR 1.0 (MCU: 32-bit ARM Cortex®-M7 with FPU (216MHz, 462DMIPS))
- **Camera** : Intel Realsense D435 or D455
- **Actuator** : Robotis Dynamixel XM430-W350-T, XM540-W270-R
- **OS** : Ubuntu 20.04 (Jetpack SDK 5.1.1)
- **Programming Language** : Python (>= 3.8.0)
- **Dependency Packages** : 
  - Opencv 4.10.0 (Cmake 설치 필요)
  - librealsense SDK
  - ROS Noetic

---

## **🤖 로봇 하드웨어 구성**
- 다양한 하드웨어 모듈과 센서들이 조합되어 로봇의 제어와 자율주행을 수행합니다.  

![로봇 구조](https://github.com/user-attachments/assets/18b73962-f9ee-4911-bbff-e7cfdc65dd8b)
![로봇 이미지](https://github.com/user-attachments/assets/907fc4cf-6839-46f7-b7f8-67d89534f16a)

---

## **📌 TurtleBot3 패키지 사용 안내**

본 프로젝트에서는 로봇 제어에 Robotis에서 제공하는 [TurtleBot3 패키지](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)를 기반으로 구축되었습니다. 패키지의 용량이 큰 관계로, 모든 파일을 포함하지 않고 주요 스크립트와 설정 파일만 업로드되었습니다.  
자세한 사용법 및 설치 방법은 Robotis 공식 매뉴얼을 참조하시기 바랍니다.

---

## **🧩 코드 구성**
1. **임베디드 시스템 - 로봇 제어**  
   - Jetson Orin Nano와 OpenCR을 활용하여 로봇의 이동 및 제어를 수행합니다.
   - Dynamixel 모터와 LiDAR 센서를 통합하여 자율주행 및 물체 인식을 가능하게 합니다.
  
2. **인공지능 모델 -  Grounding DINO 1.6**  
   - **YOLOv8** : 객체 탐지와 경로 예측을 위한 딥러닝 모델을 사용합니다.
   - **Grounding DINO 1.6** : 제로 샷 학습 모델을 활용하여 엘리베이터 문 상태 인식을 수행합니다.

---

## **📚 Case Study**
- 표윤석, 조한철, 정려운, 임태훈 (2017), “ROS 개발환경 구축”, 「ROS 로봇 프로그래밍」, 루비페이퍼
- Alexey Spizhevoy, Aleksandr Rynnikov (2018), “Multiple View Geometry”, 「Opencv 3 Computer Vision with Python Cookbook」, Packt Publishing

---

## **📈 프로젝트 성과**
1. **2024 SW중심대학 우수작품 경진대회** 참가
2. **대한전자공학회 논문 제출**

---

## **🔚 결론**
- 이번 프로젝트를 통해 학교 내 자율주행 로봇의 필요성과 이를 구현하기 위한 기술적인 접근을 논의했습니다. 다양한 실험과 개선을 통해, 실제 환경에서 효율적으로 작동할 수 있는 로봇 시스템을 설계하였습니다.
