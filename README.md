# ROS 2 Humble 기반 SLAM 및 자율주행 연구 프로젝트

> **기본 SLAM 패키지 분석부터 커스텀 토픽 설계 및 실제 RC카 하드웨어 적용까지의 과정**

## 1. 프로젝트 개요
본 프로젝트는 Ubuntu 22.04 및 ROS 2 Humble 환경에서 SLAM(Simultaneous Localization and Mapping)과 Navigation의 원리를 이해하고 구현하는 것을 목표로 합니다. 시뮬레이션 환경에서의 테스트를 거쳐 실제 RC카 로봇에 적용하는 단계별 과정을 포함하고 있습니다.

## 2. 워크스페이스 구성 및 담당 역할

워크스페이스(`turtle_ws`)는 크게 세 가지 단계의 패키지로 구성되어 있습니다.

### ① [turtlebot3] - 표준 환경 분석
* **내용:** TurtleBot3 공식 SLAM 및 Navigation2 패키지를 활용한 환경 구축.
* **목적:** 표준화된 자율주행 시스템의 구조(TF, Sensor Data, Costmap)를 분석하고 시뮬레이션 환경에서 기준 성능을 파악함.

### ② [custom_slam] - 사용자 정의 알고리즘 구현 (핵심 역량)
* **내용:** 기존의 UI 레이아웃은 유지하되, 내부 **토픽(Topic) 구조 및 메시지 통신을 직접 설계**.
* **특징:** 단순히 패키지를 실행하는 것을 넘어, 로봇의 구동 로직에 필요한 토픽을 직접 구성하여 시스템의 데이터 흐름을 최적화하고 제어 알고리즘을 직접 구현해 본 과정입니다.

### ③ [capstone_robot] - 실물 하드웨어 적용 (진행 중)
* **내용:** 실제 **RC카(Real Hardware)**를 기반으로 한 SLAM 시스템 구축 프로젝트.
* **현황:** 시뮬레이션을 넘어 실제 환경에서의 Lidar 데이터 처리 및 모터 제어를 진행 중이며, 캡스톤 디자인 프로젝트로 발전시키고 있습니다.

## 3. 개발 환경
* **OS:** Ubuntu 22.04 LTS
* **Platform:** ROS 2 Humble
* **Hardware:** Turtlebot3 (Simulation), Custom RC Car
* **Language:** C++ / Python
