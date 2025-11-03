# Control Package

ROS 2 컨트롤 패키지로, Bada 프로젝트의 제어 및 추정 알고리즘을 위한 노드들을 포함합니다.

## 개요

이 패키지는 C++로 작성된 여러 제어 관련 노드들을 포함하며, 배의 자율 주행을 위한 제어 및 상태 추정 기능을 담당합니다.

## 노드 설명

현재 계획된 노드들:

1. **Controller Node**: 배의 제어 알고리즘을 실행하는 노드
2. **Estimator Node**: 센서 데이터를 기반으로 배의 상태를 추정하는 노드

## 의존성
```bash
sudo apt update
sudo apt install libgeographic-dev
```

## 빌드 방법

```bash
cd ~/bada2_ws
colcon build --packages-select control
source install/setup.bash
```

## 사용 방법

노드 구현 후 추가 예정입니다.

## 디렉토리 구조

```
control/
├── include/control/    # 헤더 파일 디렉토리
├── src/               # 소스 파일 디렉토리
├── launch/            # 런치 파일 디렉토리
├── config/            # 설정 파일 디렉토리
├── CMakeLists.txt     # 빌드 설정 파일
└── package.xml        # 패키지 메타데이터
```

## 조건
* non-working 조건
   * waypoint empty
   * not mission mode
   * 