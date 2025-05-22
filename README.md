# Gesture-Controlled Robot System with ROS2, Web Integration and Conveyor Coordination

## 📝 프로젝트 개요

**목표**  
손 제스처를 인식하여 두봇 매지션(Dobot Magician)을 ROS2 기반으로 제어하고, 컨베이어 벨트 및 웹 인터페이스와 통합하는 로봇 제어 시스템을 구현합니다.

**기간**  
4일

---

## 🧱 시스템 구조 개요

### 데이터 흐름

```
[사용자 손 제스처 인식]  
      ↓  
[Windows]  
  └─ MediaPipe 기반 제스처 분류 (1~4)  
  └─ 웹소켓으로 리눅스에 전송  
      ↓  
[Linux]  
  └─ 웹소켓 수신 노드 (ROS2 Bridge)  
  └─ ROS2 토픽으로 제스처 전달 (/gesture_cmd)  
  └─ 제어 노드에서 두봇 및 컨베이어 동작 명령 발행  
      ↓  
[Dobot Magician / Conveyor]  
  └─ 제스처에 따른 실시간 반응  
      ↓  
[웹 대시보드 (선택)]  
  └─ 로봇 및 컨베이어 상태 시각화  
```

---

## 🔧 주요 기술 구성 요소

| 항목 | 기술 스택 | 비고 |
|------|------------|------|
| 손 제스처 인식 | MediaPipe (Python) | Windows 환경 |
| 제스처 통신 | WebSocket (Python) | TCP 기반, 제스처 코드 전송 (1~4) |
| ROS2 인터페이스 | ROS2 Foxy / Humble | 리눅스 기반 메시지 처리 |
| 로봇 제어 | Dobot SDK + ROS2 | 좌우 이동, 그리퍼 제어 |
| 컨베이어 제어 | GPIO or 릴레이 + ROS2 노드 | ROS2에서 On/Off 제어 |
| 웹 대시보드 (선택) | Flask + JS or ROSBridge | 시스템 상태 시각화 용도 |

---

## ✋ 제스처 매핑 테이블

| 제스처 코드 | 의미 | 동작 |
|--------------|------|------|
| `1` | 주먹 | 그리퍼 닫기 |
| `2` | 손 펼침 | 그리퍼 열기 |
| `3` | 오른손가락 제스처 | 두봇 오른쪽 이동 |
| `4` | 왼손가락 제스처 | 두봇 왼쪽 이동 |

---

## 🗂 ROS2 노드 구성

| 노드명 | 기능 |
|--------|------|
| `gesture_bridge_node` | 웹소켓으로 수신한 제스처를 `/gesture_cmd` 토픽으로 publish |
| `dobot_ptp_move` | `/gesture_cmd`를 구독하여 두봇에 명령 전송 |
| `conveyor_controller_node` | 로봇 동작 시 컨베이어 정지 및 재시작 제어 |
| `state_dashboard_node` (선택) | 현재 상태를 웹 UI로 전달 (ROSBridge or REST API 사용 가능) |

---

## 🗓 개발 일정

| 날짜 | 작업 내용 |
|------|-----------|
| **Day 1** | MediaPipe 제스처 인식 구현 (Windows)<br>웹소켓 전송 모듈 구성 |
| **Day 2** | ROS2 웹소켓 브리지 구현<br>Dobot ROS2 제어 노드 구축 |
| **Day 3** | 컨베이어 ROS2 제어 노드 구축<br>통합 시스템 연결 테스트 |
| **Day 4** | 예외 처리 및 최적화<br>웹 대시보드 구성 (선택)<br>시연 시나리오 구성 및 리허설 |

---

## ✅ 기대 결과

- 손의 간단한 제스처로 두봇의 이동 및 그리퍼 동작 제어 가능  
- 로봇 제어와 컨베이어 제어가 매끄럽게 연동됨  
- 향후 웹 대시보드를 통해 상태 모니터링 및 수동 제어 확장 가능  

---

## 📁 디렉토리 구조 (예시)

```
project/
├── ros2_ws/
│   └── src/
│       ├── gesture_bridge_node/
│       ├── dobot_controller_node/
│       └── conveyor_controller_node/
├── mediapipe_gesture/
│   └── gesture_sender.py  # 웹소켓 송신
├── web_dashboard/         # 선택
│   ├── app.py (Flask)
│   └── static/
```

---

## 📌 주의사항

- MediaPipe는 Windows 환경에서만 실행하며, ROS2는 Linux에서 실행합니다.  
- 두 시스템은 **웹소켓을 통해 연결**되며, 최소한의 지연으로 명령을 전달할 수 있도록 최적화해야 합니다.  
- ROS2에서 Dobot 및 컨베이어를 실시간으로 제어하기 위해, 각 노드는 비동기 방식으로 설계됩니다.
