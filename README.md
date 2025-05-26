
# Gesture-Controlled Robot System with ROS2, Vue Dashboard, and YOLO-based Conveyor Coordination

## 📝 프로젝트 개요

**목표**  
MediaPipe 기반 손 제스처와 Web UI, 음성(GPT), YOLO 객체 인식을 통합하여 Dobot Magician을 ROS2로 제어하는 실시간 로봇 제어 시스템 구현

**기간**  
4일

---

## 🧱 시스템 구조 개요

### 데이터 흐름 (End-to-End)

```
[손 제스처, 음성, 버튼 입력]
      ↓
[Vue 대시보드 (WebSocket)]
      ↓
[gesture_bridge_node.py (ROS2 중계)]
  ├─ 좌표 입력 → /hand_position → hand_position_controller.py
  └─ 명령 코드 → /gesture_cmd → gesture_ptp_move.py
      ↓
[Dobot Magician / Suction]
      ↓
[객체 인식 (YOLO + Realsense)]
      ↓
[컨베이어 분류 동작 (integration.py)]
      ↓
[Vue 대시보드 실시간 상태 시각화]
```

---
## 🧭 시스템 구성도

![시스템 구성도](./assets/system-architecture.png)
## 🔧 주요 기술 구성 요소

| 항목 | 기술 스택 | 비고 |
|------|-----------|------|
| 손 제스처 인식 | MediaPipe + OpenCV | 실시간 index finger 좌표 추출 |
| 객체 인식 | YOLOv5 + HSV 기반 색상 분류 | ROI 기반 색/형태 분류 |
| 입력 통합 | Vue WebSocket | 음성, 버튼, 좌표 송신 |
| 음성 명령 해석 | Web Speech API + GPT-4 API | 자연어 → 명령 코드 변환 |
| 로봇 제어 | ROS2 + Dobot SDK + ActionClient | /gesture_cmd, /hand_position 처리 |
| 컨베이어 제어 | Raspberry Pi GPIO | Python + Threading 기반 |
| 실시간 대시보드 | Vue3 + Chart.js | 이동 좌표/입력 비율 시각화 |

---

## 🧠 명령 매핑 테이블

| 명령 코드 | 의미 | 동작 |
|------------|------|------|
| `1` | Z 아래 | Dobot Z축 하강 |
| `2` | Z 위 | Dobot Z축 상승 |
| `3` | Suction OFF | 진공 해제 |
| `4` | Suction ON | 진공 흡착 |
| `7` | 리셋 | 초기 위치 복귀 |

---

## 🗂 ROS2 노드 구성

| 노드명 | 기능 |
|--------|------|
| `gesture_bridge_node.py` | TCP/WebSocket 수신 → ROS2 publish (`/gesture_cmd`, `/hand_position`) |
| `gesture_ptp_move.py` | `/gesture_cmd` 구독, Z축/Suction 동작 수행 |
| `hand_position_controller.py` | `/hand_position` 구독 후 XY 이동 수행 |
| `realsense_yolo_sorter.py` | YOLO + 색상분류 후 객체 라벨 전송 |
| `integration.py` | 소켓 수신 → 서보 및 컨베이어 GPIO 제어 |

---

## 🖥️ Vue 대시보드 기능

| 항목 | 설명 |
|------|------|
| 명령 입력 | 버튼 or 음성 인식 (Web Speech API + GPT) |
| 실시간 로그 | 최신 5개 로그 표시 |
| 좌표 시각화 | ScatterChart (Chart.js 기반) |
| 입력 통계 | DoughnutChart (음성/버튼 비율) |
| WebSocket 연결 상태 표시 | ROS/CAM/MIC 상태 표시 |

---

## 📁 디렉토리 구조 예시

```
ssafy-pjt-main/
├── gesture-socket/
│   ├── realsense_hand_publisher.py
│   ├── test.py
├── ros2_ws/
│   └── src/
│       ├── gesture_bridge_node.py
│       ├── gesture_ptp_move.py
│       ├── hand_position_controller.py
│       ├── integration.py
│       ├── realsense_yolo_sorter.py
├── web-dashboard/
│   ├── Dashboard.vue
│   ├── ScatterChart.vue
│   ├── DoughnutChart.vue
```

---

## ✅ 프로젝트 주요 변경 사항 요약

- 초기 Flask UI → **Vue + WebSocket 기반 대시보드**로 완전 전환
- **제스처 직접 처리 코드(test.py)** → WebSocket 중계로 통합
- **음성 명령**은 GPT API를 통해 정제된 명령으로 전환
- **Dobot PTP 제어 방식 개선** → XY는 좌표 기반, Z/Suction은 명령 기반 분리
- **제스처는 입력 비율 통계에서 제외됨 (2025.05 기준)**

---

## 📌 주의사항 및 향후 발전 방향

- 좌표 → Dobot 이동은 워크스페이스 유효성 검사가 필수
- GPT 응답이 복합 명령(예: "2,2,4")일 경우 딜레이 포함 반복 처리됨
- 향후 YOLO 색상 분류 정확도 향상 및 라벨 다양화 고려 가능
- WebSocket 통신이 중단될 경우 Vue에서 명확한 상태표시 필요
