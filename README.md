# AI 노면 예측 기반 구급차 침대 2자유도 제어 — 코드 v0.1

## 현재 구현한 범위

- **실행 가능:** 침대의 상하(heave)·앞뒤 회전(pitch) 2자유도 기구학 계산
- **실행 가능:** Python으로 `제어 없음 / 피드백 제어 / 이상적 미래 입력 제어` 세 조건 비교
- **실행 가능:** 액추에이터 변위/속도 제한, 비정상 센서값 차단
- **실행 가능:** 뉴턴 제2법칙 및 피치 회전 방정식을 이용한 앞/뒤 필요 수직력 계산 (`bedcontrol/dynamics.py`)
- **실행 가능:** 시뮬레이션 CSV 및 (선택) 그래프 저장
- **실행 가능:** 노트북 USB 카메라 영상 수집 도구 (OpenCV 설치 필요)
- **구조 준비:** ESP32 C++에서 시리얼 명령 검증과 안전 스위치·워치독 **미리보기**
- **아직 미구현:** 실제 학습한 딥러닝 노면 예측기, 센서 실측, 모터의 실제 이동, 진동 발생장치

> 정확히 구별: 시뮬레이션의 `preview`는 노면에 관한 **이상적인 미래 데이터**를 시뮬레이터가 제공하는 실험 조건입니다. 실제 AI가 카메라 영상을 읽고 미래 충격을 예측했다는 뜻이 아닙니다.
>
> 이 소프트웨어는 **축소 모형 연구·실습 전용**입니다. 의료용 또는 실제 환자가 누운 구급차 침대의 운용에 사용하지 마세요.

## 1. 노트북에서 실행하기 (Windows / Ubuntu)

Python 3.10 이상 권장. 프로젝트 폴더에서 아래 명령을 실행합니다.

```bash
python main.py
python -m unittest discover -s tests -v
```

그래프도 저장하려면:

```bash
python -m pip install matplotlib
python main.py --plot
```

결과는 `results/passive.csv`, `results/feedback.csv`, `results/preview.csv`, `results/heave.png`, `results/pitch.png`에 저장됩니다.

## 2. 카메라 영상 수집 (AI 학습용 원본만)

```bash
python -m pip install opencv-python
python tools/camera_record.py --seconds 10
```

`recordings/road_video.mp4` 및 `recordings/frames.csv` 생성. **현재 도로 충격 예측은 수행하지 않습니다.**
실제 AI 학습에는 영상별 노면 높이(또는 충격 목표값), 차량 속도, 충격 발생 시각, 센서 동기화 데이터가 필요합니다.

## 3. 노트북 없이 작동시키는 최종 구조

```text
  전방 카메라  ───────>  라즈베리파이 (Python, AI 추론)
                                  |
  침대 IMU·위치센서 --> ESP32 (C++, 안전검사·모터 제어)
                                  |
                            앞/뒤 구동부
```

라즈베리파이(또는 동등한 SBC)는 노트북의 Python 처리 역할을 대신합니다.
ESP32만으로도 단순 제어는 가능하지만, 영상 딥러닝까지 실행하려면 모델 경량화 및 별도 가속기 성능 검토가 필요합니다.
최종 제품의 제어 루프는 ESP32에서 작동하도록 분리하고, AI가 동작하지 않아도 안전 상태를 유지하도록 해야 합니다.

## 4. 기계 설계 기준

- `support_spacing_m = 0.20` (앞/뒤 지지점 사이 거리 200 mm)
- `stroke_limit_m = 0.025` (중립 기준 앞/뒤 ±25 mm)
- 관절/가이드는 **상하+피치만** 허용하도록 설계. 사진의 4개 지지기둥은 각 독립 구동이 아니라 **앞쪽 한 쌍과 뒤쪽 한 쌍을 동기 제어**하는 2자유도 구조로 구현할 수 있습니다.
- 앞/뒤 높이 변위: `front = heave + L*pitch/2`, `rear = heave - L*pitch/2` (피치 rad, 소각도 가정).
- 실제 모터 회전각/PWM과 상대 높이(mm) 간 관계는 **링크 길이·리드스크루 피치·감속기 선정 후 교정**해야 합니다.
- MPU6050만으로 절대적인 침대 높이를 정확히 알 수는 없습니다. 위치센서 또는 엔코더가 추가로 필요합니다.

## 5. ESP32 초기 통신 시험 (출력 없는 안전 미리보기)

`firmware/platformio.ini`는 PlatformIO + ESP32 보드용입니다.

```bash
cd firmware
pio run
# 보드가 연결되었을 때만
pio run -t upload
pio device monitor -b 115200
```

GPIO26을 GND에 연결하는 별도 **실험용 활성화 스위치**가 켜져 있을 때만 명령 형식을 검증합니다. 이 스위치는 실제 구동의 비상정지를 대체하지 않습니다.

전송 예: `CMD,1,2.0,-2.0` → `ACK,1,PREVIEW`

노트북의 시리얼 통신 점검 예시: `python -m pip install pyserial` 후 `python tools/serial_preview.py --port COM5` (Windows) 또는 `/dev/ttyUSB0` (Linux).

**모터는 실제로 움직이지 않습니다.** 서보/스텝모터/리니어액추에이터 모델, 드라이버, 전압, 행정, 피드백 센서가 확정된 후 하드웨어 구동 계층을 추가해야 합니다.

## 동역학 방정식 검증

`bedcontrol/dynamics.py`는 다음 식에서 전후 지지력(N)을 계산합니다.

```
F_front + F_rear - m*g = m*a_z
(L/2)*(F_front - F_rear) = I_pitch*alpha_pitch
```

```python
from bedcontrol.dynamics import LoadParameters, required_vertical_forces
from bedcontrol.geometry import Geometry
front_N, rear_N = required_vertical_forces(Geometry(), LoadParameters(), 0.0, 0.0)
print(front_N, rear_N)
```

**주의:** 이는 수직 작용력의 간이 분배 계산입니다. 실제 모터 토크는 링크의 기계적 이득과 기구각, 마찰, 구조 강도까지 포함하여 별도로 계산해야 합니다.

## 6. 3명 역할 제안

- 기구 팀: 3D 프린팅 프레임, 전/후 지지대, 수직/회전 조인트
- 제어 팀: ESP32 통신·센서·모터 출력 및 안전 제한
- AI 팀: 카메라 데이터 수집, 노면 상태 인식 모델 학습, 동역학 검증

## 7. 다음 개발 단계

1. 침대 상판 크기, 실험 하중, 전후 지지점 거리, 구동기 모델 확정
2. 실제 센서/모터 드라이버 연결 및 ESP32 단독 제어 구현
3. 베이스 진동 발생 모형과 위치·가속도 데이터 동기 수집
4. Python 모델 학습 (도로 영상 → 예상 충격 시점·크기), 가짜 미래 입력 대신 실제 예측값 사용
5. Raspberry Pi에서 영상/AI 프로세스 실행하고 노트북 분리 테스트
6. 실패·통신 단절·센서 고장 시 기계식 하드스톱 및 안전 상태 시험

## 한계

- 이 모델은 **이동 기구학과 액추에이터 1차 지연**만 가정합니다. 질량, 강성, 감쇠계수, 로드셀, 모터 전기적 특성, 지면 충격의 실제 전달함수는 반영하지 않았습니다.
- 시뮬레이션 결과는 개발 코드 비교용이며 실물 구급차 진동 감소를 입증하지 않습니다.
