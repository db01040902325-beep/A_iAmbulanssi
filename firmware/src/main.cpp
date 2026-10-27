/*
 * ESP32 직렬 명령 인터페이스 초기 검증용.
 * 모터 제어 출력은 의도적으로 구현하지 않았습니다.
 * 명령 형식: CMD,<번호>,<앞쪽 mm>,<뒤쪽 mm>\n
 * 유효하면 ACK,<번호>,PREVIEW 로 답합니다.
 * 하드웨어/모터 드라이버 선정 전 움직이는 것은 위험합니다.
 */
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

static constexpr float LIMIT_MM = 25.0f;
static constexpr unsigned long WATCHDOG_MS = 250;
static constexpr int ENABLE_SWITCH_PIN = 26; // 스위치를 GND에 연결해야 활성화 (비연결 시 차단)
static char line[96];
static size_t used = 0;
static unsigned long lastValidMs = 0;
static bool commandHealthy = false;

static void neutralize() {
  // 출력을 구현할 때, 강제 중립/안전 상태를 하드웨어 별로 정의해야 합니다.
  commandHealthy = false;
}

static void consumeLine() {
  unsigned int seq = 0;
  float front = 0, rear = 0;
  // 잘못된 접두부 / 과도한 스트로크 / NaN 명령 차단
  if (sscanf(line, "CMD,%u,%f,%f", &seq, &front, &rear) != 3 ||
      strncmp(line, "CMD,", 4) != 0 || !isfinite(front) || !isfinite(rear) ||
      fabsf(front) > LIMIT_MM || fabsf(rear) > LIMIT_MM) {
    neutralize();
    Serial.println("ERR,FORMAT_OR_LIMIT");
    return;
  }
  if (digitalRead(ENABLE_SWITCH_PIN) != LOW) {
    neutralize();
    Serial.println("ERR,SAFETY_SWITCH_OFF");
    return;
  }
  lastValidMs = millis();
  commandHealthy = true;
  // 안전상 여기에서는 실제 모터 드라이버로 명령을 출력하지 않습니다.
  Serial.printf("ACK,%u,PREVIEW\n", seq);
}

void setup() {
  pinMode(ENABLE_SWITCH_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  neutralize();
  Serial.println("READY,PREVIEW_ONLY");
}

void loop() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n') {
      line[used] = 0;
      if (used) consumeLine();
      used = 0;
    } else if (c != '\r') {
      if (used + 1 < sizeof(line)) line[used++] = c;
      else { used = 0; neutralize(); Serial.println("ERR,LINE_TOO_LONG"); }
    }
  }
  if (commandHealthy && millis() - lastValidMs > WATCHDOG_MS) {
    neutralize();
    Serial.println("ERR,WATCHDOG");
  }
}
