#define PIN_ENA 8
#define PIN_DIR 2
#define PIN_PUL 5

unsigned long step_count = 0;
String state = "Ready";

void setup() {
  Serial.begin(115200);

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_PUL, OUTPUT);

  digitalWrite(PIN_ENA, LOW); // 모터 활성화
  digitalWrite(PIN_DIR, LOW); // 방향 설정
}

void loop() {
  // 시리얼로부터 명령 읽기
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    step_count = command.toInt(); // 수신한 명령을 스텝 수로 변환
    if (step_count > 0) {
      state = "Running";
    }
  }

  // 스텝 모터 제어
  if (state == "Running" && step_count > 0) {
    digitalWrite(PIN_PUL, HIGH);
    delayMicroseconds(1000);
    digitalWrite(PIN_PUL, LOW);
    delayMicroseconds(1000);

    step_count--;
    if (step_count <= 0) {
      state = "Ready";
    }
  }

  // 상태를 시리얼로 발행
  Serial.println(state);
  delay(100); // 상태 발행 주기
}

