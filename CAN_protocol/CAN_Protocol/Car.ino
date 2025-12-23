#include <Servo.h>

// ================== 서보 / 초음파 ==================
Servo EduServo;
int trigPin = 13;
int echoPin = 12;

// ================== 모터 핀 ==================
int RightMotor_E_pin = 5;
int LeftMotor_E_pin  = 6;
int RightMotor_1_pin = 8;
int RightMotor_2_pin = 9;
int LeftMotor_3_pin  = 10;
int LeftMotor_4_pin  = 11;

// ================== 라인 센서 핀 ==================
int L_Line = A5;
int C_Line = A4;
int R_Line = A3;

// ================== 속도 설정 ==================
int L_MotorSpeed = 0;
int R_MotorSpeed = 0;

const int NORMAL_SPEED  = 130;
const int SLOW_SPEED    = 100;
const int TURN_SPEED    = 150;

// ================== 초음파 관련 상수 ==================
const int SAFE_DISTANCE_CM = 25;   // 회피 판단 기준
const int TOO_CLOSE_CM     = 17;   // 즉시 후진
const int MAX_DISTANCE_CM  = 400;

// ================== 그림자 튐 방지 필터 ==================
static uint8_t lineHit = 0;
const uint8_t LINEHIT_MAX = 3;
const uint8_t LINE_DETECT_LEVEL = 1; // 그림자 심하면 2로

// ================== 라인 탐색 상태 ==================
int lastTurn = 0; // -1: 마지막 좌, +1: 마지막 우, 0: 없음
unsigned long lostStart = 0;
const unsigned long SWITCH_DIR_MS = 1200;
const int SEARCH_SPIN_MS = 55;
const int SEARCH_FWD_MS  = 70;

// ================== 서보 스캔 각도 ==================
const int SERVO_CENTER = 90;
const int SERVO_LEFT   = 140;
const int SERVO_RIGHT  = 40;
const int CLEAR_MARGIN = 5;

// ================== 장애물 회피 상태머신 ==================
enum RunState {
  STATE_LINE = 0,     // 정상 라인/탐색
  STATE_AVOID_BACK,   // 후진으로 거리 확보
  STATE_AVOID_SCAN,   // 좌/정면/우 스캔
  STATE_AVOID_TURN,   // 안전한 방향으로 회전
  STATE_AVOID_GO      // 안전한 방향으로 "조건부 전진"(계속 초음파 확인)
};

RunState runState = STATE_LINE;

// 회피 동작 파라미터(환경에 맞게 조절)
const int AVOID_BACK_MS   = 260;  // 후진 시간
const int AVOID_TURN_MS   = 320;  // 방향 회전 시간(좌/우)
const int AVOID_GO_MS     = 220;  // 한 번 전진할 때의 단위 시간(짧게 끊어서 재스캔)
const int AVOID_GO_SPEED  = 120;  // 회피 전진 속도
const int AVOID_BACK_SPEED= 140;  // 회피 후진 속도

unsigned long stateUntil = 0;
int avoidTurnDir = 0; // -1 좌, +1 우, 0 직진

// ================== 함수 선언 ==================
void line_follow(int L, int C, int R);
void line_search();
void motor_role(int R_motor, int L_motor);
int  Ultrasonic();
int  UltrasonicFront();
int  UltrasonicAt(int angle);
void stop_motors();
void emergency_back();
void obstacle_state_step(int dist, bool lineDetected);

void spin_left_ms(int ms);
void spin_right_ms(int ms);

// ================== SETUP ==================
void setup() {
  EduServo.attach(2);
  EduServo.write(SERVO_CENTER);
  delay(300);

  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);

  pinMode(RightMotor_E_pin, OUTPUT);
  pinMode(LeftMotor_E_pin,  OUTPUT);
  pinMode(RightMotor_1_pin, OUTPUT);
  pinMode(RightMotor_2_pin, OUTPUT);
  pinMode(LeftMotor_3_pin,  OUTPUT);
  pinMode(LeftMotor_4_pin,  OUTPUT);

  pinMode(L_Line, INPUT);
  pinMode(C_Line, INPUT);
  pinMode(R_Line, INPUT);

  Serial.begin(9600);
  Serial.println("=== START: Line + Ultrasonic Avoid (Back->Scan->Turn->Go with recheck) ===");
}

// ================== LOOP ==================
void loop() {
  int dist = UltrasonicFront();
  if (dist <= 0 || dist > MAX_DISTANCE_CM) dist = MAX_DISTANCE_CM;

  int L = digitalRead(L_Line);
  int C = digitalRead(C_Line);
  int R = digitalRead(R_Line);

  bool anyLine = (L == 0 || C == 0 || R == 0);
  if (anyLine) {
    if (lineHit < LINEHIT_MAX) lineHit++;
  } else {
    if (lineHit > 0) lineHit--;
  }
  bool lineDetected = (lineHit >= LINE_DETECT_LEVEL);

  // TOO_CLOSE는 최우선
  if (dist > 0 && dist < TOO_CLOSE_CM) {
    emergency_back();
    runState = STATE_AVOID_SCAN;  // 후진 직후 바로 스캔으로 넘어가도 되지만, 안전하게 스캔부터 다시 시작
    stateUntil = 0;
    delay(20);
    return;
  }

  // 회피 상태머신 동작
  if (runState != STATE_LINE) {
    obstacle_state_step(dist, lineDetected);
    delay(10);
    return;
  }

  // -------- 정상 상태(라인/탐색) --------

  // 라인이 잡히면 라인 팔로우
  if (lineDetected) {
    lostStart = 0;
    line_follow(L, C, R);
    delay(10);
    return;
  }

  // 라인 없고 장애물 가까움 -> 회피 상태 시작
  if (dist > 0 && dist < SAFE_DISTANCE_CM) {
    stop_motors();
    runState = STATE_AVOID_BACK;
    stateUntil = millis() + AVOID_BACK_MS;
    Serial.println("AVOID: 시작 -> BACK");
    delay(20);
    return;
  }

  // 라인 없고 장애물도 멀면 탐색
  line_search();
  delay(10);
}

// ================== 장애물 회피 상태머신 ==================
void obstacle_state_step(int dist, bool lineDetected) {
  // 라인 재획득 시 언제든 복귀
  if (lineDetected) {
    stop_motors();
    runState = STATE_LINE;
    stateUntil = 0;
    Serial.println("AVOID: 라인 재획득 -> LINE 복귀");
    return;
  }

  switch (runState) {
    case STATE_AVOID_BACK: {
      // 일정 시간 후진하여 공간 확보
      L_MotorSpeed = AVOID_BACK_SPEED;
      R_MotorSpeed = AVOID_BACK_SPEED;
      motor_role(LOW, LOW); // 후진 (배선에 따라 반대면 여기만 바꾸면 됨)

      if (millis() >= stateUntil) {
        stop_motors();
        runState = STATE_AVOID_SCAN;
        stateUntil = 0;
        Serial.println("AVOID: BACK 완료 -> SCAN");
      }
    } break;

    case STATE_AVOID_SCAN: {
      // 좌/정면/우 스캔 후 가장 안전한 방향 선택
      stop_motors();
      delay(80);

      int dFront = UltrasonicAt(SERVO_CENTER);
      int dLeft  = UltrasonicAt(SERVO_LEFT);
      int dRight = UltrasonicAt(SERVO_RIGHT);

      EduServo.write(SERVO_CENTER);
      delay(40);

      Serial.print("SCAN: L="); Serial.print(dLeft);
      Serial.print(" F="); Serial.print(dFront);
      Serial.print(" R="); Serial.println(dRight);

      bool frontOK = (dFront >= SAFE_DISTANCE_CM);
      bool leftOK  = (dLeft  >= SAFE_DISTANCE_CM);
      bool rightOK = (dRight >= SAFE_DISTANCE_CM);

      // 선택 규칙:
      // 1) 가장 큰 거리 방향 우선
      // 2) 모두 막혔으면 다시 BACK로(좀 더 후진/각도 바꿔 재시도)
      int best = dFront;
      avoidTurnDir = 0; // 0 = 직진(회전 없이)

      if (dLeft > best + CLEAR_MARGIN) {
        best = dLeft;
        avoidTurnDir = -1;
      }
      if (dRight > best + CLEAR_MARGIN) {
        best = dRight;
        avoidTurnDir = +1;
      }

      if (!frontOK && !leftOK && !rightOK) {
        // 완전 막힘: 조금 더 후진하고 다시 스캔
        runState = STATE_AVOID_BACK;
        stateUntil = millis() + (AVOID_BACK_MS + 120);
        Serial.println("AVOID: 전방/좌/우 모두 막힘 -> BACK 재시도");
        return;
      }

      // frontOK이고 좌/우가 크게 우세하지 않으면 직진 선택(회전 최소화)
      runState = STATE_AVOID_TURN;
      stateUntil = millis() + ((avoidTurnDir == 0) ? 0 : AVOID_TURN_MS);
      Serial.print("AVOID: SCAN 결과 -> TURN dir=");
      Serial.println(avoidTurnDir);
    } break;

    case STATE_AVOID_TURN: {
      // 선택된 방향으로 회전(또는 회전 생략)
      if (avoidTurnDir == 0) {
        stop_motors();
        runState = STATE_AVOID_GO;
        stateUntil = millis() + AVOID_GO_MS;
        Serial.println("AVOID: TURN 생략 -> GO");
        return;
      }

      if (avoidTurnDir < 0) {
        // 좌회전
        L_MotorSpeed = TURN_SPEED;
        R_MotorSpeed = TURN_SPEED;
        motor_role(HIGH, LOW); // (R 전진, L 후진) = 좌 스핀
      } else {
        // 우회전
        L_MotorSpeed = TURN_SPEED;
        R_MotorSpeed = TURN_SPEED;
        motor_role(LOW, HIGH); // (R 후진, L 전진) = 우 스핀
      }

      if (millis() >= stateUntil) {
        stop_motors();
        runState = STATE_AVOID_GO;
        stateUntil = millis() + AVOID_GO_MS;
        Serial.println("AVOID: TURN 완료 -> GO");
      }
    } break;

    case STATE_AVOID_GO: {
      // "막무가내 전진"이 아니라, 전진 중에도 초음파 확인해서
      // 안전하면 짧게 전진, 위험하면 즉시 SCAN으로 복귀
      if (dist > 0 && dist < SAFE_DISTANCE_CM) {
        stop_motors();
        runState = STATE_AVOID_SCAN;
        stateUntil = 0;
        Serial.println("AVOID: GO 중 장애물 재근접 -> SCAN");
        return;
      }

      // 안전하면 짧게 전진하고 다시 SCAN으로 재평가(환경 변화 대응)
      L_MotorSpeed = AVOID_GO_SPEED;
      R_MotorSpeed = AVOID_GO_SPEED;
      motor_role(HIGH, HIGH);

      if (millis() >= stateUntil) {
        stop_motors();
        runState = STATE_AVOID_SCAN;
        stateUntil = 0;
        Serial.println("AVOID: GO 단위 전진 완료 -> SCAN(재평가)");
      }
    } break;

    default:
      runState = STATE_LINE;
      break;
  }
}

// ================== 라인트레이서 (패턴 고정) ==================
void line_follow(int L, int C, int R) {
  // 확정 규칙:
  // 1 0 0 : 좌회전
  // 0 0 1 : 우회전
  // 0 1 0 : 직진

  if (L == 1 && C == 0 && R == 0) {          // 100 : 좌
    lastTurn = -1;
    spin_left_ms(120);
  }
  else if (L == 0 && C == 0 && R == 1) {     // 001 : 우
    lastTurn = +1;
    spin_right_ms(120);
  }
  else if (L == 0 && C == 1 && R == 0) {     // 010 : 직진
    L_MotorSpeed = NORMAL_SPEED;
    R_MotorSpeed = NORMAL_SPEED;
    motor_role(HIGH, HIGH);
  }
  else {
    if (C == 0) {
      L_MotorSpeed = NORMAL_SPEED;
      R_MotorSpeed = NORMAL_SPEED;
      motor_role(HIGH, HIGH);
    } else {
      if (lastTurn < 0) spin_left_ms(70);
      else if (lastTurn > 0) spin_right_ms(70);
      else {
        L_MotorSpeed = SLOW_SPEED;
        R_MotorSpeed = SLOW_SPEED;
        motor_role(HIGH, HIGH);
      }
    }
  }
}

// ================== 라인 놓쳤을 때 탐색 ==================
void line_search() {
  if (lostStart == 0) lostStart = millis();

  bool switchDir = (millis() - lostStart > SWITCH_DIR_MS);

  int dir = (lastTurn == 0) ? +1 : lastTurn;
  if (switchDir) dir = -dir;

  if (((millis() - lostStart) % 400) < 60) {
    L_MotorSpeed = SLOW_SPEED;
    R_MotorSpeed = SLOW_SPEED;
    motor_role(HIGH, HIGH);
    delay(SEARCH_FWD_MS);
    stop_motors();
    delay(40);
  }

  if (dir < 0) spin_left_ms(SEARCH_SPIN_MS);
  else         spin_right_ms(SEARCH_SPIN_MS);
}

// ================== 제자리 회전(스핀) ==================
void spin_left_ms(int ms) {
  L_MotorSpeed = TURN_SPEED;
  R_MotorSpeed = TURN_SPEED;
  motor_role(HIGH, LOW);
  delay(ms);
  stop_motors();
}

void spin_right_ms(int ms) {
  L_MotorSpeed = TURN_SPEED;
  R_MotorSpeed = TURN_SPEED;
  motor_role(LOW, HIGH);
  delay(ms);
  stop_motors();
}

// ================== 비상 후진 ==================
void emergency_back() {
  L_MotorSpeed = TURN_SPEED;
  R_MotorSpeed = TURN_SPEED;
  motor_role(LOW, LOW);
  delay(260);
  stop_motors();
  delay(80);
}

// ================== 모터 정지 ==================
void stop_motors() {
  L_MotorSpeed = 0;
  R_MotorSpeed = 0;
  analogWrite(RightMotor_E_pin, 0);
  analogWrite(LeftMotor_E_pin,  0);

  digitalWrite(RightMotor_1_pin, LOW);
  digitalWrite(RightMotor_2_pin, LOW);
  digitalWrite(LeftMotor_3_pin,  LOW);
  digitalWrite(LeftMotor_4_pin,  LOW);
}

// ================== 공용 모터 함수 ==================
void motor_role(int R_motor, int L_motor) {
  digitalWrite(RightMotor_1_pin, R_motor);
  digitalWrite(RightMotor_2_pin, !R_motor);
  digitalWrite(LeftMotor_3_pin,  L_motor);
  digitalWrite(LeftMotor_4_pin,  !L_motor);

  analogWrite(RightMotor_E_pin, R_MotorSpeed);
  analogWrite(LeftMotor_E_pin,  L_MotorSpeed);
}

// ================== 초음파 (cm) ==================
int Ultrasonic() {
  long duration;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return MAX_DISTANCE_CM;

  int distance = duration / 58;
  if (distance > MAX_DISTANCE_CM) distance = MAX_DISTANCE_CM;
  return distance;
}

int UltrasonicFront() {
  EduServo.write(SERVO_CENTER);
  delay(5);
  return Ultrasonic();
}

int UltrasonicAt(int angle) {
  EduServo.write(angle);
  delay(120);
  int d = Ultrasonic();
  if (d <= 0 || d > MAX_DISTANCE_CM) d = MAX_DISTANCE_CM;
  return d;
}
