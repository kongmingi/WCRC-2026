//////////////  모터 통신 속도(Baud Rate) 변경 도구
//
//  BlackBerry 로봇의 Dynamixel 모터 8개(바퀴 ID 1~4, 팔 ID 5~8)의 통신 속도를 바꾼다.
//  모터의 실제 속도는 실행할 스케치의 통신 선택과 같아야 한다.
//  Mission은 MissionConfig.h, poseMon은 자체 Motor.h의 USE_DXL_1M_BPS로 선택하며 motorAdjust는 별도 설정이다.
//
//  사용법: 아래 두 매크로 중 "하나만" 주석을 풀고 올린다.
//          둘 다 주석이면 모터 설정을 바꾸지 않고 안내만 출력한다(실수로 올려도 안전).
//          올린 뒤 시리얼 모니터(115200)에서 두 속도에서 각각 응답하는 모터 목록을 확인하고 SW1 버튼 또는 시리얼 입력으로 시작한다.
//          소리: 입력 대기 시작 = 삑 1회, 최종 8개 응답 확인 = 삐비빅, 오류 = 삐뽀~ 삐뽀~ 1묶음.
//          이미 8개 모두 목표 속도이면 삑 삑 1묶음. 재확인 중 같은 알림은 반복하지 않는다.
//          초기 검색 오류는 같은 대기 중 반복해서 울리지 않으며, 최종 변경 결과는 별도로 알린다.
//
//  일부 모터만 바뀐 상태(두 속도가 섞인 상태)도 처리한다: 바꾸기 전 속도(BAUD_FROM)에서 응답하는 모터만 바꾸고,
//  이미 목표 속도(BAUD_TO)인 모터는 건너뛴다. 되돌릴 때는 반대 방향 매크로로 같은 방식으로 실행한다.

#define CHANGE_MOTOR_BAUDRATE_57600_TO_1M    // 57600 -> 1,000,000
//#define CHANGE_MOTOR_BAUDRATE_1M_TO_57600    // 1,000,000 -> 57600 (Mission/Motor.h 기본값으로 되돌릴 때)

#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial1
const int DXL_DIR_PIN = 2;                 // DIR PIN (Mission/Motor.h와 같음)
const float DXL_PROTOCOL_VERSION = 2.0;

#define SW1_PIN      41                    // 시작 확인 버튼 (Mission/Devices.h와 같음)
#define BUZZER_PIN   5                     // 부저 (Mission/Devices.h와 같음)

// 컨트롤 테이블 BAUD_RATE(8번지) 값 (XL430 기준): 0= 9600, 1= 57600, 2= 115200, 3= 1M
#define MOTOR_BAUDRATE_57600    (int32_t) 1
#define MOTOR_BAUDRATE_1M       (int32_t) 3

const uint8_t DXL_CNT = 8;
const uint8_t DXL_IDS[DXL_CNT] = {1, 2, 3, 4, 5, 6, 7, 8};
const uint8_t ALL_FOUND = (uint8_t)((1u << DXL_CNT) - 1);   // 8개 모두 응답했을 때의 비트마스크

#if defined(CHANGE_MOTOR_BAUDRATE_57600_TO_1M) && defined(CHANGE_MOTOR_BAUDRATE_1M_TO_57600)
  #error "CHANGE_MOTOR_BAUDRATE_... 매크로는 하나만 정의할 것"
#endif

#if defined(CHANGE_MOTOR_BAUDRATE_57600_TO_1M)
  const unsigned long BAUD_FROM     = 57600;
  const unsigned long BAUD_TO       = 1000000;
  const int32_t       BAUD_TO_VALUE = MOTOR_BAUDRATE_1M;
#elif defined(CHANGE_MOTOR_BAUDRATE_1M_TO_57600)
  const unsigned long BAUD_FROM     = 1000000;
  const unsigned long BAUD_TO       = 57600;
  const int32_t       BAUD_TO_VALUE = MOTOR_BAUDRATE_57600;
#else
  #define NO_CHANGE_SELECTED
#endif

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// 컨트롤 테이블 아이템의 이름을 사용하기 위해 이 네임스페이스가 필요함
using namespace ControlTableItem;


//////////////  준비·완료·오류 알림 (모터 통신 중에는 호출하지 않는다)

void PlayBaudrateReadySound()
{
  tone(BUZZER_PIN, 784, 150);  // 삑: SW1 또는 시리얼 입력을 기다릴 준비 완료
}

void PlayBaudrateAlreadySetSound()
{
  for (uint8_t i = 0; i < 2; i++) {
    tone(BUZZER_PIN, 784, 150);  // 이미 목표 속도: 삑 삑
    delay(250);
  }
  noTone(BUZZER_PIN);
}

void PlayBaudrateCompleteSound()
{
  const unsigned int notes[3] = { 784, 988, 1319 };
  const unsigned int durations[3] = { 100, 100, 180 };
  for (uint8_t i = 0; i < 3; i++) {
    tone(BUZZER_PIN, notes[i], durations[i]);
    delay(durations[i] + 60);
  }
  noTone(BUZZER_PIN);
}

void PlayBaudrateErrorSound()
{
  for (uint8_t i = 0; i < 2; i++) {  // 삐뽀~ 삐뽀~: 높은음·낮은음 두 쌍만 재생
    tone(BUZZER_PIN, 880, 250);
    delay(250);
    tone(BUZZER_PIN, 440, 400);
    delay(500);
  }
  noTone(BUZZER_PIN);
}


//////////////  메인 프로그램

void setup()
{
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);

#ifdef NO_CHANGE_SELECTED
  Serial.println(F("\n[changeMotorBaudrate] 변경 방향이 선택되지 않았습니다. 모터 설정은 바꾸지 않았습니다."));
  Serial.println(F("스케치 맨 위의 CHANGE_MOTOR_BAUDRATE_... 매크로 중 하나만 주석을 풀고 다시 올리세요."));
  PlayBaudrateErrorSound();
  while (1) { delay(1000); }
#else
  Serial.print(F("\n[changeMotorBaudrate] "));
  Serial.print(BAUD_FROM);
  Serial.print(F(" -> "));
  Serial.println(BAUD_TO);
  Serial.println(F("Mission/Motor.h의 DXL_BAUDRATE와 같은 값이어야 Mission이 모터를 찾습니다."));

  // 두 속도에서 각각 어느 모터가 응답하는지 확인. 바꿀 대상은 BAUD_FROM에서 응답하는 모터만.
  uint8_t atFrom, atTo;
  bool scanErrorAnnounced = false;  // 초기 검색 중 같은 오류음은 한 번만
  bool alreadySetAnnounced = false; // 이미 목표 속도인 경우 두 번의 삑을 한 묶음만

  while (1) {
    atFrom = ScanServos(BAUD_FROM);
    atTo   = ScanServos(BAUD_TO);

    Serial.print(F("  at ")); Serial.print(BAUD_FROM); Serial.print(F(" : ")); PrintIdList(atFrom);
    Serial.print(F("  at ")); Serial.print(BAUD_TO);   Serial.print(F(" : ")); PrintIdList(atTo);
    Serial.print(F("  no response at either : "));  PrintIdList((uint8_t)(~(atFrom | atTo) & ALL_FOUND));

    if (atFrom != 0) break;

    if (atTo == ALL_FOUND) {
      Serial.print(F("모든 모터가 이미 "));
      Serial.print(BAUD_TO);
      Serial.println(F(" bps입니다. 바꿀 것이 없습니다. 5초 후 다시 확인..."));
      if (!alreadySetAnnounced) {
        PlayBaudrateAlreadySetSound();
        alreadySetAnnounced = true;
      }
    } else {
      Serial.println(F("바꿀 모터(BAUD_FROM에서 응답)가 없습니다. 전원, 케이블, 매크로 방향을 확인하세요. 5초 후 다시 확인..."));
      if (!scanErrorAnnounced) {
        PlayBaudrateErrorSound();
        scanErrorAnnounced = true;
      }
    }
    delay(5000);  // 변경 대상이 없으면 5초 대기 후 다시 검색
  }

  if ((atFrom | atTo) != ALL_FOUND) {
    Serial.println(F("  주의: 두 속도 모두에서 응답하지 않는 모터가 있습니다. 전원·케이블을 확인한 뒤 진행하세요."));
    if (!scanErrorAnnounced)
      PlayBaudrateErrorSound();
  }

  Serial.print(F("변경 대상: ")); PrintIdList(atFrom);
  Serial.println(F("시작하려면 SW1 버튼을 누르거나 시리얼로 아무 글자를 보내세요..."));
  PlayBaudrateReadySound();
  WaitButtonOrInputChar();
  noTone(BUZZER_PIN);  // 즉시 입력한 경우에도 실제 모터 설정 전에는 준비음을 끝낸다.
  Serial.println(F("start setting"));

  // 속도 변경. BAUD_RATE는 EEPROM 영역이므로 토크를 끈 상태에서 쓴다. BAUD_FROM에서 응답한 모터만 대상.
  dxl.begin(BAUD_FROM);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (uint8_t i = 0; i < DXL_CNT; i++) {
    if (!(atFrom & (1 << i))) continue;

    dxl.writeControlTableItem(TORQUE_ENABLE, DXL_IDS[i], 0);
    bool ok = dxl.writeControlTableItem(BAUD_RATE, DXL_IDS[i], BAUD_TO_VALUE);

    Serial.print(F("  ID "));
    Serial.print(DXL_IDS[i]);
    Serial.println(ok ? F(": write OK") : F(": write FAIL"));
  }

  // 목표 속도에서 8개가 모두 응답하는지 확인
  delay(200);
  uint8_t after = ScanServos(BAUD_TO);

  Serial.print(F("\n새 속도 ")); Serial.print(BAUD_TO); Serial.print(F(" 에서 응답: ")); PrintIdList(after);

  if (after == ALL_FOUND) {
    Serial.println(F("complete setting. 모터 8개가 모두 새 속도로 응답합니다."));
    PlayBaudrateCompleteSound();  // 개별 write 결과가 아니라 최종 목표 속도 응답으로 완료 판정
  } else {
    Serial.print(F("아직 새 속도가 아닌 모터: ")); PrintIdList((uint8_t)(~after & ALL_FOUND));
    Serial.println(F("모터 전원을 다시 켜고 같은 방향으로 한 번 더 올리면 남은 모터만 바꿉니다."));
    PlayBaudrateErrorSound();
  }
#endif
}

void loop() {}


/*
 * 지정한 속도로 포트를 열고 ID 1~8 중 응답하는 모터를 비트마스크로 반환 (bit0= ID1 ... bit7= ID8)
 */
uint8_t ScanServos(unsigned long baud)
{
  dxl.begin(baud);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  uint8_t ids_pinged[16] = {0,};
  uint8_t count = dxl.ping(DXL_BROADCAST_ID, ids_pinged, sizeof(ids_pinged), 100);
  uint8_t mask = 0;

  for (uint8_t j = 0; j < count; j++) {
    for (uint8_t i = 0; i < DXL_CNT; i++) {
      if (ids_pinged[j] == DXL_IDS[i]) { mask |= (1 << i); break; }
    }
  }
  return mask;
}

/*
 * 비트마스크의 ID 목록을 한 줄로 출력. 없으면 "-"
 */
void PrintIdList(uint8_t mask)
{
  if (mask == 0) { Serial.println(F("-")); return; }

  for (uint8_t i = 0; i < DXL_CNT; i++) {
    if (mask & (1 << i)) { Serial.print(DXL_IDS[i]); Serial.print(' '); }
  }
  Serial.println();
}

/*
 * SW1 버튼이 눌리거나 시리얼로 글자가 들어올 때까지 대기
 */
void WaitButtonOrInputChar()
{
  while (1) {
    if (digitalRead(SW1_PIN) == 0) {      // 버튼 눌림
      delay(20);
      return;
    }
    if (Serial.available()) {             // 시리얼 모니터에 문자 입력
      while (Serial.read() >= 0) { delay(2); }
      return;
    }
    delay(10);
  }
}
