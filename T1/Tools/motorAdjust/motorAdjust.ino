//////////////  바퀴 모터 속도 비율 조정 도구
//
//  SW2로 시간 주행(false, 삑) / 거리 프로파일 주행(true, 삐삑)을 전환한다. 시작은 시간 주행 모드.
//  시간 주행: 아래 loop()의 기존 속도/시간 설정으로 바퀴별 비율(xxMotRateXX)을 조정한다.
//  거리 주행: 전진 1000 mm -> 후진 1000 mm -> 좌진 300 mm -> 우진 300 mm.
//  두 모드 모두 각 구간마다 SW1 또는 시리얼 입력을 기다린다. 모드 변경 후에는 전진부터 다시 시작한다.
//  주행 중 SW2를 누르면 현재 주행을 중단하고 모드를 바꾼 뒤 다음 시작 입력을 기다린다.
//  비율 상수의 기준 파일은 Mission/Mobilebase.h 이다. 사용 전 그 값을 아래에 복사하고, 조정 결과를 다시 Mobilebase.h에 적는다.
//
//  거리 모드는 Mission과 같은 확장 위치 제어 + 시간 기반 프로파일을 사용하며, 이 파일의 보정 비율을 적용한다.
//  바퀴 회전량 기준 거리이므로 바닥 미끄러짐/하중에 따른 실제 이동 거리는 실측 확인한다.

#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial1
const int DXL_DIR_PIN = 2;  // DIR PIN
const float DXL_PROTOCOL_VERSION = 2.0;
const unsigned long DXL_BAUDRATE = 1000000;  // Mission/Motor.h의 DXL_BAUDRATE와 같게: 57600 / 1000000

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// 컨트롤 테이블 아이템의 이름을 사용하기 위해 이 네임스페이스가 필요함
using namespace ControlTableItem;


//////////////  모터 구동용 변수/상수
#define MOBILE_DXL_ID_FL            0x01 // 좌측 전방 모터 아이디
#define MOBILE_DXL_ID_FR            0x02 // 우측 전방 모터 아이디
#define MOBILE_DXL_ID_BL            0x03 // 좌측 후방 모터 아이디
#define MOBILE_DXL_ID_BR            0x04 // 우측 후방 모터 아이디

#define NORMAL_MODE                 0x00 // drive mode의 방향 모드
#define REVERSE_MODE                0x01

#define VELOCITY_CONTROL_MODE       0x01 // operating mode
#define EXTENDED_POSITION_CONTROL_MODE 0x04
#define TIME_BASED_PROFILE          0x04 // drive mode의 시간 기반 프로파일 비트

#define SW1_PIN                     41
#define SW2_PIN                     40
#define BUZZER_PIN                   5

#define MOVE_FORWARD   1
#define MOVE_BACKWARD  2
#define MOVE_LEFTWARD  3
#define MOVE_RIGHTWARD 4

struct ButtonState {
  uint8_t pin;
  bool rawState;
  bool stableState;
  uint32_t changedAt;
};

ButtonState sw1 = { SW1_PIN, HIGH, HIGH, 0 };
ButtonState sw2 = { SW2_PIN, HIGH, HIGH, 0 };

// Arduino의 자동 함수 선언보다 사용자 정의 자료형을 먼저 알 수 있도록 직접 선언한다.
bool ButtonPressed(ButtonState& button);

// FindMobileBaseServos 함수에서 찾을 모터 아이디들
const uint8_t MOBILE_DXL_ID_CNT = 4;
const uint8_t MOBILE_DXL_IDS[MOBILE_DXL_ID_CNT] = { MOBILE_DXL_ID_FL, MOBILE_DXL_ID_FR,
                                                    MOBILE_DXL_ID_BL, MOBILE_DXL_ID_BR };


const float WHEEL_RADIUS_MM = 30.0f;
const uint32_t PROFILE_ACCELERATION_MS = 100; // Mission과 같은 설정값. 시간 기반 프로파일의 단위는 ms.
const uint32_t PROFILE_TIMEOUT_MARGIN_MS = 1000;

// sync write 용 상수, 구조체 정의 및 인스턴스화 (1 단위: 0.229 rpm)
const int32_t GOAL_VELOCITY_SPEED_SLOW =  75;
const int32_t GOAL_VELOCITY_SPEED      = 100;  // 회전 속도: 100 * 0.229 = 22.9 rev/min
                                               // 22.9 * 바퀴둘레(= 2 * PI * 30) = 4316.54 mm/min = 71.94 mm/sec
const int32_t GOAL_VELOCITY_SPEED_MORE = 130;
const int32_t GOAL_VELOCITY_SPEED_FAST = 160;

const uint16_t GOAL_VELOCITY_ADDR = 104; // sync write start 주소
const uint16_t GOAL_VELOCITY_SIZE = 4; // sync write data 길이

typedef struct sw_data { // 모터에 데이터를 쓰기 위한 구조체 정의
  int32_t goal_velocity;
} __attribute__((packed)) sw_data_t;

sw_data_t sw_data[MOBILE_DXL_ID_CNT];  // 모터 개수만큼 인스턴스 생성
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;  // syncwrite 정보 인스턴스 생성

// syncWrite할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[MOBILE_DXL_ID_CNT];

// 주소 108부터 가속 시간 / 전체 이동 시간 / 목표 위치를 네 바퀴에 동시에 쓴다.
struct ProfileData {
  int32_t acceleration_ms;
  int32_t duration_ms;
  int32_t goal_position;
} __attribute__((packed));

ProfileData profile_data[MOBILE_DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t profile_infos;
DYNAMIXEL::XELInfoSyncWrite_t profile_xels[MOBILE_DXL_ID_CNT];


//////////////////////////////////////////////////////////////    메인 프로그램

void  Beeps( int32_t nCount= 1, int32_t nDuration= 125, int32_t nFrequency= 330 );
void  Beeps( int32_t nCount, int32_t nDuration, int32_t nFrequency )
{
  //tone(BUZZER_PIN, nFrequency, nDuration);

  for(int32_t i=0; i<nCount; i++)
  {
    tone(BUZZER_PIN, nFrequency);
    delay(nDuration);
    noTone(BUZZER_PIN);
    delay(nDuration);
  }
}


bool profileMode = true;  // false: 기존 시간 주행(삑), true: 거리 프로파일 주행(삐삑)
bool motorReady = false;

void setup()
{
  Serial.begin( 115200 );

  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(SW2_PIN, INPUT_PULLUP);

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  sw1.rawState = sw1.stableState = digitalRead(SW1_PIN);
  sw2.rawState = sw2.stableState = digitalRead(SW2_PIN);

  dxl.begin( DXL_BAUDRATE );
  dxl.setPortProtocolVersion( DXL_PROTOCOL_VERSION );

  // 모터가 모두 있는지 확인될 때 까지 진행하지 않음 (1초마다 재시도, 빠진 ID 출력)
  while( ! FindMobileBaseServos() ) {
    Serial.println(F("  wheel motors not found. check power/cable/baudrate. retry in 1s..."));
    delay( 1000 );
  }

  // sync write 준비
  sw_infos.packet.p_buf = nullptr;  // nullptr을 전달하면 내부 버퍼를 사용
  sw_infos.packet.is_completed = false;  // false로 초기화
  sw_infos.addr = GOAL_VELOCITY_ADDR;  // 컨트롤 테이블에 sync write를 시작하는 주소 (= 104)
  sw_infos.addr_length = GOAL_VELOCITY_SIZE;  // sync write하는 데이터 길이 (= 4)
  sw_infos.p_xels = info_xels_sw;  // sync write 할 모터 정보
  sw_infos.xel_count = 0;  // sync write 할 모터 개수

  sw_data[0].goal_velocity = 0;  // 모터에 write 할 데이터 초기화
  sw_data[1].goal_velocity = 0;
  sw_data[2].goal_velocity = 0;
  sw_data[3].goal_velocity = 0;

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for( uint8_t i = 0; i < MOBILE_DXL_ID_CNT; i++)
  {
    info_xels_sw[i].id = MOBILE_DXL_IDS[i];
    info_xels_sw[i].p_data = (uint8_t*) &sw_data[i].goal_velocity;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true; // sync write 정보가 변경됨을 설정

  profile_infos.packet.p_buf = nullptr;
  profile_infos.packet.is_completed = false;
  profile_infos.addr = 108;
  profile_infos.addr_length = sizeof(ProfileData);
  profile_infos.p_xels = profile_xels;
  profile_infos.xel_count = MOBILE_DXL_ID_CNT;

  for (uint8_t i = 0; i < MOBILE_DXL_ID_CNT; ++i) {
    profile_xels[i].id = MOBILE_DXL_IDS[i];
    profile_xels[i].p_data = (uint8_t*)&profile_data[i];
  }
  profile_infos.is_info_changed = true;

  Serial.println(F("setup() done. rates: see Mission/Mobilebase.h"));

  if (ConfigureMotorMode(profileMode))
  {
    delay( 1000 );
    AnnounceMode();
  }
}


/*
 * MOBILE_DXL_IDS 배열에 있는 아이디의 모터들이 모두 통신 가능한지
 * 확인하는 함수. 모두 통신 가능하면 true, 아니면 false를 반환. 빠진 ID를 출력한다.
 */
bool FindMobileBaseServos()
{
  uint8_t ids_pinged[16] = {0,};
  uint8_t count = dxl.ping( DXL_BROADCAST_ID, ids_pinged, sizeof(ids_pinged), 100 );
  bool is_each_motor_found = true;

  for( uint8_t i = 0; i < MOBILE_DXL_ID_CNT; i++ )
  {
    bool found = false;

    for( uint8_t j = 0; j < count; j++ ) {
      if( ids_pinged[j] == MOBILE_DXL_IDS[i] ) { found = true; break; }
    }

    if( ! found ) {
      Serial.print(F("  missing wheel motor ID "));
      Serial.println( MOBILE_DXL_IDS[i] );
      is_each_motor_found = false;
    }
  }

  return  is_each_motor_found;
}


/*
 * sync write 하기 위한 모바일베이스의 goal velocity 값들을 설정하는 함수
 */
void SetMobileGoalVelocityForSyncWrite( int32_t fl_goal_velocity, int32_t fr_goal_velocity,
                                        int32_t bl_goal_velocity, int32_t br_goal_velocity )
{
  sw_data[0].goal_velocity = fl_goal_velocity;
  sw_data[1].goal_velocity = fr_goal_velocity;
  sw_data[2].goal_velocity = bl_goal_velocity;
  sw_data[3].goal_velocity = br_goal_velocity;

  sw_infos.is_info_changed = true;

  int count = 0;
  while( ! dxl.syncWrite( &sw_infos ) ) {  // Mission/Mobilebase.cpp와 같이 10회 상한
    count++;
    delay( 1 );

    if( count > 10 ) {
      MotorFault();
      break;
    }
  }
}


///////////////////////////////////////////////////////////////////////
//  바퀴별 속도 비율. 기준 파일: Mission/Mobilebase.h (같은 값 유지).
//  (MOBILEBASE_SPEED_FAST = 160 일때)

// 전진 시 속도 비율
const float flMotRateFW = 1.000;  // + R 진행방향(앞쪽)을 바라볼 때 좌측/우측
const float frMotRateFW = 0.980;  // + L
const float blMotRateFW = 0.990;  // + L
const float brMotRateFW = 0.990;  // + R

// 후진 시 속도 비율
const float flMotRateBW = 0.990;  // - R 진행방향(뒷쪽)을 바라볼 때 좌측/우측
const float frMotRateBW = 0.990;  // - L
const float blMotRateBW = 0.990;  // - L
const float brMotRateBW = 1.000;  // - R

// 좌진 시 속도 비율
const float flMotRateLW = 0.935;  // - L 진행방향(왼쪽)을 바라볼 때 좌측/우측
const float frMotRateLW = 0.935;  // + R
const float blMotRateLW = 1.000;  // + R
const float brMotRateLW = 0.985;  // - L

// 우진 시 속도 비율
const float flMotRateRW = 0.990;  // + L 진행방향(오른쪽)을 바라볼 때 좌측/우측
const float frMotRateRW = 0.990;  // - R
const float blMotRateRW = 1.000;  // - R
const float brMotRateRW = 0.990;  // + L


void  Forward( int32_t drivingSpeed )  // 전진
{
  SetMobileGoalVelocityForSyncWrite( drivingSpeed * flMotRateFW, drivingSpeed * frMotRateFW,
                                     drivingSpeed * blMotRateFW, drivingSpeed * brMotRateFW );
}

void  Backward( int32_t drivingSpeed )  // 후진
{
  SetMobileGoalVelocityForSyncWrite( -drivingSpeed * flMotRateBW, -drivingSpeed * frMotRateBW,
                                     -drivingSpeed * blMotRateBW, -drivingSpeed * brMotRateBW );
}

void  Leftward( int32_t drivingSpeed )  // 좌진
{
  SetMobileGoalVelocityForSyncWrite( -drivingSpeed * flMotRateLW,  drivingSpeed * frMotRateLW,
                                      drivingSpeed * blMotRateLW, -drivingSpeed * brMotRateLW );
}

void  Rightward( int32_t drivingSpeed )  // 우진
{
  SetMobileGoalVelocityForSyncWrite( drivingSpeed * flMotRateRW, -drivingSpeed * frMotRateRW,
                                    -drivingSpeed * blMotRateRW,  drivingSpeed * brMotRateRW );
}

void  Stop()  // 모든 모터 정지
{
  SetMobileGoalVelocityForSyncWrite( 0, 0, 0, 0 );
}


// 통신 실패 값을 정상 위치 0으로 오인하지 않도록 오류 상태까지 확인한다.
bool ReadMotorItem(uint8_t item, uint8_t id, int32_t& value)
{
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    value = dxl.readControlTableItem(item, id, 10);
    if (dxl.getLastLibErrCode() == 0 && dxl.getLastStatusPacketError() == 0) return true;
  }
  return false;
}

bool WriteMotorItem(uint8_t item, uint8_t id, int32_t value)
{
  // 자료형을 명시해 Dynamixel2Arduino의 model_num 오버로드와 구분한다.
  return dxl.writeControlTableItem(item, id, value, uint32_t(10));
}

void MotorFault()
{
  motorReady = false;

  for (uint8_t i = 0; i < MOBILE_DXL_ID_CNT; ++i)
    WriteMotorItem(TORQUE_ENABLE, MOBILE_DXL_IDS[i], 0);

  tone(BUZZER_PIN, 330, 500);
  Serial.println(F("Motor error: torque-off requested. Check motors, then press SW2 to retry."));
}

bool ConfigureMotorMode(bool useProfile)
{
  motorReady = false;
  bool success = true;

  // 전 바퀴를 먼저 멈춘 다음 모드를 설정한다. 실패하면 주행을 허용하지 않는다.
  for (uint8_t i = 0; i < MOBILE_DXL_ID_CNT; ++i)
    success = WriteMotorItem(TORQUE_ENABLE, MOBILE_DXL_IDS[i], 0) && success;

  if (!success) { MotorFault(); return false; }

  for (uint8_t i = 0; i < MOBILE_DXL_ID_CNT; ++i)
  {
    const uint8_t id = MOBILE_DXL_IDS[i];
    const uint8_t direction = (id == MOBILE_DXL_ID_FL || id == MOBILE_DXL_ID_BL) ? NORMAL_MODE : REVERSE_MODE;

    success = WriteMotorItem(DRIVE_MODE, id, direction | (useProfile ? TIME_BASED_PROFILE : 0)) && success;
    success = WriteMotorItem(OPERATING_MODE, id, useProfile ? EXTENDED_POSITION_CONTROL_MODE : VELOCITY_CONTROL_MODE) && success;
    success = WriteMotorItem(PROFILE_ACCELERATION, id, 0) && success;
    success = WriteMotorItem(GOAL_VELOCITY, id, 0) && success;

    if (useProfile) {
      int32_t position;
      // 토크를 다시 켤 때 이전 목표 위치로 이동하지 않게 현재 위치를 먼저 목표로 지정한다.
      if (!ReadMotorItem(PRESENT_POSITION, id, position)) success = false;
      else success = WriteMotorItem(GOAL_POSITION, id, position) && success;
    }
  }

  if (!success) { MotorFault(); return false; }

  for (uint8_t i = 0; i < MOBILE_DXL_ID_CNT; ++i) {
    if (!WriteMotorItem(TORQUE_ENABLE, MOBILE_DXL_IDS[i], 1)) {
      MotorFault();
      return false;
    }
  }

  profileMode = useProfile;
  motorReady = true;

  return true;
}

void AnnounceMode()
{
  Serial.println(profileMode ? F("Mode: PROFILE. FW/BW 1000 mm, LW/RW 300 mm (2 beeps).")
                             : F("Mode: TIME. Original speed/time settings (1 beep)."));

  Beeps((profileMode ? 2 : 1), 125, 330);
}

bool ButtonPressed(ButtonState& button)
{
  const bool reading = digitalRead(button.pin);
  const uint32_t now = millis();

  if (reading != button.rawState) {
    button.rawState = reading;
    button.changedAt = now;
  }

  if (reading != button.stableState && uint32_t(now - button.changedAt) >= 30) {
    button.stableState = reading;
    return reading == LOW;  // 길게 눌러도 한 번만 전환한다.
  }

  return false;
}

bool CheckModeButton()
{
  if (!ButtonPressed(sw2)) return false;
  if (ConfigureMotorMode(!profileMode)) AnnounceMode();

  // 전환에 사용된 입력으로 바로 출발하지 않도록 시작 입력을 비운다.
  sw1.rawState = sw1.stableState = digitalRead(SW1_PIN);
  sw1.changedAt = millis();

  while (Serial.available()) Serial.read();

  return true;  // 현재 시험 순서를 끝내고 loop() 첫 구간부터 다시 대기
}

bool WaitMotionDelay(uint32_t duration)
{
  const uint32_t started = millis();

  while (uint32_t(millis() - started) < duration) {
    if (!motorReady || CheckModeButton()) return false;

    ButtonPressed(sw1);  // 주행 중 누른 시작 버튼을 다음 구간의 시작으로 사용하지 않는다.
    delay(10);
  }

  return motorReady;
}

bool WaitButtonOrInputChar()
{
  Serial.println(F("SW1 or ENTER: start next move. SW2: change mode / restart from forward."));

  while (true) {
    if (CheckModeButton()) return false;

    const bool startPressed = ButtonPressed(sw1);

    if (Serial.available()) {
      while (Serial.read() >= 0) delay(2);
      if (motorReady) return true;
    }

    if (startPressed && motorReady) return WaitMotionDelay(500);
    delay(10);
  }
}

bool DriveProfileDistance(float distanceMm, uint8_t direction, int32_t mmPerSec)
{
  if (!motorReady || !profileMode) return false;

  // Mission/Mobilebase.h와 같은 환산식과 바퀴별 비율을 사용한다.
  const int32_t ticks = (int32_t)(distanceMm / WHEEL_RADIUS_MM * (4095.0f / (2.0f * PI)));
  const uint32_t duration = (uint32_t)(distanceMm / mmPerSec * 1000.0f);

  int32_t delta[MOBILE_DXL_ID_CNT];

  switch (direction) {
    case MOVE_FORWARD:
      delta[0] = ticks * flMotRateFW; delta[1] = ticks * frMotRateFW;
      delta[2] = ticks * blMotRateFW; delta[3] = ticks * brMotRateFW;
      break;
    case MOVE_BACKWARD:
      delta[0] = -ticks * flMotRateBW; delta[1] = -ticks * frMotRateBW;
      delta[2] = -ticks * blMotRateBW; delta[3] = -ticks * brMotRateBW;
      break;
    case MOVE_LEFTWARD:
      delta[0] = -ticks * flMotRateLW; delta[1] = ticks * frMotRateLW;
      delta[2] = ticks * blMotRateLW; delta[3] = -ticks * brMotRateLW;
      break;
    case MOVE_RIGHTWARD:
      delta[0] = ticks * flMotRateRW; delta[1] = -ticks * frMotRateRW;
      delta[2] = -ticks * blMotRateRW; delta[3] = ticks * brMotRateRW;
      break;
    default: return false;
  }

  for (uint8_t i = 0; i < MOBILE_DXL_ID_CNT; ++i) {
    int32_t position;

    if (!ReadMotorItem(PRESENT_POSITION, MOBILE_DXL_IDS[i], position)) {
      MotorFault();
      return false;
    }

    const int64_t goal = (int64_t)position + delta[i];

    if (goal < -1048575L || goal > 1048575L) {
      Serial.println(F("Profile target exceeds extended-position range."));
      MotorFault();
      return false;
    }

    profile_data[i].acceleration_ms = PROFILE_ACCELERATION_MS;
    profile_data[i].duration_ms = duration;
    profile_data[i].goal_position = (int32_t)goal;
  }

  profile_infos.is_info_changed = true;

  bool sent = false;

  for (uint8_t attempt = 0; attempt < 11 && !sent; ++attempt)
    sent = dxl.syncWrite(&profile_infos);

  if (!sent) { MotorFault(); return false; }

  const uint32_t started = millis();
  const uint32_t timeout = duration + duration / 4 + PROFILE_TIMEOUT_MARGIN_MS;

  while (uint32_t(millis() - started) < timeout) {
    if (!WaitMotionDelay(10)) return false;

    bool arrived = true;

    for (uint8_t i = 0; i < MOBILE_DXL_ID_CNT; ++i) {
      int32_t status, position;

      if (!ReadMotorItem(MOVING_STATUS, MOBILE_DXL_IDS[i], status)
          || !ReadMotorItem(PRESENT_POSITION, MOBILE_DXL_IDS[i], position)) {
        arrived = false;
        break;
      }

      // 이전 목표의 도착 비트만 보고 조기 완료하지 않도록 실제 목표 위치도 확인한다.
      if (!(status & 0x01) || (status & 0x02)
          || abs(position - profile_data[i].goal_position) > 20) arrived = false;
    }

    if (arrived) return true; // 위치 제어가 목표에서 유지한다. 속도 0 명령은 사용하지 않는다.
  }

  Serial.println(F("Profile move timed out."));
  MotorFault();

  return false;
}

/////////////////////////////////////////////////////////////////////////////////////// LOOP

int movingTime     = 5000;  // profileMode가 false 일 때 전진/후진 이동 시간 (ms)
int movingTimeMore = movingTime * (GOAL_VELOCITY_SPEED * 1.0 / GOAL_VELOCITY_SPEED_MORE);  // 좌진/우진 속력 150
int movingTimeFast = movingTime * (GOAL_VELOCITY_SPEED * 1.0 / GOAL_VELOCITY_SPEED_FAST);  // 빠른 속력 180

// 시간 모드는 기존 시험 선택을 유지한다. 주석을 풀면 시간 기반 좌진/우진도 시험할 수 있다.
// 시간 모드의 각 구간은 같은 거리(기본 속력 5초 분량)를 달린다.

const float mmFwdBwd = 500.0f;   // profileMode가 true 일 때 전진/후진 거리 (mm)
const float mmLwdRwd = 400.0f;    // profileMode가 true 일 때 좌진/우진 거리 (mm)

const int32_t mm_MOVE_SPEED = GOAL_VELOCITY_SPEED_FAST; // profileMode가 true 일 때 이동 속도


void loop()
{
  if( profileMode )
  {
    // Serial.println(F("Next: FORWARD"));
    // if (!WaitButtonOrInputChar()) return;
    // if (!DriveProfileDistance(mmFwdBwd, MOVE_FORWARD, mm_MOVE_SPEED)) return;

    Serial.println(F("Next: BACKWARD"));
    if (!WaitButtonOrInputChar()) return;
    if (!DriveProfileDistance(mmFwdBwd, MOVE_BACKWARD, mm_MOVE_SPEED)) return;

    // Serial.println(F("Next: LEFTWARD"));
    // if (!WaitButtonOrInputChar()) return;
    // if (!DriveProfileDistance(mmLwdRwd, MOVE_LEFTWARD, mm_MOVE_SPEED)) return;

    // Serial.println(F("Next: RIGHTWARD"));
    // if (!WaitButtonOrInputChar()) return;
    // DriveProfileDistance(mmLwdRwd, MOVE_RIGHTWARD, mm_MOVE_SPEED);
    return;
  }


  Serial.println(F("Next: FORWARD (time mode)"));
  if (!WaitButtonOrInputChar()) return;

  Forward( GOAL_VELOCITY_SPEED_FAST );   // 전진 (Mission의 SEARCH_MM_PER_SEC = FAST 속력)
  if (!WaitMotionDelay(movingTimeFast)) return;
  Stop();


  Serial.println(F("Next: BACKWARD (time mode)"));
  if (!WaitButtonOrInputChar()) return;

  Backward( GOAL_VELOCITY_SPEED_FAST );  // 후진
  if (!WaitMotionDelay(movingTimeFast)) return;
  Stop();


  // Serial.println(F("Next: LEFTWARD (time mode)"));
  // if (!WaitButtonOrInputChar()) return;

  // Leftward( GOAL_VELOCITY_SPEED_MORE );   // 좌진
  // if (!WaitMotionDelay(movingTimeMore)) return;
  // Stop();


  // Serial.println(F("Next: RIGHTWARD (time mode)"));
  // if (!WaitButtonOrInputChar()) return;

  // Rightward( GOAL_VELOCITY_SPEED_MORE );  // 우진
  // if (!WaitMotionDelay(movingTimeMore)) return;
  // Stop();
}
