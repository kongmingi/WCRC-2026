//////////////  매니퓰레이터 자세 모니터 (교시 도구)
//
//  팔 모터 4개(ID 5~8)의 현재 관절값을 읽어 writeEEPROM 표에 바로 붙일 수 있는 형식(LB, HB 16진수)과
//  10진수로 출력한다. EEPROM에 실제로 기록된 자세(ID 1~36)를 확인하고, 번호를 입력해 그 자세를 재생해 볼 수 있다.
//
//  조작 (시리얼 모니터 115200):
//    SW1(왼쪽 버튼)   현재 관절값 출력                시리얼 'r' 과 같음
//    SW2(오른쪽 버튼) 팔 토크 켜기/끄기 전환           시리얼 't' 와 같음 (끄면 손으로 팔을 움직여 교시)
//    시리얼 'l'       EEPROM 자세 목록(ID 1~36) 출력
//    시리얼 숫자      그 ID의 자세 재생 (예: 28 입력 후 Enter). 토크가 켜져 있어야 함
//    시리얼 'o' / 'c' 그리퍼 열기 / 닫기
//    시리얼 'h'       조작법 출력
//
//  Mission과 같게 유지할 것: Manipulator.h(구조체·관절 한계·오프셋), Pins.h, 아래 enum의 ID 배치
//  모터 속도는 이 도구의 Motor.h에서 USE_DXL_1M_BPS로 별도 선택한다(기본 false=57600).

#define DEBUG
#define DEBUG_SERIAL    Serial

#include "Pins.h"
#include "Motor.h"
#include "Manipulator.h"

#include <Pixy2.h> // 그리퍼 서보는 Pixy2의 서보 출력으로 구동

//////////////  Pixy2 카메라
Pixy2 pixy;        // SPI 통신하는 Pixy2 객체 생성
bool pixyOk = false;     // 카메라가 없으면 그리퍼·램프 명령은 건너뜀

//////////////  그리퍼 관련 (Mission/Devices.h와 같음)

#define GRIP_ANGLE_CLOSE          940  // 그리퍼를 반쯤 닫아서 블록 잡기 (최대로 꽉 닫기는 1000)
#define GRIP_ANGLE_OPEN           100  // 그리퍼 열리는 값 (더 작으면 소리가 나고, 기구적 문제로 모터에 무리가 감)

#define POSE_PLAY_TIME_MS         1500 // 자세 재생 동작 시간

//////////////  자세 ID (Mission과 같은 번호 배치. ID 27은 기존 도구의 이름·재생 방식 유지)

enum ManipulatorPoseID {
  FIND_MISSION_BLOCKS1  =  1,
  FIND_MISSION_BLOCKS2, // 2

  FRONT_MOVE,           // 3
  STORAGE_SEARCH,       // 4

  PICK_UP_APPROACH,     // 5   L2(가운데층)
  PICK_UP_GRIP,         // 6
  PICK_UP_OUT,          // 7

  PICK_DOWN_APPROACH,   // 8   L1(맨 아래층)
  PICK_DOWN_GRIP,       // 9
  PICK_DOWN_OUT,        // 10

  // 11 ~ 18 = Dropbox A의 1~8번 칸에 넣는 자세 (DROPBOX_8POSE_START_IDX + 칸 번호)
  // 19 ~ 26 = Dropbox B의 1~8번 칸에 넣는 자세 (DROPBOX_B_8POSE_START_IDX + 칸 번호)

  STORAGE_SEARCH_TOP    = 27,  // 기존 도구 호환용. 현재 Mission에서는 RESERVED_27로 미사용
  PICK_TOP_APPROACH,    // 28  L3(맨 위층)
  PICK_TOP_GRIP,        // 29
  PICK_TOP_OUT,         // 30

  STORAGE_SEARCH_L3     = 31,  // L3(맨 위층) 탐색: 저장된 1번 관절값 사용
  STORAGE_SEARCH_L2,          // 32: L2(가운데층) 탐색
  STORAGE_SEARCH_L1,          // 33: L1(맨 아래층) 탐색
  RESERVED_STORAGE_ALIGN_L3,  // 34: Mission 미사용 예약 자세. 도구에서는 재생 가능
  RESERVED_STORAGE_ALIGN_L2,  // 35
  RESERVED_STORAGE_ALIGN_L1   // 36
};

#define DROPBOX_8POSE_START_IDX     10  // 잡은 블록을 Dropbox A의 1~8 번 칸에 넣는 자세의 시작 인덱스 (ID 11~18)
#define DROPBOX_B_8POSE_START_IDX   18  // 잡은 블록을 Dropbox B의 1~8 번 칸에 넣는 자세의 시작 인덕스 (ID 19~26)


////////////////////////////////////

void OpenGripper(Pixy2& pixy) {   // 참조로 전달 (값 복사하면 소멸자가 카메라 버퍼를 해제함)
  pixy.setServos(0, GRIP_ANGLE_OPEN);
}

void CloseGripper(Pixy2& pixy) {
  pixy.setServos(0, GRIP_ANGLE_CLOSE);
}


void setup()
{
  pinMode(SW1_PIN, INPUT);
  pinMode(SW2_PIN, INPUT);

  Serial.begin( 115200 );
  Serial.println(F("\n< Manipulator Pose Monitor >"));

  pixyOk = (pixy.init() >= 0);   // 카메라가 없으면 5초 뒤 실패로 넘어감
  if (!pixyOk)
    Serial.println(F("Pixy not found: gripper and lamp commands disabled"));

  Serial.print(F("Motor baudrate: "));
  Serial.println(DXL_BAUDRATE);
  initMotorCommunication();

  while (!initManipulator()) {
    Serial.print(F("arm motors (ID 5~8) not found. check power/cable/baudrate("));
    Serial.print(DXL_BAUDRATE);
    Serial.println(F("). retry in 1s..."));
    delay(1000);
  }

  if (pixyOk) {
    //OpenGripper( pixy );
    //CloseGripper( pixy );
    pixy.setLamp( 0, 0 ); // 카메라 램프 켜기
  }

  PrintHelp();
  PrintManipulatorPoseListFromEEPROM();   // 지금 EEPROM에 든 자세 확인 (writeEEPROM 전이면 '-'로 표시됨)
}


bool sw2PrevState = HIGH;

void loop()
{
  if( digitalRead(SW1_PIN) == LOW )
  {
    printCurrentPose();
    delay( 500 );
  }
  else
  {
    bool sw2CurrState = digitalRead( SW2_PIN );

    if( sw2PrevState == HIGH  &&  sw2CurrState == LOW )
    {
      toggleArmTorque();
      delay( 500 );
    }

    sw2PrevState = sw2CurrState;
  }

  if( Serial.available() )
    HandleSerialCommand();

  delay(10);
}


void PrintHelp()
{
  Serial.println(F("SW1 or 'r': print current joint values  |  SW2 or 't': torque on/off"));
  Serial.print(F("'l': list EEPROM poses  |  <id> (1~"));
  Serial.print(MANIPULATOR_POSE_ID_MAX_CNT);
  Serial.println(F("): play pose  |  'o'/'c': gripper open/close  |  'h': help"));
}


// 한 줄(최대 7자)을 읽어 명령 처리. 줄바꿈이 없으면 마지막 글자 뒤 100ms에 처리.
// 글자 명령은 정확히 한 글자, 자세 번호는 입력 전체가 숫자(1~36)일 때만 받는다. 그 외와 7자 초과는 무시.
void HandleSerialCommand()
{
  char buf[8];
  uint8_t len = 0;
  bool overflow = false;
  unsigned long t0 = millis();

  while( millis() - t0 < 100 )
  {
    if( Serial.available() )
    {
      char c = Serial.read();
      if( c == '\n' || c == '\r' ) { if( len > 0 || overflow ) break; else continue; }
      if( len < sizeof(buf) - 1 ) buf[len++] = c;
      else overflow = true;   // 넘친 글자는 버리되 이 줄 전체를 무효로 처리
      t0 = millis();
    }
  }
  buf[len] = 0;

  if( overflow )
  {
    Serial.println(F("input too long, ignored"));
    return;
  }
  if( len == 0 ) return;

  if( len == 1  &&  ! isDigit( buf[0] ) )   // 한 글자 명령
  {
    switch( buf[0] )
    {
      case 'r': case 'R':  printCurrentPose();  return;
      case 't': case 'T':  toggleArmTorque();   return;
      case 'l': case 'L':  PrintManipulatorPoseListFromEEPROM();  return;
      case 'h': case 'H': case '?':  PrintHelp();  return;
      case 'o': case 'O':
        if( pixyOk ) { OpenGripper( pixy );  Serial.println(F("gripper open")); }
        return;
      case 'c': case 'C':
        if( pixyOk ) { CloseGripper( pixy ); Serial.println(F("gripper close")); }
        return;
    }
  }

  // 자세 번호: 모든 글자가 숫자여야 함 ("28x", "1.5", "4 5" 등은 거부)
  int id = 0;
  for( uint8_t i = 0; i < len; i++ )
  {
    if( ! isDigit( buf[i] ) )
    {
      Serial.print(F("unknown command: "));
      Serial.println( buf );
      PrintHelp();
      return;
    }
    if( id < 1000 ) id = id * 10 + (buf[i] - '0');   // 7자리 입력도 넘치지 않게
  }

  if( id >= 1  &&  id <= MANIPULATOR_POSE_ID_MAX_CNT )
    PlayPose( (uint8_t)id );
  else {
    Serial.print(F("pose id out of range (1~"));
    Serial.print( MANIPULATOR_POSE_ID_MAX_CNT );
    Serial.print(F("): "));
    Serial.println( buf );
  }
}


// 기존 1~30 재생 방식을 유지하고 31~36은 EEPROM에 저장된 관절값으로 실행한다.
void PlayPose( uint8_t id )
{
  ManipulatorPose pose = ReadManipulatorPresentPoseFromEEPROM( id );

  if( ! IsValidManipulatorPose( pose, id ) )
  {
    Serial.print(F("ID "));
    Serial.print( id );
    Serial.println(F(": no valid pose in EEPROM. run writeEEPROM first"));
    return;
  }

  int32_t torque = dxl.readControlTableItem(TORQUE_ENABLE, ARM_DXL_IDS[0]);
  if( dxl.getLastLibErrCode() != DXL_LIB_OK )
  {
    Serial.println(F("torque read failed. check cable/power and retry"));
    return;
  }
  if( torque != TORQUE_ON )
  {
    Serial.println(F("torque is off. press SW2 or 't' to turn on, then retry"));
    return;
  }

  // 기존 재생 규칙 유지: ID 4~10, 27~30은 1번 모터(ID 5)를 0도(2047)로 지정한다.
  // 31~33은 Mission의 층별 탐색과 같이 저장된 1번 관절값을 사용한다. 34~36도 저장값으로 재생한다.
  bool forceMotor1Zero = (id >= STORAGE_SEARCH && id <= PICK_DOWN_OUT) || (id >= STORAGE_SEARCH_TOP && id <= PICK_TOP_OUT);

  Serial.print(F("play pose ID "));
  Serial.print( id );
  Serial.print(F(" : m1= "));
  if( forceMotor1Zero ) Serial.print(F("0deg(2047), table "));
  Serial.print( pose.manipulatorMotor1Value );
  Serial.print(F(", m2= "));
  Serial.print( pose.manipulatorMotor2Value );
  Serial.print(F(", m3= "));
  Serial.print( pose.manipulatorMotor3Value );
  Serial.print(F(", m4= "));
  Serial.println( pose.manipulatorMotor4Value );

  if( forceMotor1Zero )
    RunManipulatorPoseWithPoseDataInEEPROM( id, POSE_PLAY_TIME_MS, 0.0 );
  else
    RunManipulatorPoseWithPoseDataInEEPROM( id, POSE_PLAY_TIME_MS );
}


void PrintHex2( uint8_t b )
{
  Serial.print(F("0x"));
  if( b < 0x10 ) Serial.print('0');
  Serial.print( b, HEX );
}

// 현재 관절값을 writeEEPROM 표 형식(LB, HB)과 10진수로 출력
void printCurrentPose()
{
  int32_t value[ARM_DXL_ID_CNT];

  for( uint8_t i = 0; i < ARM_DXL_ID_CNT; i++ )
  {
    value[i] = dxl.readControlTableItem( PRESENT_POSITION, ARM_DXL_IDS[i] );

    // 각 읽기 직후 라이브러리 오류 코드로 판정한다. CRC 오류 등은 반환값이 0이 아닐 수 있고,
    // 1번 모터의 정상 위치 0은 실패가 아니다. 뒤의 성공 통신이 앞의 오류를 덮지 않도록 바로 확인.
    DXLLibErrorCode_t err = dxl.getLastLibErrCode();
    if( err != DXL_LIB_OK )
    {
      Serial.print(F("READ FAIL: motor ID "));
      Serial.print( ARM_DXL_IDS[i] );
      Serial.print(F(", lib error code "));
      Serial.print( err );
      Serial.println(F(". check cable/power and press again"));
      return;
    }
  }

  Serial.print(F("  writeEEPROM row (m1~m4 LB,HB):  "));
  for( uint8_t i = 0; i < ARM_DXL_ID_CNT; i++ )
  {
    PrintHex2( lowByte(value[i]) );
    Serial.print(F(", "));
    PrintHex2( highByte(value[i]) );
    if( i < ARM_DXL_ID_CNT - 1 ) Serial.print(F(",  "));
  }
  Serial.println(F(","));

  Serial.print(F("  decimal m1~m4               :  "));
  for( uint8_t i = 0; i < ARM_DXL_ID_CNT; i++ )
  {
    Serial.print( value[i] );
    if( i < ARM_DXL_ID_CNT - 1 ) Serial.print(F(", "));
  }
  Serial.println();
  Serial.println();
}

void toggleArmTorque()
{
  int32_t torque = dxl.readControlTableItem(TORQUE_ENABLE, ARM_DXL_IDS[0]);

  if( dxl.getLastLibErrCode() != DXL_LIB_OK )   // 읽기 실패 시 0이 돌아오므로 오류 코드로 판정. 실패하면 토크를 바꾸지 않음
  {
    Serial.println(F("torque read failed. check cable/power and press again"));
    return;
  }

  if (torque == TORQUE_ON)
  {
    Serial.println(F("Torque off (move the arm by hand, then press SW1 to read)"));
    turnOffManipulatorTorque();
  } else {
    Serial.println(F("Torque on"));
    turnOnManipulatorTorque();
  }
}
