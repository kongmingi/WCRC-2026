//////////////  매니퓰레이터 자세 36개를 EEPROM에 기록하는 도구
//
//  아래 표(manipulatorPoseData)가 자세의 원본이다. poseMon으로 읽은 관절값(LB, HB 16진수 쌍)을
//  해당 ID 행에 넣고 이 스케치를 올린 뒤 SW1 버튼을 누르면 EEPROM 0..1439에 기록한다.
//
//  표의 값 개수가 1,440개가 아니면(하나라도 빠지거나 많으면) 컴파일 오류(static_assert)로 막는다.
//  기록 전에는 각 행이 (1, ID)로 시작하는지 검사해 행 안에서 밀린 경우를 잡고, 기록 후에는 다시 읽어 비교한다.
//  표는 Flash(PROGMEM)에 두므로 SRAM을 쓰지 않는다. 표의 내용과 편집 방법은 그대로다.

#include <EEPROM.h>
#include <avr/pgmspace.h>

#define SINGLE_POSE_DATA_SIZE   40  // 각 자세마다 마다 40 바이트 (Mission/Manipulator.h의 MANIPULATOR_POSE_DATA_SIZE)
#define TOTAL_POSE_COUNT        36  // 기존 ID 1~30 + 층별 탐색 31~33 + Reserved 정렬용 34~36
#define TOTAL_DATA_SIZE         (TOTAL_POSE_COUNT * SINGLE_POSE_DATA_SIZE)  // (36 * 40) = 1440. Mission/Manipulator.h의 MANIPULATOR_POSE_ID_MAX_CNT와 같을 것

const uint8_t manipulatorPoseData[] PROGMEM =   // 길이는 아래 초기화 값 개수로 정해지고, 표 끝의 static_assert가 1440개인지 컴파일 때 확인한다
{
  //////////  아래의 9 줄:  메인 소스 파일에서 enum ManipulatorPoseID {...} 참고

  //----  ----    ----------  -----------  -----------  -----------    -------------------------------
  //항상  Line    ARM1-위치값  ARM2-위치값  ARM3-위치값  ARM4-위치값     이후 뒤쪽의 값들은 무시함
  //  1    ID     1-LB  1-HB   2-LB  2-HB   3-LB  3-HB   4-LB  4-HB    Ignore values here after ......
  //----  ----    ----  ----   ----  ----   ----  ----   ----  ----    -------------------------------

    // 1= FIND_MISSION_BLOCKS1, 2= FIND_MISSION_BLOCKS2
      1,  1,      0x0E, 0x04,  0x7F, 0x09,  0x3E, 0x01,  0x51, 0x08,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1,  2,      0xD9, 0x03,  0xF8, 0x09,  0x27, 0x02,  0xC7, 0x06,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // 3= FRONT_MOVE
      1,  3,      0x00, 0x08,  0x70, 0x0A,  0x64, 0x03,  0x77, 0x03,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // 4= STORAGE_SEARCH (이동/복귀 자세. 색상 탐색은 층별 ID 31~33 사용)
      1,  4,      0x00, 0x08,  0x0A, 0x0A,  0x72, 0x04,  0x6B, 0x04,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

    // (가운데 L2행 4칸) 5= PICK_UP_APPROACH, 6= PICK_UP_GRIP, 7= PICK_UP_OUT
      1,  5,      0x00, 0x07,  0x34, 0x09,  0x14, 0x03,  0xC3, 0x07,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // 6: 닫기 직전 높이 3 mm 하강(기존 대비 모델값 -3.04 mm). 전후 위치/피치 유지, m1~m4=1792,2053,967,2082
      1,  6,      0x00, 0x07,  0x05, 0x08,  0xC7, 0x03,  0x22, 0x08,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1,  7,      0x00, 0x07,  0x93, 0x09,  0x57, 0x03,  0xF6, 0x06,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

    // (아래 L1행 4칸) 8= PICK_DOWN_APPROACH, 9= PICK_DOWN_GRIP, 10= PICK_DOWN_OUT
      1,  8,      0x00, 0x07,  0x99, 0x08,  0xC0, 0x01,  0x81, 0x09,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // 9: 닫기 직전 높이 7 mm 하강(기존 대비 모델값 -7.03 mm). 전후 위치/피치 유지, m1~m4=1792,1793,610,2722
      1,  9,      0x00, 0x07,  0x01, 0x07,  0x62, 0x02,  0xA2, 0x0A,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 10,      0x00, 0x07,  0xFC, 0x08,  0xE8, 0x00,  0x26, 0x0A,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

  //////////  아래의 8 줄:  미션수행 Zone의 1번에서 8번까지의 칸 위치 (Dropbox A 잡은 블록을 내려놓는 위치)

  //----  ----    ----------  -----------  -----------  -----------    -------------------------------
  //항상  Line    ARM1-위치값  ARM2-위치값  ARM3-위치값  ARM4-위치값     이후 뒤쪽의 값들은 무시함
  //  1    ID    1-LB  1-HB   2-LB  2-HB   3-LB  3-HB   4-LB  4-HB     Ignore values here after ......
  //----  ----    ----  ----   ----  ----   ----  ----   ----  ----    -------------------------------

    // A 1 ~ 4 번칸 위치
      1, 11,      0x34, 0x02,  0x59, 0x05,  0xA3, 0x07,  0xC3, 0x06,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 12,      0xA0, 0x02,  0x90, 0x06,  0x22, 0x05,  0xED, 0x07,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 13,      0x3C, 0x03,  0x14, 0x07,  0xDB, 0x03,  0xDC, 0x08,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 14,      0xEA, 0x03,  0x25, 0x07,  0xBF, 0x03,  0xDC, 0x08,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

    // A 5 ~ 8 번칸 위치
      1, 15,      0xC9, 0x01,  0x7C, 0x06,  0x12, 0x05,  0x2B, 0x08,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 16,      0x41, 0x02,  0x6F, 0x07,  0x73, 0x03,  0x9E, 0x08,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 17,      0xED, 0x02,  0x35, 0x08,  0xB9, 0x02,  0x9E, 0x08,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 18,      0xDC, 0x03,  0xB6, 0x08,  0x58, 0x02,  0x9D, 0x08,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

  //////////  아래의 8 줄:  Dropbox B(정지선 뒤 장애물 상자를 두 번째 8칸함으로 쓰는 경우)의 1번에서 8번까지의 칸 위치
  //          ID 19~26 = DROPBOX_B_8POSE_START_IDX(18) + 칸 번호. Mission의 B 낙하 경로에서 사용.
  //          아래 값은 B용으로 별도 교시한 자세 데이터이다. A(ID 11~18)의 복사본으로 덮어쓰지 않는다.

    // B 1 ~ 4 번칸 위치
      1, 19,      0x3D, 0x06,  0x17, 0x07,  0x9B, 0x05,  0x21, 0x06,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 20,      0xDB, 0x06,  0xD5, 0x07,  0xFD, 0x04,  0x14, 0x06,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 21,      0xAF, 0x07,  0x62, 0x08,  0x87, 0x04,  0x19, 0x06,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 22,      0x8B, 0x08,  0x4A, 0x08,  0x91, 0x04,  0x1F, 0x06,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

    // B 5 ~ 8 번칸 위치
      1, 23,      0xBC, 0x05,  0x1E, 0x08,  0x88, 0x04,  0x18, 0x06,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 24,      0x64, 0x06,  0x1C, 0x09,  0xC4, 0x03,  0x14, 0x06,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 25,      0x79, 0x07,  0xF3, 0x09,  0x42, 0x03,  0x13, 0x06,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 26,      0xD2, 0x08,  0xD5, 0x09,  0x43, 0x03,  0x19, 0x06,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

  //////////  아래의 4 줄:  12칸(3층 x 4열) 적재함의 맨 위층(L3)용 자세. 메인 소스의 enum ManipulatorPoseID 참고
  //          층별 자세: L1= PICK_DOWN_*(ID 8~10), L2= PICK_UP_*(ID 5~7), L3= PICK_TOP_*(ID 28~30)
  //          ID 27은 Reserved. 층별 탐색: L3=31, L2=32, L1=33. ID 34~36 중앙 정렬 자세도 Reserved.

    // 27= FIND_MISSION_1SHOT (단일 자세로 미션 블록들의 색상 순서 찾기)
      1, 27,      0x69, 0x03,  0xD8, 0x09,  0xB3, 0x01,  0x3E, 0x07,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

    // (위 L3행 4칸) 28= PICK_TOP_APPROACH, 29= PICK_TOP_GRIP, 30= PICK_TOP_OUT
      1, 28,      0x00, 0x08,  0x77, 0x08,  0x8D, 0x05,  0xF8, 0x05,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 29,      0x00, 0x07,  0xF5, 0x07,  0x75, 0x05,  0xBB, 0x06,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 30,      0x00, 0x07,  0x04, 0x09,  0x4C, 0x05,  0xC3, 0x05,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

  //////////  각 층마다 다른 블록 찾는 자세 (ID 31~33)
    // 31= STORAGE_SEARCH_L3 (상층 색상 탐색)
      1, 31,      0x00, 0x08,  0x11, 0x09,  0xAB, 0x04,  0xA1, 0x05,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // 32= STORAGE_SEARCH_L2 (중층 색상 탐색)
      1, 32,      0x00, 0x08,  0x02, 0x09,  0x9C, 0x03,  0x53, 0x06,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // 33= STORAGE_SEARCH_L1 (하층 색상 탐색)
      1, 33,      0x00, 0x08,  0x93, 0x08,  0x7F, 0x01,  0xFD, 0x08,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

  //////////  경기 출발 자세와 종료 자세

    // 34= START_POSE
      1, 34,      0x13, 0x04,  0xC0, 0x09,  0xBA, 0x03,  0x0C, 0x04,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // 35= FINISH1_POSE
      1, 35,      0x00, 0x08,  0x85, 0x08,  0x32, 0x01,  0x4B, 0x09,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // 36= FINISH2_POSE
      1, 36,      0x00, 0x08,  0x12, 0x06,  0x7D, 0x03,  0xB4, 0x09,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};


// 값이 하나라도 빠지거나 많으면 여기서 컴파일 오류가 난다 (마지막 행의 누락도 잡힘). 실행 비용 없음.
static_assert( sizeof(manipulatorPoseData) == TOTAL_DATA_SIZE,
               "manipulatorPoseData must have exactly TOTAL_POSE_COUNT x 40 values" );


#define pinButton1    41  // 왼쪽 버튼 (SW1)
#define pinBuzzer     5   // 부저

void setup()
{
  Serial.begin( 115200 );

  noTone( pinBuzzer );

  Serial.print(F("\n[writeEEPROM] "));
  Serial.print( TOTAL_POSE_COUNT );
  Serial.print(F(" poses, "));
  Serial.print( TOTAL_DATA_SIZE );
  Serial.println(F(" bytes"));

  // 1. 표 정렬 검사: 각 행이 (1, ID)로 시작해야 함
  if ( ! CheckTable() )
  {
    Serial.println(F("표의 행이 어긋났습니다(값 누락). 위에 표시된 ID 행부터 확인하세요. EEPROM에 쓰지 않습니다."));
    while( 1 ) {  // 오류 알림음 반복
      tone( pinBuzzer, 262 );  delay( 300 );  noTone( pinBuzzer );  delay( 1000 );
    }
  }

  // 2. 기록할 자세 값(10진)을 보여 줌
  PrintTable();

  Serial.println(F("press button1 (or send any char) to start writing..."));

  // 프로그램이 시작되었음을 소리로 알림
  tone( pinBuzzer, 262 );  // 도
  delay( 30 );
  noTone( pinBuzzer );
  delay( 30 );
  tone( pinBuzzer, 262 );  // 도
  delay( 50 );
  noTone( pinBuzzer );

  WaitButtonOrInputChar();

  // 이제 EEPROM 쓰기가 시작됨을 소리로 알림
  tone( pinBuzzer, 262 );  // 도
  delay( 30 );
  tone( pinBuzzer, 330 );  // 미
  delay( 30 );
  tone( pinBuzzer, 392 );  // 솔
  delay( 50 );
  noTone( pinBuzzer );

  // 3. 기록. 같은 값은 건너뜀. 바이트 쓰기는 EEPROM 라이브러리가 완료를 기다리므로 지연이 필요 없음
  uint16_t changed = 0;

  for( uint16_t address = 0; address < TOTAL_DATA_SIZE; address++ )
  {
    uint8_t value = pgm_read_byte( &manipulatorPoseData[address] );

    if( EEPROM.read( address ) != value ) {
      EEPROM.write( address, value );
      changed++;
    }

    if( (address % SINGLE_POSE_DATA_SIZE) == 0 )
    {
      Serial.println();

      // 쓰기가 계속 진행중임을 소리로 알림 (40 바이트 마다)
      tone( pinBuzzer, 392 );  // 솔
      delay( 30 );
      noTone( pinBuzzer );
    }
    Serial.print('.');
  }

  // 4. 읽어서 확인
  uint16_t mismatch = 0;

  for( uint16_t address = 0; address < TOTAL_DATA_SIZE; address++ )
  {
    if( EEPROM.read( address ) != pgm_read_byte( &manipulatorPoseData[address] ) )
    {
      if( mismatch < 5 ) {
        Serial.print(F("\nverify mismatch at address "));
        Serial.print( address );
        Serial.print(F(" (ID "));
        Serial.print( address / SINGLE_POSE_DATA_SIZE + 1 );
        Serial.print(F(")"));
      }
      mismatch++;
    }
  }

  Serial.print(F("\nWrite completed. changed bytes= "));
  Serial.print( changed );

  if( mismatch == 0 )
  {
    Serial.print(F(", verify OK (ID 1~"));
    Serial.print(TOTAL_POSE_COUNT);
    Serial.println(F(")."));

    // 끝났음을 소리로 알림
    tone( pinBuzzer, 392 );  // 솔
    delay( 30 );
    tone( pinBuzzer, 330 );  // 미
    delay( 30 );
    tone( pinBuzzer, 262 );  // 도
    delay( 50 );
    noTone( pinBuzzer );
  }
  else
  {
    Serial.print(F(", verify FAILED: "));
    Serial.print( mismatch );
    Serial.println(F(" bytes differ. 전원을 다시 켜고 한 번 더 쓰세요."));

    while( 1 ) {  // 오류 알림음 반복
      tone( pinBuzzer, 262 );  delay( 300 );  noTone( pinBuzzer );  delay( 1000 );
    }
  }
}


void loop()
{
  delay( 1000 );
}


// 표의 각 행이 (플래그 1, ID n)으로 시작하는지 확인. 값이 빠져 행이 밀리면 여기서 걸린다.
bool CheckTable()
{
  bool ok = true;

  for( uint8_t id = 1; id <= TOTAL_POSE_COUNT; id++ )
  {
    uint16_t base = (uint16_t)(id - 1) * SINGLE_POSE_DATA_SIZE;
    uint8_t flag  = pgm_read_byte( &manipulatorPoseData[base] );
    uint8_t rowId = pgm_read_byte( &manipulatorPoseData[base + 1] );

    if( flag != 1  ||  rowId != id )
    {
      Serial.print(F("  ID "));
      Serial.print( id );
      Serial.print(F(" row misaligned: flag= "));
      Serial.print( flag );
      Serial.print(F(", id= "));
      Serial.println( rowId );
      ok = false;
    }
  }

  return ok;
}

// 표의 관절값(LB, HB -> 10진)을 ID별로 출력. poseMon의 10진 출력, Mission/Manipulator.h의 관절 한계와 비교용
void PrintTable()
{
  Serial.println(F("  ID :    m1    m2    m3    m4"));

  for( uint8_t id = 1; id <= TOTAL_POSE_COUNT; id++ )
  {
    uint16_t base = (uint16_t)(id - 1) * SINGLE_POSE_DATA_SIZE + 2;

    Serial.print(F("  "));
    if( id < 10 ) Serial.print(' ');
    Serial.print( id );
    Serial.print(F(" : "));

    for( uint8_t m = 0; m < 4; m++ )
    {
      uint16_t value = pgm_read_byte( &manipulatorPoseData[base + 2*m] )
                     | ((uint16_t)pgm_read_byte( &manipulatorPoseData[base + 2*m + 1] ) << 8);
      if( value < 1000 ) Serial.print(' ');
      if( value < 100 )  Serial.print(' ');
      if( value < 10 )   Serial.print(' ');
      Serial.print(F("  "));
      Serial.print( value );
    }
    Serial.println();
  }
}

// SW1 버튼이 눌리거나 시리얼로 글자가 들어올 때까지 대기
void WaitButtonOrInputChar()
{
  while( 1 )
  {
    if( digitalRead( pinButton1 ) == 0 ) {   // 버튼 눌림
      delay( 20 );
      return;
    }
    if( Serial.available() ) {               // 시리얼 모니터에 문자 입력
      while( Serial.read() >= 0 ) { delay( 2 ); }
      return;
    }
    delay( 10 );
  }
}
