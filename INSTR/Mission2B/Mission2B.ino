//////////////  WCRC 2026 Mission2B TODO
//
//  LEVEL1의 이동 -> 탐색 -> 집기 -> 낙하 -> 복귀 흐름과 변수명을 그대로 사용한다.
//  LEVEL2 추가: 첫 자세에서 지시 순서 읽기, 정면 PSD 보정, CAM 좌우 보정.
//
//  적재함에서 블록을 찾을 때 매번 CELL1부터 다시 찾는다.
//  A/B 혼합 낙하, B에서 CELL2 보정 후 CELL1 복귀, 홀수/짝수 컬럼 층 순서.
//

#include "Debug.h"
#include "Devices.h"
#include "Manipulator.h"
#include "Mobilebase.h"
#include <Adafruit_NeoPixel.h>

//////////////  동작 선택

#define REAL_WCRC_GAME_MODE       true   // true: 테스트 기능 제외, 경기 동작만
#define USE_SHORT_CUT_PASS        false  // true: 적재함 탐색부터 시작 (실제 경기에서는 false로 설정해야 함!)

#define RUN_TOP_ROW_SEARCH        true   // false: 맨 위층(L3)은 찾지 않음
#define SEARCH_SWEEP_LIMIT        1      // 정방향 탐색 후 반대 방향으로 한 번 더 찾고 다음 색상으로 진행

#define STOP_BEFORE_DROP          false  // true: 낙하 자세에서 그리퍼를 열기 전에 버튼 대기 (자세 확인용)
#define PAUSE_AT_EACH_STEP        false  // LEVEL1과 같은 단계별 설명용 대기
#define RUN_CAM_X_ALIGN           true   // 찾은 블록의 화면 x로 좌우 위치를 보정

//////////////  장치

#define NUM_PIXELS    1
Adafruit_NeoPixel pixels = Adafruit_NeoPixel( NUM_PIXELS, RGB_LED_PIN, NEO_GRBW + NEO_KHZ800 );

Pixy2SPI_SS pixy;
Dynamixel2Arduino dxl( DXL_SERIAL, DXL_DIR_PIN );

//////////////  속도

const int32_t MOVE_MM_PER_SEC   = MOBILEBASE_SPEED_FAST;  // 긴 이동: 공통 160 mm/s
const int32_t SIDE_MM_PER_SEC   = MOBILEBASE_SPEED_FAST;  // 좌진/우진
const int32_t SEARCH_MM_PER_SEC = MOBILEBASE_SPEED_FAST;  // 적재함 앞 칸 이동
const int32_t MOVE_ALIGN_SPEED  = MOBILEBASE_SPEED_FAST;  // PSD 보정 (오차에 비례한 속도의 상한)

const int PICK_ACT_TIME        = 350;
const int MOVE_LAYER_POSE_TIME = 350;  // 층 자세 변경 시간은 집기 시간과 구분
const int GRIP_ACT_TIME        = 350;
const int DROP_INDEX_POS_TIME  = 500;
const int CHANGE_NEXT_POS_TIME = 500;


//////////////  PSD 목표값

#define PSD_OBSTACLE_FR             590
#define PSD_OBSTACLE_MOV_TOLERANCE  2

#define PSD_MISSION_SL              476
#define PSD_MISSION_MOV_TOLERANCE   3     // 미션지시 Zone, Dropbox 옆, 낙하 위치의 SL 보정에 함께 사용

#define PSD_DROPBOXA_SEARCH_SL      558
#define PSD_DROPBOXA_DROP_SL        632

#define PSD_STORAGE_FR              315
#define PSD_STORAGE_MOV_TOLERANCE   2

// B 정면과 B 복귀의 CELL2 경유 보정. 값은 mm가 아니라 PSD의 ADC 값이다.
#define PSD_DROPBOX_B_FRONT_AVG     591
#define PSD_DROPBOX_B_ROT_ADJUST    6
#define PSD_DROPBOX_B_MOV_TOLERANCE 2
#define PSD_DROPBOX_B_ROT_TOLERANCE 2
#define PSD_STORAGE_FRONT_AVG      311   // CELL2에서 (FL 307 + FR 315) / 2
#define PSD_STORAGE_ROT_ADJUST     8
#define PSD_STORAGE_ROT_TOLERANCE  2
#define PSD_STORAGE_CELL2_SL       200
#define PSD_STORAGE_CELL2_SL_TOLERANCE 2

#define MAX_MOV_ALIGN_REPEAT_COUNT  3     // 거리 보정: 과잉 보정 방지를 위해 반복하는 최대 횟수
#define MAX_MOV_ALIGN_1TIME_RETRY   5     // 거리 보정: 1회 조정에서 측정+이동을 반복하는 최대 횟수
#define ALIGN_OUT_REPEAT_DELAY_MS   10


//////////////  이동 거리 (mm)

#define mm_F_BLINKING_to_OBSTACLE   (float) 462.0
#define mm_L_OBSTACLE_to_MISSION    (float) 304.0
#define mm_F_MISSION_to_DROPBOX     (float) 858.0

#define mm_R_DROPBOX_to_CELL1       (float) 110.0
#define mm_R_CELL1_to_CELL2         (float) 67.0
#define mm_R_CELL2_to_CELL3         (float) 68.0
#define mm_R_CELL3_to_CELL4         (float) 67.0

#define mm_L_CELL4_to_CELL3         (mm_R_CELL3_to_CELL4 - 3.0)
#define mm_L_CELL3_to_CELL2         mm_R_CELL2_to_CELL3
#define mm_L_CELL2_to_CELL1         mm_R_CELL1_to_CELL2

#define mm_L_CELL1_to_DROPSIDE      ((float) 115.0 + 9)
#define mm_R_DROPSIDE_to_CELL1      mm_L_CELL1_to_DROPSIDE

// 이하 디롭박스 B 낙하와 종료 관련 MODIFIED ADDED
#define mm_B_DROPBAX_to_FINISH      1320
#define mm_F_TO_DROPBOX_B          (float) 366.0  // B 정면 직진/복귀 후진
#define mm_F_DROPBOX_B_to_CELL2    ((mm_L_CELL4_to_CELL3) + (mm_L_CELL3_to_CELL2) - 8.0f)  // 복귀 전진
#define mm_R_DROPBOX_B_to_FINISH   290.0f  // 박스 B에서 종료 시 우진 거리
#define mm_F_DROPBOX_B_to_FINISH   (230.0f + 250.0f + 200.0f + 270.0f)  // 종료 전진 구간


//////////////  매니퓰레이터 자세 (EEPROM ID)

enum ManipulatorPoseID {
  FIND_MISSION_BLOCKS1  =  1,
  FIND_MISSION_BLOCKS2,
  FRONT_MOVE,
  STORAGE_SEARCH,

  PICK_UP_APPROACH,     // 5
  PICK_UP_GRIP,
  PICK_UP_OUT,

  PICK_DOWN_APPROACH,   // 8
  PICK_DOWN_GRIP,
  PICK_DOWN_OUT,

  // 11~18: Dropbox A 1~8번 칸, 19~26: Dropbox B 1~8번 칸

  FIND_MISSION_1SHOT    = 27,  // 단일 자세로 미션 블록들의 색상 순서 찾기
  PICK_TOP_APPROACH,    // 28
  PICK_TOP_GRIP,
  PICK_TOP_OUT,

  STORAGE_SEARCH_L3     = 31,  // L3 색상 탐색 자세 MODIFIED
  STORAGE_SEARCH_L2,          // 32: L2 색상 탐색 자세
  STORAGE_SEARCH_L1,          // 33: L1 색상 탐색 자세

  START_POSE,    // 34: 출발 자세 MODIFIED
  FINISH1_POSE,  // 35: 종료 자세1
  FINISH2_POSE,  // 36: 종료 자세2

  // 12칸 적재함의 층별 이름 (같은 자세의 다른 이름)
  //   L1(맨 아래층)= PICK_DOWN_*, L2(가운데층)= PICK_UP_*, L3(맨 위층)= PICK_TOP_*
  PICK_L1_APPROACH = PICK_DOWN_APPROACH, PICK_L1_GRIP = PICK_DOWN_GRIP, PICK_L1_OUT = PICK_DOWN_OUT,
  PICK_L2_APPROACH = PICK_UP_APPROACH,   PICK_L2_GRIP = PICK_UP_GRIP,   PICK_L2_OUT = PICK_UP_OUT,
  PICK_L3_APPROACH = PICK_TOP_APPROACH,  PICK_L3_GRIP = PICK_TOP_GRIP,  PICK_L3_OUT = PICK_TOP_OUT
};

#define DROPBOX_8POSE_START_IDX     10
#define DROPBOX_B_8POSE_START_IDX   18

const uint8_t storageSearchPoseIds[3] = { STORAGE_SEARCH_L1, STORAGE_SEARCH_L2, STORAGE_SEARCH_L3 };
const uint8_t pickApproachPoseIds[3]  = { PICK_DOWN_APPROACH, PICK_UP_APPROACH, PICK_TOP_APPROACH };
const uint8_t pickGripPoseIds[3]      = { PICK_DOWN_GRIP,     PICK_UP_GRIP,     PICK_TOP_GRIP };
const uint8_t pickOutPoseIds[3]       = { PICK_DOWN_OUT,      PICK_UP_OUT,      PICK_TOP_OUT };


//////////////  미션 블록 (경기 전 추첨 결과를 블록 하나당 { 색상, 낙하 칸 } 한 줄씩 입력)
//              색상: 1=빨 2=주 3=노 4=초 5=파 6=보. 같은 색은 줄을 따로 쓰고 최대 2개.
//              낙하 칸: A는 1~8, B는 8+칸번호(9~16). 목적지는 겹치지 않게 적는다.
//              줄 순서는 미션지시 순서와 무관하다. 지시 순서는 카메라로 읽는다.

const uint8_t MISSION_COLOR_COUNT = 6;
const uint8_t MAX_MISSION_BLOCKS_PER_COLOR = 2;
const uint8_t MAX_MISSION_BLOCKS  = 8;
const uint8_t DROPBOX_CELL_COUNT = 8;
const uint8_t MISSION_DROP_CELL_COUNT = DROPBOX_CELL_COUNT * 2;

struct MissionBlockAssignment {
  uint8_t signature;
  uint8_t dropCell;
};

#if USE_SHORT_CUT_PASS  // MODIFIED
  const MissionBlockAssignment missionAssignments[] = {
    { 1, 8+1 },
    { 3, 8+2 },
    { 4, 8+3 },
    { 5, 8+4 }  
  };
#else
  const MissionBlockAssignment missionAssignments[] = {
    { 1, 1 },    // 첫 번째 빨강 -> A1
    { 1, 8+5 },  // 두 번째 빨강 -> B5
    { 3, 4 },    // 노랑 -> A4
    { 4, 8+2 },  // 첫 번째 초록 -> B2
    { 4, 5 },    // 두 번째 초록 -> A5
    { 5, 6 }     // 파랑 -> A6
  };
#endif

const int KNOWN_MISSION_BLOCK_COUNT = sizeof(missionAssignments) / sizeof(missionAssignments[0]);
static_assert( KNOWN_MISSION_BLOCK_COUNT >= 1 && KNOWN_MISSION_BLOCK_COUNT <= MAX_MISSION_BLOCKS, "Enter 1..8 mission blocks" );

// 카메라가 읽은 순서로 같은 배열을 채운다.
uint8_t targetBlockSigmaps[MAX_MISSION_BLOCKS] = { 0 };
int     goalPositions[MAX_MISSION_BLOCKS] = { 0 };
int     nMissionBlocksFound = 0;

int     searchIndex = 0;
int     searchSweepCount = 0;
uint8_t lastCheckedLayerPoseId = 0;


//////////////  카메라

#define CAM_FIND_REPEAT_COUNT       3    // 미션지시 Zone 색상 순서 읽기 횟수
#define CAM_MISSION_READ_ATTEMPTS   3    // 지시 순서를 확정하기 위한 최대 읽기 시도 횟수 MODIFIED
#define CAM_SEARCH_REPEAT_COUNT     5    // 적재함 칸에서 블록 찾기 횟수

#define CAM_DETECTING_LOOP_DELAY                30
#define MISSION_BLOCK_DUPLICATE_X_TOLERANCE     10     // 같은 색이 이 거리 안에 보이면 한 블록
const int MISSION_1ST_CAM_SINGLE_BLOCK_WIDTH_PX = 45;  // 첫 자세의 블록 한 개 폭: 실측 후 조정
const int MISSION_JOINED_BLOCK_WIDTH_PERCENT = 160;    // 1.6배 이상이면 붙은 같은 색 두 개

#define PIXY_SET_POINT_X            (158 + 8)   // 그리퍼 중심에 해당하는 화면 x
#define PIXY_TOLERANCE_MM           2

constexpr float SETPOINT_X_PX_TO_MM = 0.30f;

#define MIN_PIXY_SEARCH_X           (158 - 80)
#define MAX_PIXY_SEARCH_X           (158 + 80)
#define MIN_PIXY_SEARCH_Y           (208 / 4)
#define MAX_PIXY_SEARCH_Y           (208 * 3 / 4)

#define LAYER_POSE_SETTLE_MS        80

int16_t slPSD, valPSD;


///////////////////////////////////////////////////////////////////////////////////////////  공용

// waitDelay가 0이면 버튼이나 시리얼 입력까지 기다리고, 0보다 크면 그 시간(ms)만 살피고 지나간다
int DelayOrWaitButtonOrInputChar( int waitDelay= 0 );
int DelayOrWaitButtonOrInputChar( int waitDelay )
{
  if( waitDelay > 0 )
  {
    const int msWaitLoopInterval = 10;  // 10ms 단위로 인터럽트 입력이 있는지 체크하면서 대기
    int i, msLoopCount = waitDelay / msWaitLoopInterval;

    for( i=0; i<msLoopCount; i++ )
    {
      if( digitalRead(SW1_PIN) == 0 ) {  // 버튼1 눌림
        delay( 20 );
        return  1;
      }
      if( DEBUG_SERIAL.available() ) {  // 단축 경로는 실전 모드에서도 Enter(CR/LF)를 포함한 시리얼 입력 허용
        while( DEBUG_SERIAL.read() > 0 ) { delay( 10 ); }
        return  2;
      }
      delay( msWaitLoopInterval );
    }
    return 0;
  }

  while( 1 )
  {
    if( digitalRead(SW1_PIN) == 0 ) {  // 버튼1 눌림
      delay( 20 );
      return  -1;
    }
    if( DEBUG_SERIAL.available() ) {  // 단축 경로는 실전 모드에서도 Enter(CR/LF)를 포함한 시리얼 입력 허용
      while( DEBUG_SERIAL.read() > 0 ) { delay( 10 ); }
      return -2;
    }
    delay( 10 );
  }
}

void PauseIfStepMode( const __FlashStringHelper* nextStep )
{
#if PAUSE_AT_EACH_STEP && (! REAL_WCRC_GAME_MODE)
  Serial.print(F("Next: "));
  Serial.println( nextStep );
  Serial.println(F("Press button1 to continue..."));
  DelayOrWaitButtonOrInputChar();
#else
  (void) nextStep;
#endif
}

void Beep( int cnt )
{
  for( int i = 0; i < cnt; i++ ) {
    tone( BUZZER_PIN, 262 );  delay( 150 );
    noTone( BUZZER_PIN );     delay( 100 );
  }
  noTone( BUZZER_PIN );
}

void Beeps( int cnt )
{
  for( int i = 0; i < cnt; i++ ) {
    tone( BUZZER_PIN, 392 );  delay( 50 );
    noTone( BUZZER_PIN );     delay( 50 );
  }
  noTone( BUZZER_PIN );
}

void ShowColor( int r, int g, int b )
{
  pixels.setPixelColor( 0, pixels.Color(r, g, b) );
  pixels.show();
}

void ShowBlockColor( int sigColor )
{
  switch( sigColor ) {
    case 1: ShowColor( 255, 0, 0 );    break;
    case 2: ShowColor( 255, 64, 32 );  break;
    case 3: ShowColor( 255, 255, 0 );  break;
    case 4: ShowColor( 0, 255, 0 );    break;
    case 5: ShowColor( 0, 0, 255 );    break;
    case 6: ShowColor( 255, 0, 255 );  break;
    default: ShowColor( 0, 0, 0 );     break;
  }
}


///////////////////////////////////////////////////////////////////////////////////////////  PSD 보정

// 좌측면 PSD로 좌우 거리 맞추기. 측정+이동을 한 걸음씩 최대 15회(바깥 3 × 안쪽 5) 반복한다.
// 내부 함수가 "더 이동 불필요"를 돌려주거나(허용 안에 들어옴. 허용 밖의 작은 오차는 최소 명령 10으로 계속 보정) 시간 상한이면 끝난다.
// 허용 오차 안까지 맞춘다는 보장은 없으므로 결과 위치는 줄자·psdMon으로 확인한다.
void PerformLeftMoveAlign( int psdTarget, int psdTolerance, int32_t moveSpeed = MOVE_ALIGN_SPEED );
void PerformLeftMoveAlign( int psdTarget, int psdTolerance, int32_t moveSpeed )
{
  ChangeMobilebaseMode2VelocityControlMode( dxl );

  const unsigned long startTime = millis();
  bool bTimeout = false;

  for( int i = 0; i < MAX_MOV_ALIGN_REPEAT_COUNT; i++ )
  {
    int nAdjusted = 0;

    for( int j = 0; j < MAX_MOV_ALIGN_1TIME_RETRY; j++ )
    {
      if( millis() - startTime >= ALIGN_TIMEOUT ) {
        bTimeout = true;
        break;
      }

      GetValueFromSideLeftPSDSensor( &slPSD );

      if( ! MoveAlignWithLeftSidePSD( dxl, psdTarget - slPSD, psdTolerance, PSD_MOVE_CONTROL_RATIO, i == 0 && j == 0, moveSpeed ) )
        break;

      nAdjusted++;
    }

    if( bTimeout || nAdjusted == 0 )
      break;

    delay( ALIGN_OUT_REPEAT_DELAY_MS );
  }

  SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode( dxl );
}

//////////  우측면 PSD로 로봇과 박스 사이의 이격 거리 조정

void  PerformRightMoveAlign( int psdTarget, int psdTolerance, int retryCount= MAX_MOV_ALIGN_1TIME_RETRY,
                             int moveSpeed= MOBILEBASE_SPEED, float controlRatio= PSD_MOVE_CONTROL_RATIO );

void  PerformRightMoveAlign( int psdTarget, int psdTolerance, int retryCount, int moveSpeed, float controlRatio )  // MODIFIED ADDED
{
  ChangeMobilebaseMode2VelocityControlMode( dxl );

  unsigned long alignStartTime = millis();
  bool bTimeout = false;
  int16_t srPSD = 0;

  for( int i = 0; i < MAX_MOV_ALIGN_REPEAT_COUNT; i++ )
  {
    int nAdjusted = 0;

    for( int j = 0; j < retryCount; j++ )
    {
      if( millis() - alignStartTime >= ALIGN_TIMEOUT ) {
        bTimeout = true;
        break;
      }

      GetValueFromSideRightPSDSensor( &srPSD );

      // 우측 센서는 좌측 센서와 이동 방향이 반대이므로 오차의 부호를 반전한다.
      bool needsMore = MoveAlignWithLeftSidePSD( dxl, srPSD - psdTarget, psdTolerance, controlRatio, i == 0 && j == 0, moveSpeed );

      if( ! needsMore )
        break;

      nAdjusted++;
    }

    if( bTimeout || nAdjusted == 0 )
      break;

    delay( ALIGN_OUT_REPEAT_DELAY_MS );
  }

  SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode( dxl );
}

// LEVEL2 추가: LEVEL1의 측면 보정과 같은 방식(최대 15회, 같은 종료 조건)으로 전방우측 PSD의 앞뒤 거리도 맞춘다.
void PerformFrontMoveAlign( int psdTarget, int psdTolerance )
{
  ChangeMobilebaseMode2VelocityControlMode( dxl );

  const unsigned long startTime = millis();
  bool bTimeout = false;

  for( int i = 0; i < MAX_MOV_ALIGN_REPEAT_COUNT; i++ )
  {
    int nAdjusted = 0;

    for( int j = 0; j < MAX_MOV_ALIGN_1TIME_RETRY; j++ )
    {
      if( millis() - startTime >= ALIGN_TIMEOUT ) {
        bTimeout = true;
        break;
      }

      GetValueFromFrontRightPSDSensor( &valPSD );

      if( ! MoveAlignWithFrontRightPSD( dxl, psdTarget - valPSD, psdTolerance, PSD_MOVE_CONTROL_RATIO, i == 0 && j == 0, MOVE_ALIGN_SPEED ) )
        break;

      nAdjusted++;
    }

    if( bTimeout || nAdjusted == 0 )
      break;

    delay( ALIGN_OUT_REPEAT_DELAY_MS );
  }

  SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode( dxl );
}


// B 정면 / 복귀 CELL2: 전방 평균 거리와 FR-FL 차이를 함께 맞추는 간단 연속 보정.
// 이동·회전 허용 오차 2 사용. 종료는 허용 오차 도달뿐 아니라 반복/시간 한도도 포함한다.
void PerformMoveRotateAlign( int frontAvgTarget, int moveTolerance, int rotateTolerance, int rotationAdjust )
{
  ChangeMobilebaseMode2VelocityControlMode( dxl );

  const unsigned long startTime = millis();
  bool bTimeout = false;
  int16_t flPSD, frPSD;

  for( int i = 0; i < MAX_MOV_ALIGN_REPEAT_COUNT; i++ )
  {
    int nAdjusted = 0;
    for( int j = 0; j < MAX_MOV_ALIGN_1TIME_RETRY; j++ )
    {
      if( millis() - startTime >= ALIGN_TIMEOUT ) { bTimeout = true; break; }

      GetValueFromFrontPSDSensors( &flPSD, &frPSD );
      if( millis() - startTime >= ALIGN_TIMEOUT ) { bTimeout = true; break; }

      if( ! RotateAlignWithFront2PSD( dxl,
              frontAvgTarget - (flPSD + frPSD) / 2, (frPSD - flPSD) - rotationAdjust,
              moveTolerance, rotateTolerance, PSD_MOVE_CONTROL_RATIO, PSD_ROTATE_CONTROL_RATIO,
              i == 0 && j == 0, MOVE_ALIGN_SPEED ) )
        break;
      nAdjusted++;
    }

    if( bTimeout || nAdjusted == 0 ) 
      break;

    delay( ALIGN_OUT_REPEAT_DELAY_MS );
  }

  SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode( dxl );
}


///////////////////////////////////////////////////////////////////////////////////////////  미션 블록 추첨표

uint8_t GetMissionBlockCountForSignature( uint8_t signature )
{
  uint8_t count = 0;

  for( int i = 0; i < KNOWN_MISSION_BLOCK_COUNT; i++ )
    if( missionAssignments[i].signature == signature ) count++;
  return count;
}

// LEVEL2 추가: 카메라에 추첨표의 색만 요청하기 위한 비트마스크
uint8_t GetMissionSignatureMask()
{
  uint8_t sigmap = 0;

  for( int i = 0; i < KNOWN_MISSION_BLOCK_COUNT; i++ ) {
    const uint8_t sig = missionAssignments[i].signature;
    if( sig >= 1 && sig <= MISSION_COLOR_COUNT ) sigmap |= 1 << (sig - 1);
  }
  return sigmap;
}

bool AreMissionAssignmentsValid()
{
  uint16_t usedCells = 0;

  for( int i = 0; i < KNOWN_MISSION_BLOCK_COUNT; i++ )
  {
    const uint8_t sig  = missionAssignments[i].signature;
    const uint8_t cell = missionAssignments[i].dropCell;

    if( sig < 1 || sig > MISSION_COLOR_COUNT || cell < 1 || cell > MISSION_DROP_CELL_COUNT ||
        GetMissionBlockCountForSignature( sig ) > MAX_MISSION_BLOCKS_PER_COLOR )
      return false;

    const uint16_t cellBit = uint16_t(1U) << (cell - 1);
    if( usedCells & cellBit )
      return false;

    usedCells |= cellBit;
  }
  return true;
}

void StopForMissionError( const __FlashStringHelper* message )
{
  Serial.println(message);

  ChangeMobilebaseMode2VelocityControlMode( dxl );
  SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );

  tone( BUZZER_PIN, 262 );  delay( 1000 );  noTone( BUZZER_PIN );

  while( 1 ) {
    ShowColor( 255, 0, 0 );  delay( 1000 );
    ShowColor( 0, 0, 0 );    delay( 1000 );
  }
}

void StopForInvalidMissionBlockInfo()
{
  StopForMissionError(F("Invalid missionAssignments table: check colors and unique A1..8 / B9..16 cells. Stopped."));
}

// 목적지 번호는 1~16, EEPROM 자세는 A11~18 / B19~26. 잘못된 목적지는 0.
uint8_t GetDropPoseId( int destination )
{
  if( destination < 1 || destination > MISSION_DROP_CELL_COUNT ) return 0;

  const bool useDropboxB = destination > DROPBOX_CELL_COUNT;
  const int dropCell = useDropboxB ? destination - DROPBOX_CELL_COUNT : destination;

  return (useDropboxB ? DROPBOX_B_8POSE_START_IDX : DROPBOX_8POSE_START_IDX) + dropCell;
}

bool IsMissionPosePresent( uint8_t poseId )
{
  const ManipulatorPose pose = ReadManipulatorPresentPoseToEEPROM(poseId); // 읽기만 수행

  if( pose.isTherePoseData == 1 && pose.id == poseId ) return true;

  Serial.print(F("Missing pose ID "));
  Serial.println(poseId);
  return false;
}

// 출발 전에 운반 자세와 이번에 실제로 사용하는 낙하 자세만 확인한다. EEPROM에는 쓰지 않는다.
bool AreRequiredDropboxPosesPresent()
{
  if( ! IsMissionPosePresent(STORAGE_SEARCH) ) return false;

  for( int i = 0; i < KNOWN_MISSION_BLOCK_COUNT; i++ )
    if( ! IsMissionPosePresent(GetDropPoseId(missionAssignments[i].dropCell)) ) return false;
  return true;
}

bool IsUsableMissionBlockSequence( const uint8_t* sigs, int count )
{
  if( count < 1 || count > KNOWN_MISSION_BLOCK_COUNT || count > MAX_MISSION_BLOCKS )
    return false;

  uint8_t colorCounts[MISSION_COLOR_COUNT + 1] = { 0 };

  for( int i = 0; i < count; i++ ) {
    const uint8_t sig = sigs[i];
    if( sig < 1 || sig > MISSION_COLOR_COUNT || ++colorCounts[sig] > GetMissionBlockCountForSignature( sig ) )
      return false;
  }
  return true;
}

bool IsCompleteMissionBlockSequence( const uint8_t* sigs, int count )
{
  return count == KNOWN_MISSION_BLOCK_COUNT && IsUsableMissionBlockSequence(sigs, count);
}

// 지시 순서(sigs)대로 찾을 색상을 정하고, 각 블록에 그 색의 아직 쓰지 않은 낙하 칸을 하나씩 배정
bool SetMissionBlockTargets( const uint8_t* sigs, int count )
{
  nMissionBlocksFound = 0;
  
  for( int block = 0; block < MAX_MISSION_BLOCKS; block++ ) {
    targetBlockSigmaps[block] = 0;
    goalPositions[block] = 0;
  }

  if( ! AreMissionAssignmentsValid() || ! IsUsableMissionBlockSequence(sigs, count) )
    return false;

  bool assigned[MAX_MISSION_BLOCKS] = { false };

  for( int block = 0; block < count; block++ )
  {
    targetBlockSigmaps[block] = 1 << (sigs[block] - 1);
    goalPositions[block] = 0;

    for( int entry = 0; entry < KNOWN_MISSION_BLOCK_COUNT; entry++ ) {
      if( assigned[entry] || missionAssignments[entry].signature != sigs[block] )
        continue;
      
      assigned[entry] = true;
      goalPositions[block] = missionAssignments[entry].dropCell;
      break;
    }
  }

  nMissionBlocksFound = count;

  // 설정 완료된 블록의 색상 ID 순서를 시리얼 모니터에 출력
  Serial.print(F("[MISSION] color IDs: "));
  for( int i = 0; i < nMissionBlocksFound; i++ ) {
    if( i > 0 ) Serial.print(F(" -> "));
    Serial.print(sigs[i]);
  }
  Serial.println();

  return true;
}

// 적재함 단축 테스트에서만 배열 순서를 사용한다. 경기 중 인식 실패의 대체 순서로 사용하지 않는다. MODIFIED
void SetMissionBlockInfo()
{
  uint8_t sigs[MAX_MISSION_BLOCKS] = { 0 };

  for( int i = 0; i < KNOWN_MISSION_BLOCK_COUNT; i++ )
    sigs[i] = missionAssignments[i].signature;

  if( ! SetMissionBlockTargets(sigs, KNOWN_MISSION_BLOCK_COUNT) )
    StopForInvalidMissionBlockInfo();
}

// 지시존에서 읽은 좌->우 순서만 적용한다. missionAssignments는 색상별 목적지 배정에만 사용한다.
void ApplyMissionBlockSequence( const uint8_t* sigs, int count )
{
  if( SetMissionBlockTargets(sigs, count) )
    return;

  // 기존 지시 순서 인식 실패 시에도 배열 순서로 대체하지 않음
  StopForMissionError(F("No usable mission order from camera. Check instruction blocks/camera. Stopped."));
}


///////////////////////////////////////////////////////////////////////////////////////////  미션지시 Zone 색상 순서

// 왼쪽부터 읽는다. 같은 색이 두 개 배정되고 한 덩어리로 보일 때만 폭으로 두 블록을 구분한다. MODIFIED
int ReadMissionFrame( uint8_t* sigs )
{
  struct Seen { uint8_t sig; int x; int width; } seen[MAX_MISSION_BLOCKS];
  int count = 0;

  const uint8_t missionSigmap = GetMissionSignatureMask();

  if( missionSigmap == 0 || pixy.ccc.getBlocks( true, missionSigmap ) < 0 )
    return -1;

  for( int j = 0; j < pixy.ccc.numBlocks; j++ )
  {
    const uint16_t sig = pixy.ccc.blocks[j].m_signature;
    const int x = pixy.ccc.blocks[j].m_x;
    const int width = pixy.ccc.blocks[j].m_width;

    if( sig < 1 || sig > MISSION_COLOR_COUNT || (missionSigmap & (1 << (sig - 1))) == 0 ||
        width <= 0 || pixy.ccc.blocks[j].m_height <= 0 )
      continue;

    bool duplicate = false;

    for( int k = 0; k < count; k++ ) {
      if( seen[k].sig == sig && abs(seen[k].x - x) <= MISSION_BLOCK_DUPLICATE_X_TOLERANCE ) {
        if( width > seen[k].width ) seen[k].width = width;
        duplicate = true;
        break;
      }
    }

    if( duplicate )
      continue;

    if( count >= MAX_MISSION_BLOCKS )
      return 0;

    seen[count].sig = sig;
    seen[count].x = x;
    seen[count].width = width;
    count++;
  }

  for( int a = 0; a < count - 1; a++ )
    for( int b = a + 1; b < count; b++ )
      if( seen[a].x > seen[b].x ) { Seen t = seen[a]; seen[a] = seen[b]; seen[b] = t; }

  int blockCount = 0;

  for( int i = 0; i < count; i++ ) {
    int sameColorBlobs = 0;

    for( int j = 0; j < count; j++ )
      if( seen[j].sig == seen[i].sig ) sameColorBlobs++;

    const int copies = sameColorBlobs == 1 && GetMissionBlockCountForSignature(seen[i].sig) == 2 &&
                       seen[i].width * 100L >= MISSION_1ST_CAM_SINGLE_BLOCK_WIDTH_PX * MISSION_JOINED_BLOCK_WIDTH_PERCENT ? 2 : 1;

    if( blockCount + copies > MAX_MISSION_BLOCKS )
      return 0;

    for( int copy = 0; copy < copies; copy++ )
      sigs[blockCount++] = seen[i].sig;
  }

  return blockCount;
}

// 최대 3차례의 모든 후보를 보관한다. 전체 순서가 2프레임 이상 같으면 확정한다. MODIFIED
// 확정하지 못하면 인식 개수가 가장 많고, 같은 개수 중 반복 검출이 가장 많은 좌->우 순서를 사용한다.
int ReadMissionBlockSequence( uint8_t* result )
{
  const int maxFrames = CAM_FIND_REPEAT_COUNT * CAM_MISSION_READ_ATTEMPTS;
  uint8_t frames[maxFrames][MAX_MISSION_BLOCKS] = { 0 };

  int     frameCounts[maxFrames] = { 0 };
  int     validFrames = 0;
  int     bestIdx = -1, bestSupport = -1;

  for( int attempt = 0; attempt < CAM_MISSION_READ_ATTEMPTS; attempt++ )
  {
    for( int sample = 0; sample < CAM_FIND_REPEAT_COUNT; sample++ )
    {
      const int n = ReadMissionFrame( frames[validFrames] );

      if( IsUsableMissionBlockSequence(frames[validFrames], n) ) {
        frameCounts[validFrames] = n;
        validFrames++;
      }
      delay( CAM_DETECTING_LOOP_DELAY );
    }

    bestIdx = -1;
    bestSupport = -1;

    for( int i = 0; i < validFrames; i++ )
    {
      int support = 0;

      for( int j = 0; j < validFrames; j++ ) {
        if( frameCounts[j] != frameCounts[i] ) continue;

        bool sameOrder = true;
        for( int block = 0; block < frameCounts[i]; block++ )
          if( frames[j][block] != frames[i][block] ) { sameOrder = false; break; }
        if( sameOrder ) support++;
      }

      if( bestIdx < 0 || frameCounts[i] > frameCounts[bestIdx] ||
          (frameCounts[i] == frameCounts[bestIdx] && support > bestSupport) ) {
        bestIdx = i;
        bestSupport = support;
      }
    }

    if( bestIdx >= 0 && bestSupport >= 2 &&
        IsCompleteMissionBlockSequence(frames[bestIdx], frameCounts[bestIdx]) ) break;
  }

  if( bestIdx < 0 )
    return 0;

  for( int i = 0; i < frameCounts[bestIdx]; i++ )
    result[i] = frames[bestIdx][i];

  return frameCounts[bestIdx];
}


///////////////////////////////////////////////////////////////////////////////////////////  적재함 블록 찾기

uint8_t GetFirstStorageSearchPoseId()
{
  return storageSearchPoseIds[(RUN_TOP_ROW_SEARCH ? 3 : 2) - 1]; // CELL1의 첫 탐색 층
}

bool RunStorageLayerPose( uint8_t poseId, int32_t poseTimeMs )
{
  if( lastCheckedLayerPoseId == poseId )
    return true;

  const ManipulatorPose pose = RunManipulatorPoseWithPoseDataInEEPROM( dxl, poseId, poseTimeMs, -360.0 );

  if( pose.isTherePoseData != 1 ) {
    Serial.print(F("Missing pose ID "));
    Serial.println( poseId );
    lastCheckedLayerPoseId = 0;
    return false;
  }

  delay( poseTimeMs );
  lastCheckedLayerPoseId = poseId;
  return true;
}

// 현재 자세에서 목표 색 블록이 화면 안에 3프레임 이상 보이면 x 기준 중앙값 검출을 돌려준다. 좌표를 CAM 보정에 쓴다
// (평균을 쓰면 같은 색 두 블록이 번갈아 잡힐 때 빈 자리를 가리킬 수 있어 실제 검출 하나를 고른다)
bool CamFindBlockAndGetStablePosition( int blockIndex, BlockPosition* result ) // MODIFIED
{
  BlockPosition candidates[CAM_SEARCH_REPEAT_COUNT];
  int count = 0;

  for( int i = 0; i < CAM_SEARCH_REPEAT_COUNT; i++ )
  {
    const int8_t camResult = pixy.ccc.getBlocks( true, targetBlockSigmaps[blockIndex] );
    if( camResult < 0 ) {
      delay( CAM_DETECTING_LOOP_DELAY );
      continue;
    }

    for( int j = 0; j < pixy.ccc.numBlocks; j++ )
    {
      const int x = pixy.ccc.blocks[j].m_x;
      const int y = pixy.ccc.blocks[j].m_y;

      if( x < MIN_PIXY_SEARCH_X || x > MAX_PIXY_SEARCH_X || y < MIN_PIXY_SEARCH_Y || y > MAX_PIXY_SEARCH_Y ) {
        continue;
      }

      candidates[count].x = x;
      candidates[count].y = y;
      candidates[count].w = pixy.ccc.blocks[j].m_width;
      candidates[count].h = pixy.ccc.blocks[j].m_height;
      count++;
      break;
    }

    delay( CAM_DETECTING_LOOP_DELAY );
  }

  if( count < 3 ) {
    return false;
  }

  for( int a = 0; a < count - 1; a++ )
    for( int b = a + 1; b < count; b++ )
      if( candidates[a].x > candidates[b].x ) { BlockPosition t = candidates[a]; candidates[a] = candidates[b]; candidates[b] = t; }

  *result = candidates[count / 2];
  return true;
}

void PickBlockAtLayer( int layer ) // MODIFIED
{
  const int idx = layer - 1;

  lastCheckedLayerPoseId = 0;

  RunManipulatorPoseWithPoseDataInEEPROM( dxl, pickApproachPoseIds[idx], PICK_ACT_TIME, 0.0 );
  delay( PICK_ACT_TIME );

  RunManipulatorPoseWithPoseDataInEEPROM( dxl, pickGripPoseIds[idx], PICK_ACT_TIME, 0.0 );
  delay( PICK_ACT_TIME );

  CloseGripper( pixy );
  delay( GRIP_ACT_TIME );

  RunManipulatorPoseWithPoseDataInEEPROM( dxl, pickOutPoseIds[idx], PICK_ACT_TIME, 0.0 );
  delay( PICK_ACT_TIME );

  RunManipulatorPoseWithPoseDataInEEPROM( dxl, STORAGE_SEARCH, PICK_ACT_TIME, 0.0 );
  delay( PICK_ACT_TIME );
}

// 홀수 CELL은 위->아래, 짝수 CELL은 아래->위. 왕복 방향과 무관하게 이웃 컬럼의 층 자세를 이어 쓴다.
// 반환: 잡은 층(1~3), 0=못 찾음.
int CamFindBlockAndPickup( int blockIndex, int columnNo ) // MODIFIED
{
  const int lastLayer = RUN_TOP_ROW_SEARCH ? 3 : 2;

  for( int scanIndex = 0; scanIndex < lastLayer; scanIndex++ )
  {
    const int layer = columnNo % 2 == 1 ? lastLayer - scanIndex : scanIndex + 1;

    if( ! RunStorageLayerPose( storageSearchPoseIds[layer - 1], MOVE_LAYER_POSE_TIME ) ) {
      continue;
    }
    delay( LAYER_POSE_SETTLE_MS );

    BlockPosition pos;

    if( ! CamFindBlockAndGetStablePosition( blockIndex, &pos ) ) {
      continue;
    }

#if RUN_CAM_X_ALIGN  // TODO-01
    // 블록 찾은 후 집기 전에 CAM으로 좌우 보정.
    const float dsxm = (pos.x - PIXY_SET_POINT_X) * SETPOINT_X_PX_TO_MM;

    if( abs(dsxm) > PIXY_TOLERANCE_MM )
    {
      DriveDistanceSmoothWithInternalAcceleration( dxl, abs(dsxm), dsxm > 0 ? MOVE_RIGHTWARD : MOVE_LEFTWARD, SEARCH_MM_PER_SEC );
      delay( 100 );

      if( ! CamFindBlockAndGetStablePosition( blockIndex, &pos ) ||
          abs((pos.x - PIXY_SET_POINT_X) * SETPOINT_X_PX_TO_MM) > PIXY_TOLERANCE_MM )
      {
        DriveDistanceSmoothWithInternalAcceleration( dxl, abs(dsxm), dsxm > 0 ? MOVE_LEFTWARD : MOVE_RIGHTWARD, SEARCH_MM_PER_SEC );
        delay( 100 );
        continue;
      }
    }
#endif

    PickBlockAtLayer( layer );
    return layer;
  }

  return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////  낙하와 복귀

// 낙하 자세가 유효할 때만 그리퍼를 열고, 접힌 자세 완료 후 true를 반환한다.
bool DropPickedBlockIntoDropbox( int blockIndex )
{
  if( blockIndex < 0 || blockIndex >= nMissionBlocksFound || blockIndex >= MAX_MISSION_BLOCKS )
    return false;

  const int destination = goalPositions[blockIndex];
  const uint8_t dropPoseId = GetDropPoseId(destination);

  if( dropPoseId == 0 || ! IsMissionPosePresent(dropPoseId) || ! IsMissionPosePresent(STORAGE_SEARCH) )
    return false;

  const bool useDropboxB = destination > DROPBOX_CELL_COUNT;

  PauseIfStepMode( F("drop block into Dropbox") );

  // 드롭박스 A의 경우, 1번이나 8번 칸 같이 거리가 먼 경우는 자세를 취하는 시간을 약간 더 늘려준다. TODO-02
  const ManipulatorPose dropPose = RunManipulatorPoseWithPoseDataInEEPROM( dxl, dropPoseId, DROP_INDEX_POS_TIME );
  if( dropPose.isTherePoseData != 1 ) return false;
  delay( DROP_INDEX_POS_TIME );


#if STOP_BEFORE_DROP && (! REAL_WCRC_GAME_MODE)
  const int dropCell = useDropboxB ? destination - DROPBOX_CELL_COUNT : destination;
  Serial.print(F("Stopped before drop at Dropbox "));
  Serial.print(useDropboxB ? 'B' : 'A');
  Serial.print(F(" cell #")); Serial.print(dropCell);
  Serial.print(F(" pose ID ")); Serial.println(dropPoseId);
  Beeps( 2 );
  DelayOrWaitButtonOrInputChar();
#endif

  OpenGripper( pixy );
  delay( GRIP_ACT_TIME / 2 );

  if( ! useDropboxB ) {
    // A의 기존 접기 순서와 필수 대기는 유지한다.
    const ManipulatorPose carryPose = RunManipulatorPoseWithPoseDataInEEPROM( dxl, STORAGE_SEARCH, CHANGE_NEXT_POS_TIME, -90.0 );
    if( carryPose.isTherePoseData != 1 ) return false;
    delay( CHANGE_NEXT_POS_TIME );
  }

  const ManipulatorPose carryPose = RunManipulatorPoseWithPoseDataInEEPROM( dxl, STORAGE_SEARCH, CHANGE_NEXT_POS_TIME, 0.0 );
  if( carryPose.isTherePoseData != 1 ) return false;
  delay( CHANGE_NEXT_POS_TIME ); // B도 팔이 접힌 뒤 후진/회전한다.

  lastCheckedLayerPoseId = 0;
  return true;
}

void BackToFinishZoneAndGameOver()
{
  pixy.setLamp( 0, 0 );

  PauseIfStepMode( F("back to Finish Zone") );
  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_B_DROPBAX_to_FINISH, MOVE_BACKWARD, MOVE_MM_PER_SEC );

  Beep( 2 );
  while( 1 )
  {
    ShowColor( 255, 255, 0 );  delay( 1000 );
    ShowColor( 0, 0, 0 );      delay( 1000 );
  }
}

void FinishFromDropboxB() // 마지막 B 낙하 후 우진 1회, 전진 1회로 FINISH 진입 MODIFIED ADDED
{
  pixy.setLamp( 0, 0 );

  // DropPickedBlockIntoDropbox()에서 이미 팔을 접고 자세 완료까지 기다렸다.
  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_R_DROPBOX_B_to_FINISH, MOVE_RIGHTWARD, SEARCH_MM_PER_SEC );
  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_F_DROPBOX_B_to_FINISH, MOVE_FORWARD, SEARCH_MM_PER_SEC );

  ChangeMobilebaseMode2VelocityControlMode( dxl );
  SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );

  Beep( 2 );
  while( 1 )
  {
    ShowColor( 255, 255, 0 );  delay( 1000 );
    ShowColor( 0, 0, 0 );      delay( 1000 );
  }
}

// CELL4 기준 B 접근 후진 거리. 최종 Mission의 CELL1/2/3/4
float GetBackwardDistanceToDropboxB( int columnNo )  // MODIFIED
{
  float d = 0.0f;
  if( columnNo <= 3 ) d += mm_L_CELL4_to_CELL3;
  if( columnNo <= 2 ) d += mm_L_CELL3_to_CELL2;
  if( columnNo <= 1 ) d += (mm_L_CELL2_to_CELL1 - 6);
  return d;
}

float GetLeftDistanceToDropside( int columnNo )
{
  float leftDistance = mm_L_CELL1_to_DROPSIDE;
  if( columnNo >= 2 ) leftDistance += mm_L_CELL2_to_CELL1;
  if( columnNo >= 3 ) leftDistance += mm_L_CELL3_to_CELL2;
  if( columnNo >= 4 ) leftDistance += mm_L_CELL4_to_CELL3;
  return leftDistance;
}


/////////////////////////////////////////////// 드롭 박스 B 및 FINISH 경로 관련 코드 추가/수정 MODIFIED ADDED

void GotoDropboxB( int columnNo )
{
  PauseIfStepMode( F("go to Dropbox B") );

  // 집기 때의 카메라 X 보정은 칸 중앙 맞춤이므로 여기서 되돌리거나 거리에서 빼지 않는다.
  RotateLeft90Degree( dxl, MOVE_MM_PER_SEC, 0.0f );  // 1차 좌회전

  // CLO1 에서 블록 찾아낸 경우에는 1차 좌회전 후 전면 박스A와 회전 보정 추가 TODO-03

  if( columnNo < 4 )
    DriveDistanceSmoothWithInternalAcceleration( dxl, GetBackwardDistanceToDropboxB(columnNo), MOVE_BACKWARD, SIDE_MM_PER_SEC );

  RotateLeft90Degree( dxl, MOVE_MM_PER_SEC, 0.0f );  // 2차 좌회전

  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_F_TO_DROPBOX_B, MOVE_FORWARD, MOVE_MM_PER_SEC );

  // 박스 B 앞에서 좌우 정렬 보정 TODO-04

  //DelayOrWaitButtonOrInputChar();  // PPP
}

// B에서 CELL2까지의 공통 복귀. 다음 블록 탐색을 위한 CELL1 복귀에 사용한다.
void ReturnFromDropboxBToCell2()
{
  PauseIfStepMode( F("return from Dropbox B to CELL2") );

  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_F_TO_DROPBOX_B, MOVE_BACKWARD, MOVE_MM_PER_SEC );
  RotateRight90Degree( dxl, MOVE_MM_PER_SEC, 1.0f );  // 1차 우회전

  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_F_DROPBOX_B_to_CELL2, MOVE_FORWARD, SIDE_MM_PER_SEC );

  // 전방 우측 PSD와 드롭박스 A와 거리 보정 TODO-05

  // 우측 PSD와 적재함의 거리 보정 TODO-06

  RotateRight90Degree( dxl, MOVE_MM_PER_SEC, 1.5f );  // 2차 우회전

  //DelayOrWaitButtonOrInputChar();  // PPP

  // 두 회전이 끝난 뒤에만 팔을 펼치고, 완료된 자세에서 보정한다.
  if( ! RunStorageLayerPose(GetFirstStorageSearchPoseId(), MOVE_LAYER_POSE_TIME) ) {
    StopForMissionError(F("Cannot prepare storage search pose after B return. Stopped."));
    return;
  }

  // 적재함 앞에서 전면 좌우 비틀림 보정 TODO-07
  
  // 좌측 PSD로 거리 보정 TODO-08
}

void ReturnFromDropboxBToCell1() // 다음 블록을 찾을 때는 기존대로 CELL1까지 복귀한다.
{
  ReturnFromDropboxBToCell2();

  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_L_CELL2_to_CELL1, MOVE_LEFTWARD, SEARCH_MM_PER_SEC );
}

void GotoStorageForSearchingNextBlock( bool fromDropboxB )
{
  searchSweepCount = 0;

  if( fromDropboxB ) {
    ReturnFromDropboxBToCell1(); // 다음 블록 탐색은 CELL1부터 시작한다.
  } else {
    if( ! RunStorageLayerPose(GetFirstStorageSearchPoseId(), MOVE_LAYER_POSE_TIME) ) {
      StopForMissionError(F("Cannot prepare CELL1 search pose after A drop. Stopped."));
      return;
    }

    PauseIfStepMode( F("go back to CELL1 for next block") );
    DriveDistanceSmoothWithInternalAcceleration( dxl, mm_R_DROPSIDE_to_CELL1, MOVE_RIGHTWARD, SEARCH_MM_PER_SEC );
  }

  pixy.setLamp( 1, 1 );
}

// 현재 CELL에서 A 옆 DROPSIDE 종료선까지 한 번에 좌진한다.
void FinishFromStorageColumn( int columnNo )
{
  pixy.setLamp( 0, 0 );
  PauseIfStepMode( F("finish from storage via DROPSIDE") );

  // 현재 1열 앞이면 전면 거리 보정 TODO-09

  const ManipulatorPose carryPose = RunManipulatorPoseWithPoseDataInEEPROM( dxl, STORAGE_SEARCH, 1000, -90.0 );
  if( carryPose.isTherePoseData != 1 ) {
    StopForMissionError(F("Cannot prepare finish carry pose. Stopped."));
    return;
  }

  lastCheckedLayerPoseId = 0;
  delay( 1000 );

  DriveDistanceSmoothWithInternalAcceleration( dxl, GetLeftDistanceToDropside(columnNo), MOVE_LEFTWARD, SEARCH_MM_PER_SEC );

  // Dropbox 옆 탐색선과 낙하 위치에서 좌측면 PSD로 거리 맞추기 TODO-10

  BackToFinishZoneAndGameOver();
}

void FinishFromCell1() // CELL1에서 탐색을 포기할 때의 기존 종료 경로 유지
{
  FinishFromStorageColumn( 1 );
}

// 컬럼 번호로 기존 A 좌진 거리와 B 경로를 선택한다. 다음 미션은 공통 함수로 CELL1부터 다시 찾는다.
void DropAndReturn( int columnNo )
{
  if( columnNo < 1 || columnNo > 4 || searchIndex < 0 ||
      searchIndex >= nMissionBlocksFound || searchIndex >= MAX_MISSION_BLOCKS ) {
    StopForMissionError(F("Invalid column or mission index before delivery. Stopped."));
    return;
  }

  const int destination = goalPositions[searchIndex];
  if( GetDropPoseId(destination) == 0 ) {
    StopForInvalidMissionBlockInfo();
    return;
  }

  const bool useDropboxB = destination > DROPBOX_CELL_COUNT;

  pixy.setLamp( 0, 0 );

  if( useDropboxB ) {
    GotoDropboxB( columnNo );
  } else {
    const ManipulatorPose carryPose = RunManipulatorPoseWithPoseDataInEEPROM( dxl, STORAGE_SEARCH, 1000, -90.0 );
    if( carryPose.isTherePoseData != 1 ) {
      StopForMissionError(F("Cannot prepare A carry pose. Stopped."));
      return;
    }

    const float leftDistance = GetLeftDistanceToDropside(columnNo); // 좌진 거리 유지

    // 컬럼4에서 집어 박스 A로 운반할 때만 컬럼2에서 추가 보정 TODO-11
    DriveDistanceSmoothWithInternalAcceleration( dxl, leftDistance, MOVE_LEFTWARD, SEARCH_MM_PER_SEC );

    // Dropbox 옆 탐색선과 낙하 위치에서 좌측면 PSD로 거리 맞추기 TODO-12
  }

  if( ! DropPickedBlockIntoDropbox(searchIndex) ) {
    StopForMissionError(F("Drop aborted: check destination, drop pose and carry pose. Stopped."));
    return;
  }

  searchIndex++;

  if( searchIndex >= nMissionBlocksFound ) {
    if( useDropboxB ) {
      FinishFromDropboxB(); // B에서 우진 후 전진하여 종료
    } else {
      BackToFinishZoneAndGameOver();
    }
    return;
  }

  GotoStorageForSearchingNextBlock( useDropboxB );
}


///////////////////////////////////////////////////////////////////////////////////////////  MAIN

void setup()
{
  InitDebug();

  InitMotorCommunication( dxl );
  while( ! InitManipulator( dxl ) ) {}
  while( ! InitMobilebase( dxl ) ) {}

  InitPSD();
  InitPixy( pixy );
  pixy.setLamp( 0, 0 );

  pixels.begin();
  pixels.setBrightness( 50 );
  pixels.show();

  SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );
  OpenGripper( pixy );
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode( dxl );

  ShowColor( 0, 0, 0 );

  if( ! AreMissionAssignmentsValid() )
    StopForInvalidMissionBlockInfo();
  if( ! AreRequiredDropboxPosesPresent() )
    StopForMissionError(F("Missing required Dropbox/carry pose before start. Check EEPROM. Stopped."));

  RunManipulatorPoseWithPoseDataInEEPROM( dxl, START_POSE, 2000, -360.0 );  // FRONT_MOVE, FIND_MISSION_BLOCKS1, FIND_MISSION_1SHOT MODIFIED
  delay( 2000 );

  Serial.println(F("=> Press button1 to start game..."));
  Beeps( 1 );
  DelayOrWaitButtonOrInputChar();

  Serial.println(F("-----> Game started!"));
  Beep( 2 );


#if (! USE_SHORT_CUT_PASS)  // ====================================================================================

  //////////  장애물까지 전진, 전방 거리 맞추기

  PauseIfStepMode( F("forward to obstacle") );

  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_F_BLINKING_to_OBSTACLE, MOVE_FORWARD, MOVE_MM_PER_SEC );

  PerformFrontMoveAlign( PSD_OBSTACLE_FR, PSD_OBSTACLE_MOV_TOLERANCE );


  //////////  미션 블록 색상 순서 읽기 자세

  RunManipulatorPoseWithPoseDataInEEPROM( dxl, FIND_MISSION_1SHOT, 600, -360.0 );  // START_POSE, FRONT_MOVE, FIND_MISSION_BLOCKS1 MODIFIED


  //////////  미션지시 Zone으로 좌진, 좌측면 거리 맞추기

  PauseIfStepMode( F("leftward to Mission Zone") );

  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_L_OBSTACLE_to_MISSION, MOVE_LEFTWARD, SIDE_MM_PER_SEC );

  PerformLeftMoveAlign( PSD_MISSION_SL, PSD_MISSION_MOV_TOLERANCE );


  //////////  미션 블록 색상 순서 읽기 (첫 번째 자세에서 확정될 때까지 최대 3회 시도)

  pixy.setLamp( 1, 1 );
  delay( 150 );

  //DelayOrWaitButtonOrInputChar(); // PPP

  uint8_t missionSigs[MAX_MISSION_BLOCKS] = { 0 };
  const int missionCount = ReadMissionBlockSequence( missionSigs ); // 함수 안에서 최대 3차례 읽고 최상의 후보를 반환 MODIFIED

  pixy.setLamp( 0, 0 );

  ApplyMissionBlockSequence( missionSigs, missionCount );

  ShowBlockColor( nMissionBlocksFound );

#if (! REAL_WCRC_GAME_MODE)
  Beeps( nMissionBlocksFound );
#endif

  //DelayOrWaitButtonOrInputChar(); // PPP


  //////////  적재함 옆 탐색선까지 길게 전진

  PauseIfStepMode( F("long forward to storage line") );

  if( ! RunStorageLayerPose(GetFirstStorageSearchPoseId(), 1000) ) {
    StopForMissionError(F("Cannot prepare initial CELL1 search pose. Stopped."));
    return;
  }

  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_F_MISSION_to_DROPBOX, MOVE_FORWARD, MOVE_MM_PER_SEC );


#else // ------------------------------------------------------------------------------------------------------


  SetMissionBlockInfo();

  if( ! RunStorageLayerPose(GetFirstStorageSearchPoseId(), 1000) ) {
    StopForMissionError(F("Cannot prepare shortcut CELL1 search pose. Stopped."));
    return;
  }
  delay( 1000 );

#endif  //===============================================================================   USE_SHORT_CUT_PASS

  // DelayOrWaitButtonOrInputChar(); // PPP


  //////////  Dropbox 옆에서 좌측면 거리 맞추기 후, CELL1로 우진

  PerformLeftMoveAlign( PSD_DROPBOXA_SEARCH_SL, PSD_MISSION_MOV_TOLERANCE );  // 보정 보강 TODO-13

  pixy.setLamp( 1, 1 );

  PauseIfStepMode( F("rightward to CELL1") );

  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_R_DROPBOX_to_CELL1, MOVE_RIGHTWARD, SEARCH_MM_PER_SEC );

  //DelayOrWaitButtonOrInputChar(); // PPP 적재함 1번 또는 2번 열에서 모든 블록의 색상을 인식해야 함! (각 층 마다 테스트 포함)


  ///////////////////////////////////////////////////////////////////////////////////////////
  //  CELL1 -> CELL4 -> CELL1 왕복하며 searchIndex 블록 찾기. 각 CELL에서 전방 거리를 맞춘 뒤 찾는다 (CELL4는 거리가 멀어 생략)

  while( 1 )
  {
    searchSweepCount++; // 현재 색상으로 정방향과 역방향을 모두 탐색한다. MODIFIED

    //////////  CELL1

    PerformFrontMoveAlign( PSD_STORAGE_FR, PSD_STORAGE_MOV_TOLERANCE );

    if( CamFindBlockAndPickup( searchIndex, 1 ) ) {
      DropAndReturn( 1 );
      continue;
    }

    DriveDistanceSmoothWithInternalAcceleration( dxl, mm_R_CELL1_to_CELL2, MOVE_RIGHTWARD, SEARCH_MM_PER_SEC );

    //////////  CELL2

    PerformFrontMoveAlign( PSD_STORAGE_FR, PSD_STORAGE_MOV_TOLERANCE );

    //  CELL2 앞에서 전면의 좌우 PSD로 로봇 회전 비틀림 정렬 TODO-14

    if( CamFindBlockAndPickup( searchIndex, 2 ) ) {
      DropAndReturn( 2 );
      continue;
    }

    DriveDistanceSmoothWithInternalAcceleration( dxl, mm_R_CELL2_to_CELL3, MOVE_RIGHTWARD, SEARCH_MM_PER_SEC );

    //////////  CELL3

    PerformFrontMoveAlign( PSD_STORAGE_FR, PSD_STORAGE_MOV_TOLERANCE );

    //  CELL3 앞에서 전면의 좌우 PSD로 로봇 회전 비틀림 정렬 TODO-15

    if( CamFindBlockAndPickup( searchIndex, 3 ) ) {
      DropAndReturn( 3 );
      continue;
    }

    DriveDistanceSmoothWithInternalAcceleration( dxl, mm_R_CELL3_to_CELL4, MOVE_RIGHTWARD, SEARCH_MM_PER_SEC );

    //////////  CELL4

    if( CamFindBlockAndPickup( searchIndex, 4 ) ) {
      DropAndReturn( 4 );
      continue;
    }

    //////////  CELL4 -> CELL3 -> CELL2 -> CELL1 돌아오며 다시 찾기

    DriveDistanceSmoothWithInternalAcceleration( dxl, mm_L_CELL4_to_CELL3, MOVE_LEFTWARD, SEARCH_MM_PER_SEC );

    PerformFrontMoveAlign( PSD_STORAGE_FR, PSD_STORAGE_MOV_TOLERANCE );

    //  CELL3 앞에서 전면의 좌우 PSD로 로봇 회전 비틀림 정렬 TODO-16

    if( CamFindBlockAndPickup( searchIndex, 3 ) ) {
      DropAndReturn( 3 );
      continue;
    }

    DriveDistanceSmoothWithInternalAcceleration( dxl, mm_L_CELL3_to_CELL2, MOVE_LEFTWARD, SEARCH_MM_PER_SEC );

    PerformFrontMoveAlign( PSD_STORAGE_FR, PSD_STORAGE_MOV_TOLERANCE );

    //  CELL2 앞에서 전면의 좌우 PSD로 로봇 회전 비틀림 정렬 TODO-17

    if( CamFindBlockAndPickup( searchIndex, 2 ) ) {
      DropAndReturn( 2 );
      continue;
    }

    DriveDistanceSmoothWithInternalAcceleration( dxl, mm_L_CELL2_to_CELL1, MOVE_LEFTWARD, SEARCH_MM_PER_SEC );

    // 역방향의 마지막 CELL1까지 같은 색상을 확인한 뒤에만 다음 색상으로 넘어간다.

    // 전면 거리 보정 TODO-18

    if( CamFindBlockAndPickup( searchIndex, 1 ) ) {
      DropAndReturn( 1 );
      continue;
    }

    if( searchSweepCount >= SEARCH_SWEEP_LIMIT ) {
      Beeps( 3 );
      searchIndex++;
      searchSweepCount = 0;
      if( searchIndex >= nMissionBlocksFound ) {
        FinishFromCell1();
        return;
      }
    }
  }
}

void loop()
{
  delay( 10 );
}
