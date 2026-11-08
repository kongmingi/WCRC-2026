//////////////  WCRC 2026 Mission2B TODO
//
//  LEVEL1의 이동 -> 탐색 -> 집기 -> 낙하 -> 복귀 흐름과 변수명을 그대로 사용한다.
//  LEVEL2 추가: 첫 자세에서 지시 순서 읽기, 정면 PSD 보정, CAM 좌우 보정.
//
//  적재함에서 블록을 찾을 때 매번 CELL1부터 다시 찾는다.
//  A/B 혼합 낙하, B에서 CELL2 보정 후 CELL1 복귀, 홀수/짝수 컬럼 층 순서.
//

#include <setjmp.h>
#include "Debug.h"
#include "Devices.h"
#include "Manipulator.h"
#include "Mobilebase.h"
#include <Adafruit_NeoPixel.h>
#include "Tuning.h"

//////////////  장치

#define NUM_PIXELS    1
Adafruit_NeoPixel pixels = Adafruit_NeoPixel( NUM_PIXELS, RGB_LED_PIN, NEO_GRBW + NEO_KHZ800 );

Pixy2 pixy;
Dynamixel2Arduino dxl( DXL_SERIAL, DXL_DIR_PIN );
// t5 2부터 Finish 이동 종료까지 Pixy 램프 유지
bool t52LampActive = false;

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
  ORDER_L_POSE,
  ORDER_R_POSE,

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

const int KNOWN_MISSION_BLOCK_COUNT = sizeof(missionAssignments) / sizeof(missionAssignments[0]);
static_assert( KNOWN_MISSION_BLOCK_COUNT >= 1 && KNOWN_MISSION_BLOCK_COUNT <= MAX_MISSION_BLOCKS, "Enter 1..8 mission blocks" );

// 카메라가 읽은 순서로 같은 배열을 채운다.
uint8_t targetBlockSigmaps[MAX_MISSION_BLOCKS] = { 0 };
int     goalPositions[MAX_MISSION_BLOCKS] = { 0 };
int     nMissionBlocksFound = 0;

int     searchIndex = 0;
int     searchSweepCount = 0;
uint8_t lastCheckedLayerPoseId = 0;


int16_t slPSD, valPSD;
bool gameOver = false;
int camLayer = 1;
int curCol = 0;


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


// Count and block-colour display use the robot's existing NeoPixel.
void ShowMissionCountColor( int count )
{
  if( count == 7 ) ShowColor( 255, 40, 120 );
  else ShowBlockColor( count );
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

  ShowColor( 255, 0, 0 );
  gameOver = true;
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

  // 지시 순서 인식 실패: 배정표 줄 순서로 진행
  Serial.println(F("No usable mission order from camera. Using missionAssignments order."));
  Beeps( 3 );
  SetMissionBlockInfo();
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


void ReadOrderSide( uint8_t poseId, bool fromLeft, bool movePose, uint8_t* slotSig )
{
  if( movePose ) {
    RunManipulatorPoseWithPoseDataInEEPROM( dxl, poseId, ORDER_POSE_MS, -360.0 );
    delay( ORDER_POSE_MS + ORDER_POSE_SETTLE_MS );
    lastCheckedLayerPoseId = 0;
  }
  uint8_t seq[ORDER_FRAMES][3] = { { 0 } };
  for( int f = 0; f < ORDER_FRAMES; f++ ) {
    int xs[8], sg[8], n = 0;
    if( pixy.ccc.getBlocks( true, 0x3F ) >= 0 )
      for( int j = 0; j < pixy.ccc.numBlocks && n < 8; j++ ) {
        const int sig = pixy.ccc.blocks[j].m_signature;
        if( sig < 1 || sig > MISSION_COLOR_COUNT ) continue;
        const int x = pixy.ccc.blocks[j].m_x;
        const int width = pixy.ccc.blocks[j].m_width;
        // Trial threshold: measured red 1 block=72, merged 2 blocks=158.
        const int joinedWidthThreshold = 115;
        Serial.print( F("[ORDER] sig=") ); Serial.print( sig );
        Serial.print( F(" x=") ); Serial.print( x );
        Serial.print( F(" w=") ); Serial.print( width );
        Serial.print( F(" copies=") ); Serial.println( width >= joinedWidthThreshold ? 2 : 1 );
        if( width >= joinedWidthThreshold && n <= 6 ) {
          xs[n] = x - width / 4; sg[n++] = sig;
          xs[n] = x + width / 4; sg[n++] = sig;
        } else {
          xs[n] = x; sg[n++] = sig;
        }
      }
    for( int a = 0; a < n - 1; a++ )
      for( int b = a + 1; b < n; b++ )
        if( xs[a] > xs[b] ) { int t = xs[a]; xs[a] = xs[b]; xs[b] = t; t = sg[a]; sg[a] = sg[b]; sg[b] = t; }
    const int m = n < 3 ? n : 3;
    for( int i = 0; i < m; i++ ) {
      if( fromLeft ) seq[f][i] = sg[i];
      else seq[f][3 - m + i] = sg[n - m + i];
    }
    delay( CAM_DETECTING_LOOP_DELAY );
  }
  int best = -1, bestVotes = 0;
  for( int f = 0; f < ORDER_FRAMES; f++ ) {
    int v = 0;
    for( int g = 0; g < ORDER_FRAMES; g++ )
      if( ! memcmp( seq[f], seq[g], 3 ) ) v++;
    if( v > bestVotes ) { bestVotes = v; best = f; }
  }
  for( int k = 0; k < 3; k++ ) slotSig[k] = best >= 0 && bestVotes >= ORDER_MIN_VOTES ? seq[best][k] : 0;
}

int ReadMissionSlots( uint8_t* result )
{
  uint8_t slot[6];
  ReadOrderSide( ORDER_L_POSE, true, false, slot );
  ReadOrderSide( ORDER_R_POSE, false, true, slot + 3 );
  int n = 0;
  Serial.print( F("[ORDER] slots:") );
  Serial2.print( F("[ORDER] slots:") );
  for( int s = 0; s < 6; s++ ) {
    Serial.print( ' ' ); Serial.print( slot[s] );
    Serial2.print( ' ' ); Serial2.print( slot[s] );
    if( slot[s] && n < MAX_MISSION_BLOCKS ) result[n++] = slot[s];
  }
  Serial.println();
  Serial2.println();
  return n;
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
  for( uint8_t k = 0; k < 3; k++ ) if( storageSearchPoseIds[k] == poseId ) camLayer = k;
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

#if RUN_CAM_X_ALIGN  // TODO-01 구현: 최대 3번 반복, 실패하면 움직인 합계만큼 되돌아감
    float dsxm = (pos.x - PIXY_SET_POINT_X[layer - 1]) * SETPOINT_X_PX_TO_MM;
    float movedMm = 0.0f;
    bool seen = true;

    for( int t = 0; t < CAM_X_ALIGN_MAX_TRY && fabs(dsxm) > PIXY_TOLERANCE_MM; t++ )
    {
      DriveDistanceSmoothWithInternalAcceleration( dxl, fabs(dsxm), dsxm > 0 ? MOVE_RIGHTWARD : MOVE_LEFTWARD, SEARCH_MM_PER_SEC );
      movedMm += dsxm;
      delay( 100 );

      if( ! CamFindBlockAndGetStablePosition( blockIndex, &pos ) ) {
        seen = false;
        break;
      }
      dsxm = (pos.x - PIXY_SET_POINT_X[layer - 1]) * SETPOINT_X_PX_TO_MM;
    }

    if( ! seen || fabs(dsxm) > PIXY_TOLERANCE_MM )
    {
      if( fabs(movedMm) > 0.5f ) {
        DriveDistanceSmoothWithInternalAcceleration( dxl, fabs(movedMm), movedMm > 0 ? MOVE_LEFTWARD : MOVE_RIGHTWARD, SEARCH_MM_PER_SEC );
        delay( 100 );
      }
      continue;
    }
#endif

    // A stable target was found: replace count display with target colour.
    for( int sig = 1; sig <= 6; sig++ )
      if( targetBlockSigmaps[blockIndex] & (1U << (sig - 1)) ) {
        ShowBlockColor( sig );
        break;
      }
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

  // TODO-02 구현: A1, A4 +200ms, A5, A8 +100ms
  int dropPoseTime = DROP_INDEX_POS_TIME;
  if( ! useDropboxB ) {
    if( destination == 1 || destination == 4 ) dropPoseTime += 200;
    else if( destination == 5 || destination == 8 ) dropPoseTime += 100;
  }

  if( destination == 1 ) DriveDistanceSmoothWithInternalAcceleration( dxl, mm_B_ZONE_A1, MOVE_BACKWARD, SIDE_MM_PER_SEC );

  const ManipulatorPose dropPose = RunManipulatorPoseWithPoseDataInEEPROM( dxl, dropPoseId, dropPoseTime );
  if( dropPose.isTherePoseData != 1 ) return false;
  delay( dropPoseTime );


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
  ShowColor( 0, 0, 0 ); // Object released in zone A/B.


  if( ! useDropboxB ) {
    // A의 기존 접기 순서와 필수 대기는 유지한다.
    const ManipulatorPose carryPose = RunManipulatorPoseWithPoseDataInEEPROM( dxl, STORAGE_SEARCH, CHANGE_NEXT_POS_TIME, -90.0 );
    if( carryPose.isTherePoseData != 1 ) return false;
    delay( CHANGE_NEXT_POS_TIME );
    if( destination == 1 ) DriveDistanceSmoothWithInternalAcceleration( dxl, mm_B_ZONE_A1, MOVE_FORWARD, SIDE_MM_PER_SEC );
  }

  const ManipulatorPose carryPose = RunManipulatorPoseWithPoseDataInEEPROM( dxl, STORAGE_SEARCH, CHANGE_NEXT_POS_TIME, 0.0 );
  if( carryPose.isTherePoseData != 1 ) return false;
  delay( CHANGE_NEXT_POS_TIME ); // B도 팔이 접힌 뒤 후진/회전한다.

  lastCheckedLayerPoseId = 0;
  return true;
}

void BackToFinishZoneAndGameOver()
{
  if( !t52LampActive ) pixy.setLamp( 0, 0 );

  PauseIfStepMode( F("back to Finish Zone") );
  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_B_DROPBAX_to_FINISH, MOVE_BACKWARD, MOVE_MM_PER_SEC );
  t52LampActive = false;
  pixy.setLamp( 0, 0 );

  Beep( 2 );
  ShowColor( 255, 255, 0 );
  gameOver = true;
}

void FinishFromDropboxB() // 마지막 B 낙하 후 우진 1회, 전진 1회로 FINISH 진입 MODIFIED ADDED
{
  if( !t52LampActive ) pixy.setLamp( 0, 0 );

  // DropPickedBlockIntoDropbox()에서 이미 팔을 접고 자세 완료까지 기다렸다.
  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_R_DROPBOX_B_to_FINISH, MOVE_RIGHTWARD, SEARCH_MM_PER_SEC );
  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_F_DROPBOX_B_to_FINISH, MOVE_FORWARD, SEARCH_MM_PER_SEC );
  t52LampActive = false;
  pixy.setLamp( 0, 0 );

  ChangeMobilebaseMode2VelocityControlMode( dxl );
  SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );

  Beep( 2 );
  ShowColor( 255, 255, 0 );
  gameOver = true;
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

  // TODO-03 구현: CELL1 에서 집었을 때만, 1차 좌회전 뒤 정면 Dropbox A 기준
  if( columnNo == 1 )
    PerformMoveRotateAlign( PSD_DROPBOXA_FRONT_AVG, PSD_DROPBOX_B_MOV_TOLERANCE, PSD_DROPBOX_B_ROT_TOLERANCE, PSD_DROPBOXA_ROT_ADJUST );

  if( columnNo < 4 )
    DriveDistanceSmoothWithInternalAcceleration( dxl, GetBackwardDistanceToDropboxB(columnNo), MOVE_BACKWARD, SIDE_MM_PER_SEC );

  RotateLeft90Degree( dxl, MOVE_MM_PER_SEC, 0.0f );  // 2차 좌회전

  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_F_TO_DROPBOX_B, MOVE_FORWARD, MOVE_MM_PER_SEC );

  // TODO-04 구현: B 정면 거리와 비틀림
  PerformMoveRotateAlign( PSD_DROPBOX_B_FRONT_AVG, PSD_DROPBOX_B_MOV_TOLERANCE, PSD_DROPBOX_B_ROT_TOLERANCE, PSD_DROPBOX_B_ROT_ADJUST );

  //DelayOrWaitButtonOrInputChar();  // PPP
}

// B에서 CELL2까지의 공통 복귀. 다음 블록 탐색을 위한 CELL1 복귀에 사용한다.
void AlignAtColumn( int c );

void ReturnFromDropboxBToCell2()
{
  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_F_TO_DROPBOX_B, MOVE_BACKWARD, MOVE_MM_PER_SEC );
  RotateRight90Degree( dxl, MOVE_MM_PER_SEC, B_RETURN_TURN1_ADJ_DEG );
  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_F_DROPBOX_B_to_CELL2, MOVE_FORWARD, SIDE_MM_PER_SEC );
  RotateRight90Degree( dxl, MOVE_MM_PER_SEC, B_RETURN_TURN2_ADJ_DEG );
  curCol = 2;
  AlignAtColumn( 2 );
}

void ReturnFromDropboxBToCell1()
{
  ReturnFromDropboxBToCell2();
  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_L_CELL2_to_CELL1, MOVE_LEFTWARD, SEARCH_MM_PER_SEC );
  curCol = 1;
  AlignAtColumn( 1 );
}

void ReturnToCell1FromDropside()
{
  RunManipulatorPoseWithPoseDataInEEPROM( dxl, FRONT_MOVE, CHANGE_NEXT_POS_TIME, -360.0 );
  delay( CHANGE_NEXT_POS_TIME );
  lastCheckedLayerPoseId = 0;
  PerformLeftMoveAlign( PSD_DROPSIDE_REF_SL, PSD_MISSION_MOV_TOLERANCE );
  PerformMoveRotateAlign( PSD_SEARCH_WALL_AVG, PSD_SEARCH_WALL_TOLERANCE, PSD_SEARCH_WALL_TOLERANCE, PSD_SEARCH_WALL_ROT_ADJUST );
  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_R_REF_to_CELL1, MOVE_RIGHTWARD, SEARCH_MM_PER_SEC );
  curCol = 1;
  AlignAtColumn( 1 );
}

void GotoStorageForSearchingNextBlock( bool fromDropboxB )
{
  searchSweepCount = 0;

  if( fromDropboxB ) {
    ReturnFromDropboxBToCell1(); // 다음 블록 탐색은 CELL1부터 시작한다.
  } else {
    ReturnToCell1FromDropside();
  }

  if( !t52LampActive ) pixy.setLamp( CAM_LAMP_ON, CAM_LAMP_ON );
}

// 현재 CELL에서 A 옆 DROPSIDE 종료선까지 한 번에 좌진한다.
void FinishFromStorageColumn( int columnNo )
{
  if( !t52LampActive ) pixy.setLamp( 0, 0 );
  PauseIfStepMode( F("finish from storage via DROPSIDE") );

  // TODO-09 구현: CELL1 앞일 때만 앞 거리
  if( columnNo == 1 )
    PerformFrontMoveAlign( PSD_STORAGE_FR, PSD_STORAGE_MOV_TOLERANCE );

  const ManipulatorPose carryPose = RunManipulatorPoseWithPoseDataInEEPROM( dxl, STORAGE_SEARCH, 1000, -90.0 );
  if( carryPose.isTherePoseData != 1 ) {
    StopForMissionError(F("Cannot prepare finish carry pose. Stopped."));
    return;
  }

  lastCheckedLayerPoseId = 0;
  delay( 1000 );

  DriveDistanceSmoothWithInternalAcceleration( dxl, GetLeftDistanceToDropside(columnNo), MOVE_LEFTWARD, SEARCH_MM_PER_SEC );

  // TODO-10 구현: 종료선에서 왼쪽 PSD
  PerformLeftMoveAlign( PSD_DROPBOXA_DROP_SL, PSD_MISSION_MOV_TOLERANCE, PSD_DROPBOXA_DROP_SPEED );

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

  if( !t52LampActive ) pixy.setLamp( 0, 0 );

  if( useDropboxB ) {
    GotoDropboxB( columnNo );
  } else {
    const ManipulatorPose carryPose = RunManipulatorPoseWithPoseDataInEEPROM( dxl, STORAGE_SEARCH, 1000, -90.0 );
    if( carryPose.isTherePoseData != 1 ) {
      StopForMissionError(F("Cannot prepare A carry pose. Stopped."));
      return;
    }

    const float leftDistance = GetLeftDistanceToDropside(columnNo) - mm_L_CELL1_to_DROPSIDE + mm_R_REF_to_CELL1;

    // TODO-11 구현: CELL4 에서 집었을 때만 CELL2 앞에서 한 번 보정
    if( columnNo == 4 ) {
      const float toCell2 = mm_L_CELL4_to_CELL3 + mm_L_CELL3_to_CELL2;
      DriveDistanceSmoothWithInternalAcceleration( dxl, toCell2, MOVE_LEFTWARD, SEARCH_MM_PER_SEC );
      PerformMoveRotateAlign( PSD_STORAGE_FR, PSD_STORAGE_MOV_TOLERANCE, PSD_STORAGE_ROT_TOLERANCE, PSD_STORAGE_COL_ROT_ADJUST[1] );
      DriveDistanceSmoothWithInternalAcceleration( dxl, leftDistance - toCell2, MOVE_LEFTWARD, SEARCH_MM_PER_SEC );
    } else {
      DriveDistanceSmoothWithInternalAcceleration( dxl, leftDistance, MOVE_LEFTWARD, SEARCH_MM_PER_SEC );
    }

    // TODO-12 구현: 낙하 직전 왼쪽 PSD
    PerformLeftMoveAlign( PSD_DROPSIDE_REF_SL, PSD_MISSION_MOV_TOLERANCE, PSD_DROPBOXA_DROP_SPEED );
    DriveDistanceSmoothWithInternalAcceleration( dxl, mm_L_REF_to_DROP, MOVE_LEFTWARD, SEARCH_MM_PER_SEC );
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

void SearchLoop()
{
  while( ! gameOver )
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

    // TODO-14 구현: CELL2 앞 비틀림 정렬
    PerformMoveRotateAlign( PSD_STORAGE_FR, PSD_STORAGE_MOV_TOLERANCE, PSD_STORAGE_ROT_TOLERANCE, PSD_STORAGE_COL_ROT_ADJUST[1] );

    if( CamFindBlockAndPickup( searchIndex, 2 ) ) {
      DropAndReturn( 2 );
      continue;
    }

    DriveDistanceSmoothWithInternalAcceleration( dxl, mm_R_CELL2_to_CELL3, MOVE_RIGHTWARD, SEARCH_MM_PER_SEC );

    //////////  CELL3

    PerformFrontMoveAlign( PSD_STORAGE_FR, PSD_STORAGE_MOV_TOLERANCE );

    // TODO-15 구현: CELL3 앞 비틀림 정렬
    PerformMoveRotateAlign( PSD_STORAGE_FR, PSD_STORAGE_MOV_TOLERANCE, PSD_STORAGE_ROT_TOLERANCE, PSD_STORAGE_COL_ROT_ADJUST[2] );

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

    // TODO-16 구현: CELL3 앞 비틀림 정렬
    PerformMoveRotateAlign( PSD_STORAGE_FR, PSD_STORAGE_MOV_TOLERANCE, PSD_STORAGE_ROT_TOLERANCE, PSD_STORAGE_COL_ROT_ADJUST[2] );

    if( CamFindBlockAndPickup( searchIndex, 3 ) ) {
      DropAndReturn( 3 );
      continue;
    }

    DriveDistanceSmoothWithInternalAcceleration( dxl, mm_L_CELL3_to_CELL2, MOVE_LEFTWARD, SEARCH_MM_PER_SEC );

    PerformFrontMoveAlign( PSD_STORAGE_FR, PSD_STORAGE_MOV_TOLERANCE );

    // TODO-17 구현: CELL2 앞 비틀림 정렬
    PerformMoveRotateAlign( PSD_STORAGE_FR, PSD_STORAGE_MOV_TOLERANCE, PSD_STORAGE_ROT_TOLERANCE, PSD_STORAGE_COL_ROT_ADJUST[1] );

    if( CamFindBlockAndPickup( searchIndex, 2 ) ) {
      DropAndReturn( 2 );
      continue;
    }

    DriveDistanceSmoothWithInternalAcceleration( dxl, mm_L_CELL2_to_CELL1, MOVE_LEFTWARD, SEARCH_MM_PER_SEC );

    // 역방향의 마지막 CELL1까지 같은 색상을 확인한 뒤에만 다음 색상으로 넘어간다.

    // TODO-18 구현: 마지막 CELL1 앞 거리
    PerformFrontMoveAlign( PSD_STORAGE_FR, PSD_STORAGE_MOV_TOLERANCE );

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

void MoveOurs( const int32_t* v, int ms );
void ArriveCell1();
const int32_t ORDER_TRAVEL_ARM[4] = { -622, 1313, 4590, 1312 };

jmp_buf stopJmp;
bool stopArmed = false;

void yield()
{
  if( ! stopArmed || digitalRead( SW2_PIN ) != LOW ) return;
  uint32_t t = micros();
  while( micros() - t < 3000 ) {}
  if( digitalRead( SW2_PIN ) != LOW ) return;
  stopArmed = false;
  for( int i = 0; i < MOBILE_DXL_ID_CNT; i++ ) dxl.writeControlTableItem( TORQUE_ENABLE, MOBILE_DXL_IDS[i], TORQUE_OFF );
  longjmp( stopJmp, 1 );
}

void RunMission()
{
  gameOver = false;
  searchIndex = 0;
  searchSweepCount = 0;
  lastCheckedLayerPoseId = 0;

  if( ! AreMissionAssignmentsValid() ) { StopForInvalidMissionBlockInfo(); return; }
  if( ! AreRequiredDropboxPosesPresent() ) { StopForMissionError(F("Missing required Dropbox/carry pose before start. Check EEPROM. Stopped.")); return; }

  RunManipulatorPoseWithPoseDataInEEPROM( dxl, START_POSE, 2000, -360.0 );
  delay( 2000 );

  Serial.println(F("-----> Game started!"));
  Beep( 2 );

#if (! USE_SHORT_CUT_PASS)  // ====================================================================================

  //////////  장애물까지 전진, 전방 거리 맞추기

  PauseIfStepMode( F("forward to obstacle") );

  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_F_BLINKING_to_OBSTACLE, MOVE_FORWARD, MOVE_MM_PER_SEC );

  PerformMoveRotateAlign( PSD_OBSTACLE_AVG, 3, 3, PSD_OBSTACLE_ROT_ADJUST );


  t52LampActive = true;
  pixy.setLamp( 1, 1 );
  MoveOurs( ORDER_TRAVEL_ARM, 600 );
  lastCheckedLayerPoseId = 0;

  PauseIfStepMode( F("leftward to Mission Zone") );

  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_L_OBSTACLE_to_MISSION, MOVE_LEFTWARD, SIDE_MM_PER_SEC );
  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_B_OBSTACLE_to_MISSION, MOVE_BACKWARD, MOVE_MM_PER_SEC );
  PerformLeftMoveAlign( PSD_MISSION_SL, PSD_MISSION_MOV_TOLERANCE );

  RunManipulatorPoseWithPoseDataInEEPROM( dxl, ORDER_L_POSE, ORDER_POSE_MS, -360.0 );
  delay( ORDER_POSE_MS + ORDER_POSE_SETTLE_MS );
  delay( 150 );

  uint8_t missionSigs[MAX_MISSION_BLOCKS] = { 0 };
  const int missionCount = ReadMissionSlots( missionSigs );
  ShowMissionCountColor( missionCount );
  ApplyMissionBlockSequence( missionSigs, missionCount );

#if (! REAL_WCRC_GAME_MODE)
  Beeps( nMissionBlocksFound );
#endif

  PauseIfStepMode( F("to CELL1") );
  ArriveCell1();

#else // ------------------------------------------------------------------------------------------------------


  SetMissionBlockInfo();

  if( ! RunStorageLayerPose(GetFirstStorageSearchPoseId(), 1000) ) {
    StopForMissionError(F("Cannot prepare shortcut CELL1 search pose. Stopped."));
    return;
  }
  delay( 1000 );

  //////////  Dropbox 옆에서 좌측면 거리 맞추기 후, CELL1로 우진

  // TODO-13 구현: 탐색선 SL 보정 두 번
  PerformMoveRotateAlign( PSD_SEARCH_WALL_AVG, PSD_SEARCH_WALL_TOLERANCE, PSD_SEARCH_WALL_TOLERANCE, PSD_SEARCH_WALL_ROT_ADJUST );
  PerformLeftMoveAlign( PSD_DROPBOXA_SEARCH_SL, PSD_MISSION_MOV_TOLERANCE );
  delay( 50 );
  PerformLeftMoveAlign( PSD_DROPBOXA_SEARCH_SL, PSD_MISSION_MOV_TOLERANCE );

  if( !t52LampActive ) pixy.setLamp( CAM_LAMP_ON, CAM_LAMP_ON );

  PauseIfStepMode( F("rightward to CELL1") );

  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_R_DROPBOX_to_CELL1, MOVE_RIGHTWARD, SEARCH_MM_PER_SEC );

  //DelayOrWaitButtonOrInputChar(); // PPP 적재함 1번 또는 2번 열에서 모든 블록의 색상을 인식해야 함! (각 층 마다 테스트 포함)


#endif  //===============================================================================   USE_SHORT_CUT_PASS

  // DelayOrWaitButtonOrInputChar(); // PPP


  ///////////////////////////////////////////////////////////////////////////////////////////
  //  CELL1 -> CELL4 -> CELL1 왕복하며 searchIndex 블록 찾기. 각 CELL에서 전방 거리를 맞춘 뒤 찾는다 (CELL4는 거리가 멀어 생략)

  curCol = 1;
  if( gameOver ) return;
  SearchLoop();
}

class Dual : public Print {
public:
  size_t write( uint8_t c ) { Serial.write( c ); Serial2.write( c ); return 1; }
};
Dual out;

int PoseIdByName( const char* n )
{
  if( isDigit( n[0] ) ) { int v = atoi( n ); return v >= 1 && v <= MANIPULATOR_POSE_ID_MAX_CNT ? v : 0; }
  if( ! strcmp( n, "travel" ) ) return START_POSE;
  if( ! strcmp( n, "carry" ) ) return STORAGE_SEARCH;
  if( ! strcmp( n, "order" ) ) return FIND_MISSION_1SHOT;
  if( ! strcmp( n, "orderl" ) ) return ORDER_L_POSE;
  if( ! strcmp( n, "orderr" ) ) return ORDER_R_POSE;
  size_t len = strlen( n );
  if( len < 3 ) return 0;
  int k = n[len - 1] - '1';
  if( ! strncmp( n, "scan", 4 ) && len == 5 && k >= 0 && k < 3 ) return storageSearchPoseIds[k];
  if( len != 3 ) return 0;
  if( n[0] == 'p' && k >= 0 && k < 3 ) {
    if( n[1] == 'a' ) return pickApproachPoseIds[k];
    if( n[1] == 'g' ) return pickGripPoseIds[k];
  }
  if( n[0] == 'z' && k >= 0 && k < 8 ) {
    if( n[1] == 'a' ) return DROPBOX_8POSE_START_IDX + k + 1;
    if( n[1] == 'b' ) return DROPBOX_B_8POSE_START_IDX + k + 1;
  }
  return 0;
}

const char* PoseName( uint8_t id, char* buf )
{
  static const char* const fixed[] = { "carry", "travel", "order", "orderl", "orderr" };
  if( id == STORAGE_SEARCH ) return fixed[0];
  if( id == START_POSE ) return fixed[1];
  if( id == FIND_MISSION_1SHOT ) return fixed[2];
  if( id == ORDER_L_POSE ) return fixed[3];
  if( id == ORDER_R_POSE ) return fixed[4];
  for( uint8_t k = 0; k < 3; k++ ) {
    if( id == storageSearchPoseIds[k] ) { sprintf( buf, "scan%d", k + 1 ); return buf; }
    if( id == pickApproachPoseIds[k] ) { sprintf( buf, "pa%d", k + 1 ); return buf; }
    if( id == pickGripPoseIds[k] ) { sprintf( buf, "pg%d", k + 1 ); return buf; }
    if( id == pickOutPoseIds[k] ) { sprintf( buf, "po%d", k + 1 ); return buf; }
  }
  if( id > DROPBOX_8POSE_START_IDX && id <= DROPBOX_8POSE_START_IDX + 8 ) { sprintf( buf, "za%d", id - DROPBOX_8POSE_START_IDX ); return buf; }
  if( id > DROPBOX_B_8POSE_START_IDX && id <= DROPBOX_B_8POSE_START_IDX + 8 ) { sprintf( buf, "zb%d", id - DROPBOX_B_8POSE_START_IDX ); return buf; }
  return "-";
}

bool CenterJ1( uint8_t id )
{
  return id == STORAGE_SEARCH || ( id >= PICK_UP_APPROACH && id <= PICK_DOWN_OUT ) || ( id >= PICK_TOP_APPROACH && id <= PICK_TOP_OUT );
}

bool ReadArm( int32_t* v )
{
  for( uint8_t i = 0; i < ARM_DXL_ID_CNT; i++ ) {
    v[i] = dxl.readControlTableItem( PRESENT_POSITION, ARM_DXL_IDS[i] );
    if( dxl.getLastLibErrCode() != DXL_LIB_OK ) { out.print( F("read fail ID ") ); out.println( ARM_DXL_IDS[i] ); return false; }
  }
  return true;
}

bool ArmTorqueOn()
{
  return dxl.readControlTableItem( TORQUE_ENABLE, ARM_DXL_IDS[0] ) == TORQUE_ON;
}

void SetArmTorque( bool on )
{
  if( on ) {
    int32_t v[4];
    if( ReadArm( v ) )
      for( uint8_t i = 0; i < ARM_DXL_ID_CNT; i++ ) dxl.writeControlTableItem( GOAL_POSITION, ARM_DXL_IDS[i], v[i] );
  }
  for( uint8_t i = 0; i < ARM_DXL_ID_CNT; i++ ) dxl.writeControlTableItem( TORQUE_ENABLE, ARM_DXL_IDS[i], on ? TORQUE_ON : TORQUE_OFF );
  out.println( on ? F("arm lock") : F("arm free") );
}

bool ReadOurs( int32_t* v )
{
  if( ! ReadArm( v ) ) return false;
  for( uint8_t i = 0; i < 4; i++ ) v[i] = ToWindow( v[i], i );
  return true;
}

void MoveOurs( const int32_t* v, int ms )
{
  if( ! ArmTorqueOn() ) SetArmTorque( true );
  setManipulatorForwardMoveWithMotorValueForSyncWrite( dxl, v[0], v[1], v[2], v[3], ms );
  delay( ms + 50 );
  lastCheckedLayerPoseId = 0;
}

void PrintArm()
{
  int32_t v[4];
  if( ! ReadOurs( v ) ) return;
  out.print( F("{ ") );
  for( uint8_t i = 0; i < 4; i++ ) { out.print( v[i] ); out.print( i < 3 ? F(", ") : F(" }\n") ); }
}

bool PlayPose( uint8_t id, int ms )
{
  ManipulatorPose p = ReadManipulatorPresentPoseToEEPROM( id );
  if( p.isTherePoseData != 1 ) { out.print( F("no pose ") ); out.println( id ); return false; }
  if( ! ArmTorqueOn() ) SetArmTorque( true );
  RunManipulatorPoseWithPoseDataInEEPROM( dxl, id, ms, CenterJ1( id ) ? 0.0 : -360.0 );
  delay( ms );
  lastCheckedLayerPoseId = 0;
  for( uint8_t k = 0; k < 3; k++ ) if( storageSearchPoseIds[k] == id ) camLayer = k;
  return true;
}

void PrintPoseLine( const char* name, const Pose& p )
{
  out.print( name ); out.print( F(" { ") );
  out.print( p.j1 ); out.print( F(", ") ); out.print( p.j2 ); out.print( F(", ") );
  out.print( p.j3 ); out.print( F(", ") ); out.print( p.j4 ); out.println( F(" }") );
}

void ListPoses()
{
  char buf[8];
  out.println( F("pose (Tuning.h numbers, after save)") );
  for( uint8_t id = 1; id <= MANIPULATOR_POSE_ID_MAX_CNT; id++ ) {
    const char* nm = PoseName( id, buf );
    if( nm[0] == '-' || ( id >= PICK_UP_OUT && id <= PICK_DOWN_OUT && id != PICK_DOWN_APPROACH && id != PICK_DOWN_GRIP ) || id == PICK_TOP_OUT ) continue;
    Pose* p = PoseSlot( id );
    if( p ) PrintPoseLine( nm, *p );
  }
}

void PrintPsd()
{
  int16_t fl, fr, sl, sr;
  GetValueFromFrontPSDSensors( &fl, &fr );
  GetValueFromSideLeftPSDSensor( &sl );
  GetValueFromSideRightPSDSensor( &sr );
  out.print( F("FL ") ); out.print( fl );
  out.print( F("  SL ") ); out.print( sl );
  out.print( F("  FR ") ); out.print( fr );
  out.print( F("  SR ") ); out.print( sr );
  out.print( F("  avg ") ); out.print( (fl + fr) / 2 );
  out.print( F("  FR-FL ") ); out.println( fr - fl );
}

bool KeyOrSw2()
{
  if( Serial.available() || Serial2.available() ) {
    delay( 20 );
    while( Serial.available() ) Serial.read();
    while( Serial2.available() ) Serial2.read();
    return true;
  }
  return digitalRead( SW2_PIN ) == LOW;
}

void SensorMonitor()
{
  out.println( F("any key: stop") );
  while( ! KeyOrSw2() ) {
    PrintPsd();
    if( pixy.ccc.getBlocks( true, 0x3F ) >= 0 ) {
      for( int i = 0; i < pixy.ccc.numBlocks; i++ ) {
        int x = pixy.ccc.blocks[i].m_x, y = pixy.ccc.blocks[i].m_y;
        bool in = x >= MIN_PIXY_SEARCH_X && x <= MAX_PIXY_SEARCH_X && y >= MIN_PIXY_SEARCH_Y && y <= MAX_PIXY_SEARCH_Y;
        out.print( F("  sig ") ); out.print( pixy.ccc.blocks[i].m_signature );
        out.print( F(" x ") ); out.print( x );
        out.print( F(" y ") ); out.print( y );
        out.print( F(" w ") ); out.print( pixy.ccc.blocks[i].m_width );
        out.print( F(" h ") ); out.print( pixy.ccc.blocks[i].m_height );
        out.print( F(" dx ") ); out.print( x - PIXY_SET_POINT_X[camLayer] );
        out.println( in ? F(" IN") : F(" out") );
      }
    } else {
      out.println( F("  pixy error") );
    }
    delay( 500 );
  }
}

uint8_t DirByChar( char c )
{
  if( c == 'B' || c == 'b' ) return MOVE_BACKWARD;
  if( c == 'L' || c == 'l' ) return MOVE_LEFTWARD;
  if( c == 'R' || c == 'r' ) return MOVE_RIGHTWARD;
  return MOVE_FORWARD;
}

void WaitEnter()
{
  out.println( F("press enter") );
  while( ! Serial.available() && ! Serial2.available() ) delay( 10 );
  delay( 20 );
  while( Serial.available() ) Serial.read();
  while( Serial2.available() ) Serial2.read();
}

void MoveTest()
{
  const char names[4] = { 'F', 'B', 'L', 'R' };
  const float mm[4] = { 500, 500, 300, 300 };
  for( uint8_t i = 0; i < 4; i++ ) {
    out.print( names[i] ); out.print( ' ' ); out.print( mm[i] ); out.println( F(" mm") );
    WaitEnter();
    DriveDistanceSmoothWithInternalAcceleration( dxl, mm[i], DirByChar( names[i] ), MOVE_MM_PER_SEC );
    out.println( F("measure real distance") );
  }
}

float ColumnPitch( int from, int to )
{
  if( to > from ) return from == 1 ? mm_R_CELL1_to_CELL2 : from == 2 ? mm_R_CELL2_to_CELL3 : mm_R_CELL3_to_CELL4;
  return from == 4 ? mm_L_CELL4_to_CELL3 : from == 3 ? mm_L_CELL3_to_CELL2 : mm_L_CELL2_to_CELL1;
}

void AlignAtColumn( int c )
{
  if( c <= 3 ) PerformFrontMoveAlign( PSD_STORAGE_FR, PSD_STORAGE_MOV_TOLERANCE );
  if( c == 2 || c == 3 ) PerformMoveRotateAlign( PSD_STORAGE_FR, PSD_STORAGE_MOV_TOLERANCE, PSD_STORAGE_ROT_TOLERANCE, PSD_STORAGE_COL_ROT_ADJUST[c - 1] );
}

void GoColumn( int target )
{
  if( curCol < 1 ) { out.println( F("column unknown: t5 3 first") ); return; }
  while( curCol != target ) {
    int next = target > curCol ? curCol + 1 : curCol - 1;
    DriveDistanceSmoothWithInternalAcceleration( dxl, ColumnPitch( curCol, next ), next > curCol ? MOVE_RIGHTWARD : MOVE_LEFTWARD, SEARCH_MM_PER_SEC );
    curCol = next;
    AlignAtColumn( curCol );
  }
  out.print( F("column ") );
  out.println( curCol );
}

void ArriveCell1()
{
  RunManipulatorPoseWithPoseDataInEEPROM( dxl, FRONT_MOVE, 1000, -360.0 );
  delay( 1000 );
  lastCheckedLayerPoseId = 0;
  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_F_MISSION_to_DROPBOX, MOVE_FORWARD, MOVE_MM_PER_SEC );
  PerformLeftMoveAlign( PSD_DROPSIDE_REF_SL, PSD_MISSION_MOV_TOLERANCE );
  PerformMoveRotateAlign( PSD_SEARCH_WALL_AVG, PSD_SEARCH_WALL_TOLERANCE, PSD_SEARCH_WALL_TOLERANCE, PSD_SEARCH_WALL_ROT_ADJUST );
  DriveDistanceSmoothWithInternalAcceleration( dxl, mm_R_REF_to_CELL1, MOVE_RIGHTWARD, SEARCH_MM_PER_SEC );
  curCol = 1;
  AlignAtColumn( 1 );
  if( !t52LampActive ) pixy.setLamp( CAM_LAMP_ON, CAM_LAMP_ON );
  PrintPsd();
}

int PickLayerColor( int layer, int color )
{
  uint8_t saved = targetBlockSigmaps[0];
  int savedIdx = searchIndex;
  targetBlockSigmaps[0] = 1 << (color - 1);
  searchIndex = 0;
  int result = 0;
  BlockPosition pos;
  if( RunStorageLayerPose( storageSearchPoseIds[layer - 1], MOVE_LAYER_POSE_TIME ) ) {
    delay( LAYER_POSE_SETTLE_MS );
    if( CamFindBlockAndGetStablePosition( 0, &pos ) ) {
      float dsxm = (pos.x - PIXY_SET_POINT_X[layer - 1]) * SETPOINT_X_PX_TO_MM;
      float movedMm = 0.0f;
      bool seen = true;
      for( int t = 0; t < CAM_X_ALIGN_MAX_TRY && fabs(dsxm) > PIXY_TOLERANCE_MM; t++ ) {
        out.print( F("cam x ") ); out.print( pos.x ); out.print( F(" move ") ); out.println( dsxm, 1 );
        DriveDistanceSmoothWithInternalAcceleration( dxl, fabs(dsxm), dsxm > 0 ? MOVE_RIGHTWARD : MOVE_LEFTWARD, SEARCH_MM_PER_SEC );
        movedMm += dsxm;
        delay( 100 );
        if( ! CamFindBlockAndGetStablePosition( 0, &pos ) ) { seen = false; break; }
        dsxm = (pos.x - PIXY_SET_POINT_X[layer - 1]) * SETPOINT_X_PX_TO_MM;
      }
      if( seen && fabs(dsxm) <= PIXY_TOLERANCE_MM ) {
        PickBlockAtLayer( layer );
        result = 1;
      } else {
        out.println( F("align fail, back") );
        if( fabs(movedMm) > 0.5f ) DriveDistanceSmoothWithInternalAcceleration( dxl, fabs(movedMm), movedMm > 0 ? MOVE_LEFTWARD : MOVE_RIGHTWARD, SEARCH_MM_PER_SEC );
        result = -1;
      }
    }
  }
  targetBlockSigmaps[0] = saved;
  searchIndex = savedIdx;
  return result;
}

void FindAndPick( int color )
{
  if( curCol < 1 ) { out.println( F("column unknown: t5 3 first") ); return; }
  uint8_t saved = targetBlockSigmaps[0];
  int savedIdx = searchIndex;
  targetBlockSigmaps[0] = 1 << (color - 1);
  searchIndex = 0;
  int path[8], n = 0;
  for( int c = curCol; c <= 4; c++ ) path[n++] = c;
  for( int c = 3; c >= 1; c-- ) path[n++] = c;
  int layer = 0;
  for( int i = 0; i < n && ! layer; i++ ) {
    if( path[i] != curCol ) GoColumn( path[i] );
    else AlignAtColumn( curCol );
    layer = CamFindBlockAndPickup( 0, curCol );
  }
  targetBlockSigmaps[0] = saved;
  searchIndex = savedIdx;
  if( layer ) { out.print( F("pick ok: column ") ); out.print( curCol ); out.print( F(" layer ") ); out.println( layer ); }
  else out.println( F("not found") );
}

void DeliverTest( int dest )
{
  if( curCol < 1 ) { out.println( F("column unknown: t5 3 first") ); return; }
  int savedG = goalPositions[0];
  int savedN = nMissionBlocksFound, savedIdx = searchIndex;
  goalPositions[0] = dest;
  nMissionBlocksFound = 2;
  searchIndex = 0;
  gameOver = false;
  DropAndReturn( curCol );
  goalPositions[0] = savedG;
  nMissionBlocksFound = savedN;
  searchIndex = savedIdx;
  curCol = gameOver ? 0 : 1;
}

void DropHere( int dest )
{
  int savedG = goalPositions[0];
  int savedN = nMissionBlocksFound;
  goalPositions[0] = dest;
  nMissionBlocksFound = 1;
  if( ! DropPickedBlockIntoDropbox( 0 ) ) out.println( F("drop fail") );
  goalPositions[0] = savedG;
  nMissionBlocksFound = savedN;
}

void SelfCheck()
{
  uint8_t ids[16];
  uint8_t n = dxl.ping( DXL_BROADCAST_ID, ids, sizeof(ids), 100 );
  out.print( F("motors: ") );
  for( uint8_t i = 0; i < n; i++ ) { out.print( ids[i] ); out.print( ' ' ); }
  out.println();
  for( uint8_t id = 1; id <= 4; id++ ) {
    out.print( F("V") ); out.print( id ); out.print( ' ' );
    out.print( dxl.readControlTableItem( PRESENT_INPUT_VOLTAGE, id ) / 10.0, 1 );
    out.print( id < 4 ? F("  ") : F("\n") );
  }
  out.println( pixy.getVersion() >= 0 ? F("pixy ok") : F("pixy NO RESPONSE") );
  PrintPsd();
  out.println( AreMissionAssignmentsValid() ? F("missionAssignments ok") : F("missionAssignments INVALID") );
  out.println( AreRequiredDropboxPosesPresent() ? F("poses for mission ok") : F("poses MISSING") );
}

void PrintRates()
{
  const char* nm[4] = { "wf", "wb", "wl", "wr" };
  for( uint8_t d = 0; d < 4; d++ ) {
    out.print( nm[d] );
    for( uint8_t i = 0; i < 4; i++ ) { out.print( ' ' ); out.print( wheelRate[d][i], 3 ); }
    out.println();
  }
}

void CmdBaud()
{
  dxl.begin( 57600 );
  dxl.setPortProtocolVersion( DXL_PROTOCOL_VERSION );
  uint8_t ids[16];
  uint8_t n = dxl.ping( DXL_BROADCAST_ID, ids, sizeof(ids), 100 );
  out.print( F("57600 found ") );
  out.println( n );
  for( uint8_t i = 0; i < n; i++ ) {
    dxl.torqueOff( ids[i] );
    dxl.writeControlTableItem( BAUD_RATE, ids[i], BAUD_RATE_1M );
  }
  delay( 100 );
  InitMotorCommunication( dxl );
  out.println( F("reset board") );
}

void Help()
{
  out.println( F("lamp1: Pixy lamp on | lamp0: Pixy lamp off") );
  out.println( F("pose | free | lock | j1 +20 | j1 2047 | arm a b c d | grip 740") );
  out.println( F("p travel | p hub | save scan1 (RAM, prints Tuning.h line) | list") );
  out.println( F("move F 300 (F/B/L/R) | turn L/R [deg] | fr 315 | sl 558 | sr 275 | rot 311 8 | psd | check") );
  out.println( F("t1 sensor | t2 move | t3 2 4 (layer color) | t3 A 5 / t3 B 5 (drop)") );
  out.println( F("t4 5 (find color 5 from current column and pick)") );
  out.println( F("t5 1 | t5 2 | t5 3 (to column 1) | t5 3 2 (to column 2) | t5 4 3 (A3) | t5 5 3 (B3) | t5 6 finish") );
  out.println( F("t6 storage from column 1 | t7 full (SW1) | w | wf a b c d (wb wl wr) | baud") );
  out.println( F("pose names: travel carry order hub scan1-3 pa1-3 pg1-3 za1-8 zb1-8 (1=bottom)") );
}

void Handle( char* line )
{
  char* cmd = strtok( line, " \t" );
  if( ! cmd ) return;
  char* arg[4] = { nullptr, nullptr, nullptr, nullptr };
  for( uint8_t i = 0; i < 4; i++ ) arg[i] = strtok( nullptr, " \t" );
  char* a1 = arg[0];
  char* a2 = arg[1];
  int i1 = a1 ? atoi( a1 ) : 0, i2 = a2 ? atoi( a2 ) : 0;

  if( ! strcmp( cmd, "lamp1" ) ) { pixy.setLamp( 1, 1 ); out.println( F("Pixy lamp on") ); }
  else if( ! strcmp( cmd, "lamp0" ) ) { pixy.setLamp( 0, 0 ); out.println( F("Pixy lamp off") ); }
  else if( ! strcmp( cmd, "help" ) ) Help();
  else if( ! strcmp( cmd, "pose" ) ) PrintArm();
  else if( ! strcmp( cmd, "free" ) ) SetArmTorque( false );
  else if( ! strcmp( cmd, "lock" ) ) { SetArmTorque( true ); PrintArm(); }
  else if( cmd[0] == 'j' && cmd[1] >= '1' && cmd[1] <= '4' && ! cmd[2] && a1 ) {
    int32_t v[4];
    if( ReadOurs( v ) ) {
      uint8_t k = cmd[1] - '1';
      v[k] = (a1[0] == '+' || a1[0] == '-') ? v[k] + i1 : i1;
      MoveOurs( v, 500 );
      PrintArm();
    }
  }
  else if( ! strcmp( cmd, "arm" ) && arg[3] ) {
    int32_t v[4] = { atoi( arg[0] ), atoi( arg[1] ), atoi( arg[2] ), atoi( arg[3] ) };
    MoveOurs( v, 1000 );
    PrintArm();
  }
  else if( ! strcmp( cmd, "grip" ) && a1 ) { pixy.setServos( 0, constrain( i1, 0, 1000 ) ); out.print( F("grip = ") ); out.println( constrain( i1, 0, 1000 ) ); }
  else if( ! strcmp( cmd, "p" ) && a1 ) {
    if( ! strcmp( a1, "hub" ) ) { int32_t v[4] = { POSE_HUB.j1, POSE_HUB.j2, POSE_HUB.j3, POSE_HUB.j4 }; MoveOurs( v, 1000 ); PrintArm(); }
    else { int id = PoseIdByName( a1 ); if( ! id ) out.println( F("unknown pose") ); else if( PlayPose( id, 1000 ) ) PrintArm(); }
  }
  else if( ! strcmp( cmd, "save" ) && a1 ) {
    int id = PoseIdByName( a1 );
    int32_t v[4];
    if( ! id ) out.println( F("unknown pose") );
    else if( ReadOurs( v ) ) {
      Pose np = { (int16_t)v[0], (int16_t)v[1], (int16_t)v[2], (int16_t)v[3] };
      *PoseSlot( id ) = np;
      for( uint8_t k = 0; k < 3; k++ ) if( id == pickApproachPoseIds[k] ) *PoseSlot( pickOutPoseIds[k] ) = np;
      out.print( F("saved (until power off), copy to Tuning.h: ") );
      PrintPoseLine( a1, np );
    }
  }
  else if( ! strcmp( cmd, "list" ) ) ListPoses();
  else if( ! strcmp( cmd, "move" ) && a2 ) DriveDistanceSmoothWithInternalAcceleration( dxl, atof( a2 ), DirByChar( a1[0] ), MOVE_MM_PER_SEC );
  else if( ! strcmp( cmd, "turn" ) && a1 ) {
    float adj = a2 ? atof( a2 ) : 0.0f;
    if( a1[0] == 'L' || a1[0] == 'l' ) RotateLeft90Degree( dxl, MOVE_MM_PER_SEC, adj ); else RotateRight90Degree( dxl, MOVE_MM_PER_SEC, adj );
  }
  else if( ! strcmp( cmd, "fr" ) && a1 ) { PerformFrontMoveAlign( i1, a2 ? i2 : PSD_STORAGE_MOV_TOLERANCE ); PrintPsd(); }
  else if( ! strcmp( cmd, "sl" ) && a1 ) { PerformLeftMoveAlign( i1, a2 ? i2 : PSD_MISSION_MOV_TOLERANCE ); PrintPsd(); }
  else if( ! strcmp( cmd, "sr" ) && a1 ) { PerformRightMoveAlign( i1, a2 ? i2 : PSD_STORAGE_MOV_TOLERANCE ); PrintPsd(); }
  else if( ! strcmp( cmd, "rot" ) && a2 ) { PerformMoveRotateAlign( i1, PSD_STORAGE_MOV_TOLERANCE, PSD_STORAGE_ROT_TOLERANCE, i2 ); PrintPsd(); }
  else if( ! strcmp( cmd, "psd" ) ) PrintPsd();
  else if( ! strcmp( cmd, "check" ) ) SelfCheck();
  else if( ! strcmp( cmd, "t1" ) ) SensorMonitor();
  else if( ! strcmp( cmd, "t2" ) ) MoveTest();
  else if( ! strcmp( cmd, "t3" ) && a2 ) {
    char z = toupper( a1[0] );
    if( z == 'A' && i2 >= 1 && i2 <= 8 ) DropHere( i2 );
    else if( z == 'B' && i2 >= 1 && i2 <= 8 ) DropHere( 8 + i2 );
    else if( i1 >= 1 && i1 <= 3 && i2 >= 1 && i2 <= 6 ) {
      int r = PickLayerColor( i1, i2 );
      out.println( r > 0 ? F("pick ok") : r == 0 ? F("not seen") : F("pick fail") );
    }
    else out.println( F("? t3 1~3 1~6 | t3 A/B 1~8") );
  }
  else if( ! strcmp( cmd, "t4" ) && a1 && i1 >= 1 && i1 <= 6 ) FindAndPick( i1 );
  else if( ! strcmp( cmd, "t5" ) && a1 ) {
    gameOver = false;
    if( i1 == 1 ) {
      PlayPose( START_POSE, 1000 );
      DriveDistanceSmoothWithInternalAcceleration( dxl, mm_F_BLINKING_to_OBSTACLE, MOVE_FORWARD, MOVE_MM_PER_SEC );
      PerformMoveRotateAlign( PSD_OBSTACLE_AVG, 3, 3, PSD_OBSTACLE_ROT_ADJUST );
      PrintPsd();
    } else if( i1 == 2 ) {
      t52LampActive = true;
      pixy.setLamp( 1, 1 );
      // t5 2: set the two camera poses in the existing RAM pose table.
      Pose* orderLeft = PoseSlot( ORDER_L_POSE );
      Pose* orderRight = PoseSlot( ORDER_R_POSE );
      if( !orderLeft || !orderRight ) { out.println( F("Order pose unavailable") ); return; }
      *orderLeft = POSE_ORDER_L;
      *orderRight = POSE_ORDER_R;
      MoveOurs( ORDER_TRAVEL_ARM, 600 );
      if( !t52LampActive ) pixy.setLamp( CAM_LAMP_ON, CAM_LAMP_ON );
      lastCheckedLayerPoseId = 0;
      DriveDistanceSmoothWithInternalAcceleration( dxl, mm_L_OBSTACLE_to_MISSION, MOVE_LEFTWARD, SIDE_MM_PER_SEC );
      DriveDistanceSmoothWithInternalAcceleration( dxl, mm_B_OBSTACLE_to_MISSION, MOVE_BACKWARD, MOVE_MM_PER_SEC );
      PerformLeftMoveAlign( PSD_MISSION_SL, PSD_MISSION_MOV_TOLERANCE );
      RunManipulatorPoseWithPoseDataInEEPROM( dxl, ORDER_L_POSE, ORDER_POSE_MS, -360.0 );
      delay( ORDER_POSE_MS + ORDER_POSE_SETTLE_MS );
      if( !t52LampActive ) pixy.setLamp( CAM_LAMP_ON, CAM_LAMP_ON );
      delay( 150 );
      uint8_t sigs[MAX_MISSION_BLOCKS] = { 0 };
      int count = ReadMissionSlots( sigs );
      ShowMissionCountColor( count );
      if( !t52LampActive ) pixy.setLamp( 0, 0 );
      ApplyMissionBlockSequence( sigs, count );
    } else if( i1 == 3 && a2 ) GoColumn( i2 );
    else if( i1 == 3 ) ArriveCell1();
    else if( i1 == 4 && i2 >= 1 && i2 <= 8 ) DeliverTest( i2 );
    else if( i1 == 5 && i2 >= 1 && i2 <= 8 ) DeliverTest( 8 + i2 );
    else if( i1 == 6 ) { FinishFromStorageColumn( curCol < 1 ? 1 : curCol ); curCol = 0; }
    else out.println( F("? t5 1~6") );
  }
  else if( ! strcmp( cmd, "t6" ) ) {
    gameOver = false;
    searchIndex = 0;
    searchSweepCount = 0;
    SetMissionBlockInfo();
    if( curCol != 1 ) GoColumn( 1 );
    SearchLoop();
    curCol = 0;
  }
  else if( ! strcmp( cmd, "t7" ) ) { RunMission(); curCol = 0; }
  else if( ! strcmp( cmd, "col" ) && a1 && i1 >= 1 && i1 <= 4 ) { curCol = i1; out.print( F("column ") ); out.println( curCol ); }
  else if( ! strcmp( cmd, "w" ) ) PrintRates();
  else if( cmd[0] == 'w' && cmd[1] && ! cmd[2] && strchr( "fblr", cmd[1] ) ) {
    uint8_t d = strchr( "fblr", cmd[1] ) - "fblr";
    if( arg[3] ) for( uint8_t i = 0; i < 4; i++ ) wheelRate[d][i] = atof( arg[i] );
    PrintRates();
  }
  else if( ! strcmp( cmd, "baud" ) ) CmdBaud();
  else out.println( F("? (help)") );
}

char lineBuf[48];
uint8_t lineLen = 0;

void Feed( Stream& s )
{
  while( s.available() ) {
    char ch = s.read();
    if( ch == '\n' || ch == '\r' ) {
      if( lineLen ) { lineBuf[lineLen] = 0; lineLen = 0; Handle( lineBuf ); }
    } else if( lineLen < sizeof(lineBuf) - 1 ) {
      lineBuf[lineLen++] = ch;
    }
  }
}

void setup()
{
  const unsigned long powerStart = millis();
  InitDebug();
  Serial2.begin( 115200 );
  pinMode( SW1_PIN, INPUT );
  pinMode( SW2_PIN, INPUT );

  InitMotorCommunication( dxl );
  const bool startupArmReady = InitManipulator( dxl );
  if( ! startupArmReady ) out.println( F("arm ID 5~8 not found (check power / baud)") );
  if( ! InitMobilebase( dxl ) ) out.println( F("wheel ID 1~4 not found (check power / baud)") );

  InitPSD();
  InitPixy( pixy );
  if( !t52LampActive ) pixy.setLamp( 0, 0 );

  pixels.begin();
  pixels.setBrightness( 50 );
  pixels.show();

  SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );
  OpenGripper( pixy );
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode( dxl );

  ShowColor( 0, 0, 0 );
  out.println( F("< T23 > help | SW1 = t7 | SW2 = stop") );
  SelfCheck();
  // 초기화가 3초보다 짧으면 전원 후 3초 시점까지 대기한다.
  while( millis() - powerStart < 3000UL ) delay( 1 );
  tone( BUZZER_PIN, 392 );
  delay( 1000 );
  noTone( BUZZER_PIN );
  delay( 500 );
  if( startupArmReady ) PlayPose( START_POSE, 1000 ); // p travel = ID 34
}

void loop()
{
  if( setjmp( stopJmp ) ) {
    InitMobilebase( dxl );
    SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );
    gameOver = true;
    t52LampActive = false;
    pixy.setLamp( 0, 0 );
    curCol = 0;
    out.println( F("STOP (SW2)") );
    tone( BUZZER_PIN, 300, 300 );
    while( digitalRead( SW2_PIN ) == LOW ) {}
    return;
  }
  stopArmed = true;
  Feed( Serial );
  Feed( Serial2 );
  if( digitalRead( SW1_PIN ) == LOW ) {
    delay( 20 );
    while( digitalRead( SW1_PIN ) == LOW ) delay( 10 );
    RunMission();
    curCol = 0;
  }
  stopArmed = false;
}
