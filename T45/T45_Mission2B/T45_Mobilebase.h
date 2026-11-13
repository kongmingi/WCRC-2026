#ifndef MOBILEBASE_H
#define MOBILEBASE_H

#include "T45_Tuning.h"

#include "T45_Motor.h"

//////////////  모터 구동용 변수/상수
#define MOBILE_DXL_ID_FL            0x01  // 좌측 전방 모터 아이디
#define MOBILE_DXL_ID_FR            0x02  // 우측 전방 모터 아이디
#define MOBILE_DXL_ID_BL            0x03  // 좌측 후방 모터 아이디
#define MOBILE_DXL_ID_BR            0x04  // 우측 후방 모터 아이디

#define S_TO_MILLIS_RATIO           1000  // 일단 기본 속력
#define S_TO_MILLIS_RATIO_FAST      ( S_TO_MILLIS_RATIO / 2.00 )  //  500.00
#define S_TO_MILLIS_RATIO_MORE      ( S_TO_MILLIS_RATIO / 1.50 )  //  666.66
#define S_TO_MILLIS_RATIO_SLOW      ( S_TO_MILLIS_RATIO / 0.75 )  // 1333.33

const int32_t MOBILEBASE_SPEED_MORE = ( MOBILEBASE_SPEED * 1.50 );  // 150
const int32_t MOBILEBASE_SPEED_SLOW = ( MOBILEBASE_SPEED * 0.75 );  //  75

const int32_t MOBILEBASE_MM_PER_SEC = 72;  // 이동 속도: 72 mm/sec
const int32_t MOBILEBASE_MM_PER_SEC_FAST = MOBILEBASE_SPEED_FAST;  // 공통 거리 이동 속도와 같은 값
const int32_t MOBILEBASE_MM_PER_SEC_MORE = ( MOBILEBASE_MM_PER_SEC * 1.50 );  // 108.0 mm/sec
const int32_t MOBILEBASE_MM_PER_SEC_SLOW = ( MOBILEBASE_MM_PER_SEC * 0.75 );  //  54.0 mm/sec


// motorAdjust와 같은 시간 기반 프로파일: 가속 100 ms, 전체 시간 = 거리 / 속도 * 1000.
const uint32_t MOBILEBASE_PROFILE_ACCELERATION_MS = 100;

#define MOBILEBASE_DEFAULT_OPERATING_MODE   VELOCITY_CONTROL_MODE
#define MOBILEBASE_DEFAULT_DRIVE_MODE       VELOCITY_BASED_PROFILE

#define RADIANS_2_MOTOR_VALUE_RATIO     ( 4095 / (2*PI) )
#define RADIANS_2_MOTOR_VALUE(X)        (int32_t)( (X) * RADIANS_2_MOTOR_VALUE_RATIO )

//////////////  모바일베이스 주행 방향
#define MOVE_STOP         0x00 // 정지
#define MOVE_FORWARD      0x01 // 전진
#define MOVE_BACKWARD     0x02 // 후진
#define MOVE_LEFTWARD     0x03 // 좌진
#define MOVE_RIGHTWARD    0x04 // 우진

#define ROTATE_CCW        0x05 // 반시계 방향 회전
#define ROTATE_CW         0x06 // 시계 방향 회전

//////////////
#define WHEEL_RADIUS_MM   (60 / 2.0)

// Mission과 같은 바퀴 중심 간격. 회전 추가 보정값은 각 구간 호출에서 전달한다.
const float MOBILEBASE_WHEEL_BASE_MM = 135.0f;
const float MOBILEBASE_TRACK_WIDTH_MM = 180.0f;



// 진행 테스트용 초기값. 정상 이동 완료 시간/통신 상태를 실측한 뒤 여기서 조정.
constexpr uint8_t MOBILE_READ_RETRY_LIMIT = 20;           // 도착 상태 읽기: 바깥 시간 검사로 빨리 복귀
constexpr uint8_t MOBILE_POSITION_READ_RETRY_LIMIT = 50; // 이동 전 현재 위치 읽기만 재시도 확대
constexpr uint32_t MOBILE_MOVE_TIMEOUT_MARGIN_MS = 1000UL;
static_assert(MOBILE_READ_RETRY_LIMIT > 0 && MOBILE_POSITION_READ_RETRY_LIMIT > 0,
              "At least one mobile read is required");


const int alignInnerLoopDelay = 10;  // 내부 함수에서 반복하는 각각의 보정 시작 후 대기 시간 (ms) 20
const int movelessSpeed = 10;        // 유의미한 움직임 임계값(mm) MIN_MOVE_THRESHOLD

#define ALIGN_STABLE_COUNT       3   // 보정 성공 판정에 필요한 연속 조건만족 횟수


// FindMobileBaseServos 함수에서 찾을 모터 아이디들, 값이 중복 없이 정렬되어있어야 함
const uint8_t MOBILE_DXL_ID_CNT = 4;
const uint8_t MOBILE_DXL_IDS[MOBILE_DXL_ID_CNT] = { MOBILE_DXL_ID_FL,  MOBILE_DXL_ID_FR,
                                                    MOBILE_DXL_ID_BL,  MOBILE_DXL_ID_BR };


// mobile velocity control mode sync write 용 상수, 구조체 정의 및 인스턴스화
const uint16_t SW_WHEEL_VELOCITY_START_ADDR = 104; // sync write start 주소
const uint16_t SW_WHEEL_VELOCITY_DATA_SIZE = 4; // sync write data 길이
typedef struct sw_wheel_velocity_data{ // 모터에 데이터를 쓰기 위한 구조체 정의
  int32_t goal_velocity;
} __attribute__((packed)) sw_wheel_velocity_data_t;


// mobile extended position control mode sync write 용 상수, 구조체 정의 및 인스턴스화
const uint16_t SW_WHEEL_POSITION_START_ADDR = 112; // sync write start 주소
const uint16_t SW_WHEEL_POSITION_DATA_SIZE = 8; // sync write data 길이
typedef struct sw_wheel_position_data { // 모터에 데이터를 쓰기 위한 구조체 정의
  int32_t profile_velocity;
  int32_t goal_position;
} __attribute__((packed)) sw_wheel_position_data_t;


// mobile extended position control mode sync read 용 상수, 구조체 정의 및 인스턴스화
const uint16_t wheel_position_pkt_buf_cap = 128;

const uint16_t SR_WHEEL_POSITION_START_ADDR = 132; // sync read start 주소
const uint16_t SR_WHEEL_POSITION_DATA_SIZE = 4; // sync read data 길이
typedef struct sr_wheel_position_data{ // 모터에서 데이터를 읽기 위한 구조체 정의
  int32_t present_position;
} __attribute__((packed)) sr_wheel_position_data_t;


// is moving sync read 용 상수, 구조체 정의 및 인스턴스화
const uint16_t moving_pkt_buf_cap = 128;

const uint16_t SR_WHEEL_MOVING_START_ADDR = 122; // sync write start 주소
const uint16_t SR_WHEEL_MOVING_DATA_SIZE = 2; // sync write data 길이
typedef struct sr_wheel_moving_data{ // 모터에 데이터를 쓰기 위한 구조체 정의
  uint8_t moving;
  uint8_t moving_status;
} __attribute__((packed)) sr_wheel_moving_data_t;


//////////////  모바일베이스 함수 선언
/* 
 * 모바일베이스 모터가 통신 가능한지 확인하고 초기화 하는 함수
 * return :
 *    true  : 통신 체크 및 초기화 성공
 *    false : 통신 체크 및 초기화 실패
 */
bool InitMobilebase(Dynamixel2Arduino& dxl);

/*
 * 모바일베이스를 속도제어 모드로 변경
 */
void ChangeMobilebaseMode2VelocityControlMode(Dynamixel2Arduino& dxl);

/*
 * 모바일베이스를 확장 위치제어, 시간기반 프로파일 모드로 변경
 */
void ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(Dynamixel2Arduino& dxl);

/*
 * MOBILE_DXL_IDS 배열에 있는 아이디의 모터들이 모두 통신 가능한지
 * return :
 *    true  : 통신 체크 및 초기화 성공
 *    false : 통신 체크 및 초기화 실패
 */
bool FindMobileBaseServos(Dynamixel2Arduino& dxl);

/*
 * 모바일베이스가 goal position에 도착했는지 체크하는 함수
 * (sr_wheel_moving_data[i].moving_status)&0x01가 1일 때 arrived, 0일 때 not arrived
 * return :
 *    true  : 도착
 *    false : 이동중 또는 제한 횟수 안에 도착 상태를 읽지 못함
 */
bool CheckIfMobilebaseIsInPosition(Dynamixel2Arduino& dxl);


void DriveDistanceSmoothWithInternalAcceleration( Dynamixel2Arduino& dxl,
        float distance, uint8_t drivingDirection = MOVE_FORWARD, int32_t mmPerSec = MOBILEBASE_SPEED_FAST );

// 속도와 구간별 추가각을 호출에서 반드시 지정한다. 기존 주행 완료 대기는 그대로 사용한다.
void RotateLeft90Degree( Dynamixel2Arduino& dxl, int32_t mmPerSec, float adjustDegrees );
void RotateRight90Degree( Dynamixel2Arduino& dxl, int32_t mmPerSec, float adjustDegrees );


bool RotateAlignWithFront2PSD( Dynamixel2Arduino& dxl,
                               int16_t moveSensorError, int16_t rotationSensorError,
                               int16_t moveToleranceValue, int16_t rotationToleranceValue,
                               float moveControlRatio, float roatingControlRatio,      
                               bool resetStableCount= false, int32_t drivingSpeed= MOBILEBASE_SPEED );

bool  MoveAlignWithLeftSidePSD( Dynamixel2Arduino& dxl, int psdError, int tolerance,
                                     float controlRatio= PSD_MOVE_CONTROL_RATIO,
                                     bool resetStableCount= false, int32_t moveSpeed= MOBILEBASE_SPEED );

bool  MinusMoveAlignWithLeftSidePSD( Dynamixel2Arduino& dxl, int psdError, int tolerance,
                                     float controlRatio= PSD_MOVE_CONTROL_RATIO,
                                     bool resetStableCount= false );  // int32_t moveSpeed= MOBILEBASE_SPEED

bool  MoveAlignWithFrontRightPSD( Dynamixel2Arduino& dxl, int psdError, int tolerance,
                                  float controlRatio= PSD_MOVE_CONTROL_RATIO,
                                  bool resetStableCount= false, int32_t moveSpeed= MOBILEBASE_SPEED );


/*
 * 멀티턴모드를 사용하여 정해진 바퀴를 모터 값 만큼 회전시키는 함수
 * params :
 *    fl_relative_position : 좌측전방 바퀴 모터 회전 값 
 *    fr_relative_position : 우측전방 바퀴 모터 회전 값
 *    bl_relative_position : 좌측후방 바퀴 모터 회전 값
 *    br_relative_position : 우측전방 바퀴 모터 회전 값
 *    drivingTime : 이동에 사용할 시간
 */
// 네 위치를 모두 읽고 명령 전송에 성공한 경우만 true. 실패하면 이번 명령을 생략.
bool SetMobileRelativePositionForSyncWrite(Dynamixel2Arduino& dxl,
                                           int32_t fl_relative_position, int32_t fr_relative_position,
                                           int32_t bl_relative_position, int32_t br_relative_position,
                                           int32_t drivingTime);

/*
 * sync write 하기 위한 모바일베이스의 goal velocity 값들을 설정하는 함수
 * params :
 *    fl_goal_velocity : 1번 모터 속도
 *    fr_goal_velocity : 2번 모터 속도
 *    bl_goal_velocity : 3번 모터 속도
 *    br_goal_velocity : 4번 모터 속도
 */
void SetMobileGoalVelocityForSyncWrite(Dynamixel2Arduino& dxl,
                                       int32_t fl_goal_velocity, int32_t fr_goal_velocity,
                                       int32_t bl_goal_velocity, int32_t br_goal_velocity);


// 전진 시 속도 비율 (MOBILEBASE_SPEED_FAST = 160 mm/s일 때)

// 후진 시 속도 비율 (MOBILEBASE_SPEED_FAST = 160 mm/s일 때)

// 좌진 시 속도 비율 (MOBILEBASE_SPEED_FAST = 160 mm/s일 때)

// 우진 시 속도 비율 (MOBILEBASE_SPEED_FAST = 160 mm/s일 때)

extern float wheelRate[4][4];


// 제자리 회전 시 속도 비율 (MOBILEBASE_SPEED_FAST = 160 mm/s일 때)
// 현재 미사용: DriveDistanceSmoothWithInternalAcceleration()의 ROTATE_CW/ROTATE_CCW 분기는 비율을 곱하지 않고
// 네 바퀴에 같은 크기의 이동량을 부호만 바꿔 보낸다. 각도 보정은 회전 호출의 adjustDegrees로 지정한다(90도 함수만 있음).
// 회전 중 차체가 밀리는 현상이 실측되면 그때 이 비율을 해당 분기에 곱한다.
// 해석 기준(로봇 좌표계의 이상적인 메카넘 모델): 좌측 쌍과 우측 쌍의 비율 차이는 전후 밀림, 앞쪽 쌍과 뒤쪽 쌍의 차이는 좌우 밀림으로
// 나타나고, 같은 차이라도 CW와 CCW에서 밀리는 방향의 부호는 반대다. 실제 미끄러짐까지 확정하는 식은 아니므로 실측으로 정한다.
// 네 비율의 평균을 1.000 근처로 두면 평균 회전량(각도)이 유지되지만 필수 규칙은 아니다. 1을 넘는 비율은 같은 이동 시간에
// 그 바퀴의 회전량과 요구 속도를 그만큼 높이므로(예: 1.010은 약 1%) 바퀴 속도 여유를 함께 확인한다.

// CW 제자리 우회전 시 속도 비율: 좌측 전진(+) / 우측 후진(-), Mobilebase.cpp의 ROTATE_CW 분기와 같은 부호
const float flMotRateCW  = 1.000;  // + R CW 회전 방향
const float frMotRateCW  = 1.000;  // - R
const float blMotRateCW  = 1.000;  // + R
const float brMotRateCW  = 1.000;  // - R

// CCW 제자리 좌회전 시 속도 비율: 좌측 후진(-) / 우측 전진(+), Mobilebase.cpp의 ROTATE_CCW 분기와 같은 부호
const float flMotRateCCW = 1.000;  // - L CCW 회전 방향
const float frMotRateCCW = 1.000;  // + L
const float blMotRateCCW = 1.000;  // - L
const float brMotRateCCW = 1.000;  // + L


#endif
