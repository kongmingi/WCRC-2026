#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include "T45_Motor.h"
#include "T45_Tuning.h"

//////////////  모터 구동용 변수/상수
#define ARM_DXL_ID_1            0x05 // 매니퓰레이터 1번 모터 아이디 (가장 아래)
#define ARM_DXL_ID_2            0x06 // 매니퓰레이터 2번 모터 아이디
#define ARM_DXL_ID_3            0x07 // 매니퓰레이터 3번 모터 아이디
#define ARM_DXL_ID_4            0x08 // 매니퓰레이터 4번 모터 아이디 (가장 위)




// FinManipulatorServos 함수에서 찾을 모터 아이디들, 값이 중복 없이 정렬되어있어야 함
const uint8_t ARM_DXL_ID_CNT = 4;
const uint8_t ARM_DXL_IDS[ARM_DXL_ID_CNT] = { ARM_DXL_ID_1, ARM_DXL_ID_2, ARM_DXL_ID_3, ARM_DXL_ID_4 };

// arm sync write 용 상수, 구조체 정의
const uint16_t SW_JOINT_POSITION_START_ADDR = 112; // sync write start 주소
const uint16_t SW_JOINT_POSITION_DATA_SIZE = 8; // sync write data 길이
typedef struct sw_joint_position_data{ // 모터에 데이터를 쓰기 위한 구조체 정의
  int32_t profile_velocity;
  int32_t goal_position;
} __attribute__((packed)) sw_joint_position_data_t;


#define MANIPULATOR_POSE_ID_MAX_CNT           (10 + 8 + 8 + 4 + 6)  // 36자세: ID 1~30 + 층별 탐색 31~33 + Reserved 정렬용 34~36 (EEPROM 0..1439)

#define MANIPULATOR_POSE_DATA_SIZE            40
#define MANIPULATOR_POSE_DESCRIPTION_SIZE     30

typedef struct _ManipulatorPose { // EEPROM에 저장되는 매니퓰레이터 포즈객체
  uint8_t isTherePoseData;  // EEPROM 원시 바이트: 1만 유효
  uint8_t id;
  int16_t manipulatorMotor1Value;
  int16_t manipulatorMotor2Value;
  int16_t manipulatorMotor3Value;
  int16_t manipulatorMotor4Value;
  char description[MANIPULATOR_POSE_DESCRIPTION_SIZE];
} __attribute__((packed)) ManipulatorPose;

static_assert(sizeof(ManipulatorPose) == MANIPULATOR_POSE_DATA_SIZE,
              "EEPROM pose layout mismatch");


/* 
 * 매니퓰레이터를 네 모터의 값과 시간으로 제어하는 함수
 * argument : 
 *    mv1 : 5번 모터 값
 *    mv2 : 6번 모터 값
 *    mv3 : 7번 모터 값
 *    mv4 : 8번 모터 값
 *    operatingTime : 매니퓰레이터 동작이 완료되기까지 소요될 시간
 */
void setManipulatorForwardMoveWithMotorValueForSyncWrite( Dynamixel2Arduino& dxl,
        int32_t mv1, int32_t mv2, int32_t mv3, int32_t mv4, int32_t operatingTime );


/* 
 * 매니퓰레이터 모터가 통신 가능한지 확인하고 초기화 하는 함수
 * return :
 *    true  : 통신 체크 및 초기화 성공
 *    false : 통신 체크 및 초기화 실패
 */                        
bool InitManipulator(Dynamixel2Arduino& dxl);

/*
 * ARM_DXL_IDS 배열에 있는 아이디의 모터들이 모두 통신 가능한지
 * 확인하는 함수
 * return :
 *    true  : 통신 체크 성공
 *    false : 통신 체크 실패
 */
bool FindManipulatorServos(Dynamixel2Arduino& dxl);

/*
 * 매니퓰레이터 포즈 데이터를 EEPROM에서 읽어 반환
 * return :
 *    ManipulatorPose 객체 : 해당 아이디를 가진 ManipulatorPose 객체를 반환
 *                          isTherePoseData로 데이터가 유효한지 확인한 후 사용해야 함
 */
ManipulatorPose ReadManipulatorPresentPoseToEEPROM( uint8_t id );

/*
 * 매니퓰레이터 포즈 데이터를 EEPROM에서 읽어 실행
 * params :
 *    id : 실행할 포즈의 아이디
 *    operatingTimeMillis : 포즈 실행에 사용할 시간
 *    motor1Angle(-100~100) : 1번 모터 회전 각도(기본 값은 -360.0이며, 각도 값을 주지 않으면 포즈 데이터의 값을 그대로 사용)
 *                            과한 회전을 방지하기 위해 범위를 -100~100도로 하였음
 * return :
 *    ManipulatorPose 객체 : 실행한 ManipulatorPose 객체를 반환
 *                          isTherePoseData로 데이터가 유효한지 확인한 후 사용해야 함
 */
ManipulatorPose RunManipulatorPoseWithPoseDataInEEPROM( Dynamixel2Arduino& dxl,
                  uint8_t id, int32_t operatingTimeMillis, float motor1Angle = -360.0 );

Pose* PoseSlot( uint8_t id );
int32_t ToWindow( int32_t raw, uint8_t i );

#endif
