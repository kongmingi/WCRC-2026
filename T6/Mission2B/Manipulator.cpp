#include "Arduino.h"
#include "Debug.h"
#include "Manipulator.h"

#include <EEPROM.h>


// arm sync write 용 객체 인스턴스화
sw_joint_position_data_t sw_joint_position_data[ARM_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성
DYNAMIXEL::InfoSyncWriteInst_t sw_arm_position_infos; // syncwrite 정보 인스턴스 생성
// syncWrite할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncWrite_t info_joint_position_xels_sw[ARM_DXL_ID_CNT];


void setManipulatorForwardMoveWithMotorValueForSyncWrite( Dynamixel2Arduino& dxl,
                                      int32_t mv1, int32_t mv2, int32_t mv3, int32_t mv4, int32_t operatingTime)
{                                                           
  sw_joint_position_data[0].goal_position = constrain(mv1, ARM_DXL_1_POSITION_MIN, ARM_DXL_1_POSITION_MAX);
  sw_joint_position_data[0].profile_velocity = operatingTime;
  sw_joint_position_data[1].goal_position = constrain(mv2, ARM_DXL_2_POSITION_MIN, ARM_DXL_2_POSITION_MAX);
  sw_joint_position_data[1].profile_velocity = operatingTime;
  sw_joint_position_data[2].goal_position = constrain(mv3, ARM_DXL_3_POSITION_MIN, ARM_DXL_3_POSITION_MAX);
  sw_joint_position_data[2].profile_velocity = operatingTime;
  sw_joint_position_data[3].goal_position = constrain(mv4, ARM_DXL_4_POSITION_MIN, ARM_DXL_4_POSITION_MAX);
  sw_joint_position_data[3].profile_velocity = operatingTime;
  
  sw_arm_position_infos.is_info_changed = true;
  

  int count = 0;
  while (!dxl.syncWrite(&sw_arm_position_infos)) {
    count++;
    if (count > 10) break;
  }
}


bool InitManipulator( Dynamixel2Arduino& dxl )
{
  // 매니퓰레이터 모터가 모두 있는지 확인
  if ( ! FindManipulatorServos(dxl) )
    return false;
  
  for (int i = 0 ; i < ARM_DXL_ID_CNT ; i++)
  {
    // 토크 끄기
    dxl.writeControlTableItem(TORQUE_ENABLE, ARM_DXL_IDS[i], TORQUE_OFF);
  
    // 드라이브 모드 설정
    // 방향은 Reverse Mode, Profile Configuration은 Time-based Profile
    dxl.writeControlTableItem(DRIVE_MODE, ARM_DXL_IDS[i],
                              REVERSE_MODE + TIME_BASED_PROFILE);
    // 오퍼레이팅 모드를 위치 제어 모드로 설정
    dxl.writeControlTableItem(OPERATING_MODE, ARM_DXL_IDS[i], POSITION_CONTROL_MODE);

    switch (ARM_DXL_IDS[i]) { // 모터에 따라 position limit, homing offset 설정
      case ARM_DXL_ID_1:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT, ARM_DXL_IDS[i], ARM_DXL_1_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT, ARM_DXL_IDS[i], ARM_DXL_1_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET,      ARM_DXL_IDS[i], ARM_DXL_1_OFFSET);
        break;
      case ARM_DXL_ID_2:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT, ARM_DXL_IDS[i], ARM_DXL_2_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT, ARM_DXL_IDS[i], ARM_DXL_2_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET,      ARM_DXL_IDS[i], ARM_DXL_2_OFFSET);
        break;
      case ARM_DXL_ID_3:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT, ARM_DXL_IDS[i], ARM_DXL_3_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT, ARM_DXL_IDS[i], ARM_DXL_3_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET,      ARM_DXL_IDS[i], ARM_DXL_3_OFFSET);
        break;
      case ARM_DXL_ID_4:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT, ARM_DXL_IDS[i], ARM_DXL_4_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT, ARM_DXL_IDS[i], ARM_DXL_4_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET,      ARM_DXL_IDS[i], ARM_DXL_4_OFFSET);
        break;
    }

    // 토크 켜기
    dxl.writeControlTableItem(TORQUE_ENABLE, ARM_DXL_IDS[i], TORQUE_ON);
  }
  
  // manipulator sync write 준비
  sw_arm_position_infos.packet.p_buf = nullptr; // nullptr을 전달하면 내부 버퍼를 사용
  sw_arm_position_infos.packet.is_completed = false; // false로 초기화
  sw_arm_position_infos.addr = SW_JOINT_POSITION_START_ADDR; // 컨트롤 테이블에 sync write를 시작주소
  sw_arm_position_infos.addr_length = SW_JOINT_POSITION_DATA_SIZE; // sync write하는 데이터 길이
  sw_arm_position_infos.p_xels = info_joint_position_xels_sw; // sync write 할 모터 정보
  sw_arm_position_infos.xel_count = 0; // sync write 할 모터 개수

  sw_joint_position_data[0].profile_velocity = 1000; // 모터에 write 할 데이터 초기화
  sw_joint_position_data[0].goal_position = 2048;
  sw_joint_position_data[1].profile_velocity = 1000;
  sw_joint_position_data[1].goal_position = 2048;
  sw_joint_position_data[2].profile_velocity = 1000;
  sw_joint_position_data[2].goal_position = 2048;
  sw_joint_position_data[3].profile_velocity = 1000;
  sw_joint_position_data[3].goal_position = 2048;

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < ARM_DXL_ID_CNT; i++) {
    info_joint_position_xels_sw[i].id = ARM_DXL_IDS[i];
    info_joint_position_xels_sw[i].p_data = (uint8_t*)&sw_joint_position_data[i];
    sw_arm_position_infos.xel_count++;
  }
  
  sw_arm_position_infos.is_info_changed = true; // sync write 정보가 변경됨을 설정
  


  return true;
}

bool FindManipulatorServos(Dynamixel2Arduino& dxl)
{
  uint8_t ids_pinged[10] = {0,};
  const uint8_t count_pinged = dxl.ping(DXL_BROADCAST_ID, ids_pinged, sizeof(ids_pinged), 100);
  bool allFound = true;
  for (uint8_t expected = 0; expected < ARM_DXL_ID_CNT; ++expected) {
    bool found = false;
    for (uint8_t received = 0; received < count_pinged && received < sizeof(ids_pinged); ++received) {
      if (ids_pinged[received] == ARM_DXL_IDS[expected]) {
        found = true;
        break;
      }
    }
    if (!found) {
#ifdef DEBUG
      DEBUG_SERIAL.print(F("Missing motor ID: "));
      DEBUG_SERIAL.println(ARM_DXL_IDS[expected]);
#endif
      allFound = false;
    }
  }
  return allFound;
}

static Pose poseTable[MANIPULATOR_POSE_ID_MAX_CNT + 1];
static bool poseTableReady = false;

static void InitPoseTable()
{
  memset( poseTable, 0, sizeof(poseTable) );
  poseTable[4] = POSE_TRAVEL;
  poseTable[34] = POSE_TRAVEL;
  poseTable[27] = POSE_ORDER_VIEW;
  const uint8_t scanId[3] = { 33, 32, 31 };
  const uint8_t appId[3] = { 8, 5, 28 };
  const uint8_t gripId[3] = { 9, 6, 29 };
  const uint8_t outId[3] = { 10, 7, 30 };
  for( uint8_t k = 0; k < 3; k++ ) {
    poseTable[scanId[k]] = POSE_SCAN[k];
    poseTable[appId[k]] = POSE_PICK_APPROACH[k];
    poseTable[gripId[k]] = POSE_PICK_GRIP[k];
    poseTable[outId[k]] = POSE_PICK_APPROACH[k];
  }
  for( uint8_t k = 0; k < 8; k++ ) {
    poseTable[11 + k] = POSE_ZONE_A[k];
    poseTable[19 + k] = POSE_ZONE_B[k];
  }
  poseTableReady = true;
}

Pose* PoseSlot( uint8_t id )
{
  if( ! poseTableReady ) InitPoseTable();
  if( id < 1 || id > MANIPULATOR_POSE_ID_MAX_CNT ) return nullptr;
  return &poseTable[id];
}

int32_t OursToMotor( int32_t v, uint8_t i )
{
  return ((v + ARM_OFFSET[i]) % 4096 + 4096) % 4096;
}

int32_t MotorToOurs( int32_t v, uint8_t i )
{
  int32_t lo = ((int32_t)JOINT_MIN[i] + JOINT_MAX[i]) / 2 - 2048;
  int32_t r = ((v - ARM_OFFSET[i] - lo) % 4096 + 4096) % 4096;
  return lo + r;
}

ManipulatorPose ReadManipulatorPresentPoseToEEPROM(uint8_t id)
{
  ManipulatorPose manipulatorPose = {};
  Pose* p = PoseSlot( id );
  if( p && ( p->j1 || p->j2 || p->j3 || p->j4 ) ) {
    manipulatorPose.isTherePoseData = 1;
    manipulatorPose.id = id;
    manipulatorPose.manipulatorMotor1Value = OursToMotor( p->j1, 0 );
    manipulatorPose.manipulatorMotor2Value = OursToMotor( p->j2, 1 );
    manipulatorPose.manipulatorMotor3Value = OursToMotor( p->j3, 2 );
    manipulatorPose.manipulatorMotor4Value = OursToMotor( p->j4, 3 );
  }
  return manipulatorPose;
}


ManipulatorPose RunManipulatorPoseWithPoseDataInEEPROM( Dynamixel2Arduino& dxl,
                                                        uint8_t id, int32_t operatingTimeMillis, float motor1Angle )
{
  ManipulatorPose manipulatorPose = ReadManipulatorPresentPoseToEEPROM( id );

  if( manipulatorPose.isTherePoseData == 1 )
  {
    int32_t manipulatorMotor1Value = (motor1Angle == -360.0
                                      ? manipulatorPose.manipulatorMotor1Value
                                      : map( constrain( round(motor1Angle * 100), -10000, 10000 ),  -18000, 18000, 0, 4095 ) );

    setManipulatorForwardMoveWithMotorValueForSyncWrite( dxl, manipulatorMotor1Value,
                                                         manipulatorPose.manipulatorMotor2Value,
                                                         manipulatorPose.manipulatorMotor3Value,
                                                         manipulatorPose.manipulatorMotor4Value,
                                                         operatingTimeMillis );
  }

  return manipulatorPose;
}
