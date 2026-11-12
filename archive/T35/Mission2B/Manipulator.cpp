#include "Arduino.h"
#include "Debug.h"
#include "Manipulator.h"

#include <EEPROM.h>


// arm sync write 용 객체 인스턴스화
sw_joint_position_data_t sw_joint_position_data[ARM_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성
DYNAMIXEL::InfoSyncWriteInst_t sw_arm_position_infos; // syncwrite 정보 인스턴스 생성
// syncWrite할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncWrite_t info_joint_position_xels_sw[ARM_DXL_ID_CNT];


int32_t ToWindow( int32_t raw, uint8_t i )
{
  int32_t lo = ((int32_t)JOINT_MIN[i] + JOINT_MAX[i]) / 2 - 2048;
  int32_t v = (raw - lo) % 4096;
  if( v < 0 ) v += 4096;
  return lo + v;
}

void setManipulatorForwardMoveWithMotorValueForSyncWrite( Dynamixel2Arduino& dxl,
                                      int32_t mv1, int32_t mv2, int32_t mv3, int32_t mv4, int32_t operatingTime)
{
  const int32_t target[4] = { mv1, mv2, mv3, mv4 };
  for( uint8_t i = 0; i < ARM_DXL_ID_CNT; i++ ) {
    int32_t raw = 0;
    bool ok = false;
    for( uint8_t k = 0; k < 3 && ! ok; k++ ) {
      raw = dxl.readControlTableItem( PRESENT_POSITION, ARM_DXL_IDS[i] );
      ok = dxl.getLastLibErrCode() == DXL_LIB_OK;
    }
    if( ! ok ) return;
    int32_t goal = constrain( target[i], (int32_t)JOINT_MIN[i], (int32_t)JOINT_MAX[i] );
    sw_joint_position_data[i].goal_position = raw + (goal - ToWindow( raw, i ));
    sw_joint_position_data[i].profile_velocity = operatingTime;
  }

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
  
  for( int i = 0; i < ARM_DXL_ID_CNT; i++ ) {
    dxl.writeControlTableItem( TORQUE_ENABLE, ARM_DXL_IDS[i], TORQUE_OFF );
    dxl.writeControlTableItem( DRIVE_MODE, ARM_DXL_IDS[i], REVERSE_MODE + TIME_BASED_PROFILE );
    dxl.writeControlTableItem( OPERATING_MODE, ARM_DXL_IDS[i], EXTENDED_POSITION_CONTROL_MODE );
    dxl.writeControlTableItem( HOMING_OFFSET, ARM_DXL_IDS[i], 0 );
  }
  for( int i = 0; i < ARM_DXL_ID_CNT; i++ ) dxl.reboot( ARM_DXL_IDS[i] );
  delay( 800 );
  for( int i = 0; i < ARM_DXL_ID_CNT; i++ ) {
    dxl.writeControlTableItem( POSITION_P_GAIN, ARM_DXL_IDS[i], ARM_P_GAIN );
    dxl.writeControlTableItem( POSITION_I_GAIN, ARM_DXL_IDS[i], ARM_I_GAIN );
    int32_t raw = dxl.readControlTableItem( PRESENT_POSITION, ARM_DXL_IDS[i] );
    dxl.writeControlTableItem( GOAL_POSITION, ARM_DXL_IDS[i], raw );
    dxl.writeControlTableItem( TORQUE_ENABLE, ARM_DXL_IDS[i], TORQUE_ON );
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
  poseTable[4] = POSE_CARRY;
  poseTable[3] = POSE_DRIVE;
  poseTable[34] = POSE_TRAVEL;
  poseTable[1] = POSE_ORDER_MOVE;
  poseTable[2] = POSE_CARRY_LEFT;
  poseTable[35] = POSE_ORDER_L;
  poseTable[36] = POSE_ORDER_R;
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

ManipulatorPose ReadManipulatorPresentPoseToEEPROM(uint8_t id)
{
  ManipulatorPose manipulatorPose = {};
  Pose* p = PoseSlot( id );
  if( p && ( p->j1 || p->j2 || p->j3 || p->j4 ) ) {
    manipulatorPose.isTherePoseData = 1;
    manipulatorPose.id = id;
    manipulatorPose.manipulatorMotor1Value = p->j1;
    manipulatorPose.manipulatorMotor2Value = p->j2;
    manipulatorPose.manipulatorMotor3Value = p->j3;
    manipulatorPose.manipulatorMotor4Value = p->j4;
  }
  return manipulatorPose;
}


ManipulatorPose RunManipulatorPoseWithPoseDataInEEPROM( Dynamixel2Arduino& dxl,
                                                        uint8_t id, int32_t operatingTimeMillis, float motor1Angle )
{
  ManipulatorPose manipulatorPose = ReadManipulatorPresentPoseToEEPROM( id );

  if( manipulatorPose.isTherePoseData == 1 )
  {
    int32_t manipulatorMotor1Value = manipulatorPose.manipulatorMotor1Value;

    setManipulatorForwardMoveWithMotorValueForSyncWrite( dxl, manipulatorMotor1Value,
                                                         manipulatorPose.manipulatorMotor2Value,
                                                         manipulatorPose.manipulatorMotor3Value,
                                                         manipulatorPose.manipulatorMotor4Value,
                                                         operatingTimeMillis );
  }

  return manipulatorPose;
}
