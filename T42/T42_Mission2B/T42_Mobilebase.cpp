#include "Arduino.h"
#include "T42_Debug.h"
#include "T42_Mobilebase.h"
#include "T42_Devices.h"


// mobile velocity control mode sync write 용 객체 인스턴스화
sw_wheel_velocity_data_t sw_wheel_velocity_data[MOBILE_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성
DYNAMIXEL::InfoSyncWriteInst_t sw_mobile_velocity_infos; // syncwrite 정보 인스턴스 생성
// syncWrite할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncWrite_t info_wheel_velocity_xels_sw[MOBILE_DXL_ID_CNT];


// mobile extended position control mode sync write 용 객체 인스턴스화
sw_wheel_position_data_t sw_wheel_position_data[MOBILE_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성
DYNAMIXEL::InfoSyncWriteInst_t sw_mobile_position_infos; // syncwrite 정보 인스턴스 생성
// syncWrite할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncWrite_t info_wheel_position_xels_sw[MOBILE_DXL_ID_CNT];


// mobile extended position control mode sync read 용 객체 인스턴스화
uint8_t wheel_position_pkt_buf[wheel_position_pkt_buf_cap];
sr_wheel_position_data_t sr_wheel_position_data[MOBILE_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성
DYNAMIXEL::InfoSyncReadInst_t sr_mobile_position_infos; // syncread 정보 인스턴스 생성
// syncRead할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncRead_t info_wheel_position_xels_sr[MOBILE_DXL_ID_CNT];


// is moving sync read 용 객체 인스턴스화
uint8_t moving_pkt_buf[moving_pkt_buf_cap];
sr_wheel_moving_data_t sr_wheel_moving_data[MOBILE_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성
DYNAMIXEL::InfoSyncReadInst_t sr_mobile_moving_infos; // syncread 정보 인스턴스 생성
// syncRead할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncRead_t info_wheel_moving_xels_sr[MOBILE_DXL_ID_CNT];


//////////////  모바일베이스 함수 정의
bool InitMobilebase(Dynamixel2Arduino& dxl) {
  // 모바일베이스 모터가 모두 있는지 확인
  if (!FindMobileBaseServos(dxl)) return false;
  
  // 각 모터에 설정
  for (int i = 0 ; i < MOBILE_DXL_ID_CNT ; i++) {
    // 토크 끄기
    dxl.writeControlTableItem(TORQUE_ENABLE, MOBILE_DXL_IDS[i], TORQUE_OFF);
  
    // 모드 설정
    if (MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_FL ||
        MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_BL) { // 좌측 바퀴
      dxl.writeControlTableItem(DRIVE_MODE, MOBILE_DXL_IDS[i], NORMAL_MODE + MOBILEBASE_DEFAULT_DRIVE_MODE);
    } else if (MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_FR ||
               MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_BR) { // 우측 바퀴
      dxl.writeControlTableItem(DRIVE_MODE, MOBILE_DXL_IDS[i], REVERSE_MODE + MOBILEBASE_DEFAULT_DRIVE_MODE);
    }
    dxl.writeControlTableItem(OPERATING_MODE, MOBILE_DXL_IDS[i], MOBILEBASE_DEFAULT_OPERATING_MODE);

    // 토크 켜기
    dxl.writeControlTableItem(TORQUE_ENABLE, MOBILE_DXL_IDS[i], TORQUE_ON);
  }
  
  // mobilebase velocity control mode sync write 준비
  sw_mobile_velocity_infos.packet.p_buf = nullptr;
  sw_mobile_velocity_infos.packet.is_completed = false;
  sw_mobile_velocity_infos.addr = SW_WHEEL_VELOCITY_START_ADDR;
  sw_mobile_velocity_infos.addr_length = SW_WHEEL_VELOCITY_DATA_SIZE;
  sw_mobile_velocity_infos.p_xels = info_wheel_velocity_xels_sw;
  sw_mobile_velocity_infos.xel_count = 0;

  sw_wheel_velocity_data[0].goal_velocity = 0; // 모터에 write 할 데이터 초기화
  sw_wheel_velocity_data[1].goal_velocity = 0;
  sw_wheel_velocity_data[2].goal_velocity = 0;
  sw_wheel_velocity_data[3].goal_velocity = 0;

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < MOBILE_DXL_ID_CNT; i++) {
    info_wheel_velocity_xels_sw[i].id = MOBILE_DXL_IDS[i];
    info_wheel_velocity_xels_sw[i].p_data = (uint8_t*)&sw_wheel_velocity_data[i];
    sw_mobile_velocity_infos.xel_count++;
  }
  sw_mobile_velocity_infos.is_info_changed = true; // sync write 정보가 변경됨을 설정
  
  // mobilebase velocity extended position control mode sync write 준비
  sw_mobile_position_infos.packet.p_buf = nullptr;
  sw_mobile_position_infos.packet.is_completed = false;
  sw_mobile_position_infos.addr = SW_WHEEL_POSITION_START_ADDR;
  sw_mobile_position_infos.addr_length = SW_WHEEL_POSITION_DATA_SIZE;
  sw_mobile_position_infos.p_xels = info_wheel_position_xels_sw;
  sw_mobile_position_infos.xel_count = 0;

  sw_wheel_position_data[0].profile_velocity = MOBILEBASE_SPEED; // 모터에 write 할 데이터 초기화
  sw_wheel_position_data[0].goal_position = dxl.readControlTableItem(PRESENT_POSITION, MOBILE_DXL_IDS[0]);
  sw_wheel_position_data[1].profile_velocity = MOBILEBASE_SPEED;
  sw_wheel_position_data[1].goal_position = dxl.readControlTableItem(PRESENT_POSITION, MOBILE_DXL_IDS[1]);
  sw_wheel_position_data[2].profile_velocity = MOBILEBASE_SPEED;
  sw_wheel_position_data[2].goal_position = dxl.readControlTableItem(PRESENT_POSITION, MOBILE_DXL_IDS[2]);
  sw_wheel_position_data[3].profile_velocity = MOBILEBASE_SPEED;
  sw_wheel_position_data[3].goal_position = dxl.readControlTableItem(PRESENT_POSITION, MOBILE_DXL_IDS[3]);

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < MOBILE_DXL_ID_CNT; i++) {
    info_wheel_position_xels_sw[i].id = MOBILE_DXL_IDS[i];
    info_wheel_position_xels_sw[i].p_data = (uint8_t*)&sw_wheel_position_data[i];
    sw_mobile_position_infos.xel_count++;
  }
  sw_mobile_position_infos.is_info_changed = true; // sync write 정보가 변경됨을 설정
  
  // mobilebase extended position control mode sync read 준비
  sr_mobile_position_infos.packet.buf_capacity = wheel_position_pkt_buf_cap;
  sr_mobile_position_infos.packet.p_buf = wheel_position_pkt_buf;
  sr_mobile_position_infos.packet.is_completed = false;
  sr_mobile_position_infos.addr = SR_WHEEL_POSITION_START_ADDR;
  sr_mobile_position_infos.addr_length = SR_WHEEL_POSITION_DATA_SIZE;
  sr_mobile_position_infos.p_xels = info_wheel_position_xels_sr;
  sr_mobile_position_infos.xel_count = 0;

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < MOBILE_DXL_ID_CNT; i++) {
    info_wheel_position_xels_sr[i].id = MOBILE_DXL_IDS[i];
    info_wheel_position_xels_sr[i].p_recv_buf = (uint8_t*)&sr_wheel_position_data[i];
    sr_mobile_position_infos.xel_count++;
  }
  sr_mobile_position_infos.is_info_changed = true; // sync read 정보가 변경됨을 설정
  
  // mobilebase moving sync read 준비
  sr_mobile_moving_infos.packet.buf_capacity = moving_pkt_buf_cap;
  sr_mobile_moving_infos.packet.p_buf = moving_pkt_buf;
  sr_mobile_moving_infos.packet.is_completed = false;
  sr_mobile_moving_infos.addr = SR_WHEEL_MOVING_START_ADDR;
  sr_mobile_moving_infos.addr_length = SR_WHEEL_MOVING_DATA_SIZE;
  sr_mobile_moving_infos.p_xels = info_wheel_moving_xels_sr;
  sr_mobile_moving_infos.xel_count = 0;

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < MOBILE_DXL_ID_CNT; i++) {
    info_wheel_moving_xels_sr[i].id = MOBILE_DXL_IDS[i];
    info_wheel_moving_xels_sr[i].p_recv_buf = (uint8_t*)&sr_wheel_moving_data[i];
    sr_mobile_moving_infos.xel_count++;
  }
  sr_mobile_moving_infos.is_info_changed = true; // sync read 정보가 변경됨을 설정
  
  return true;
}

void ChangeMobilebaseMode2VelocityControlMode(Dynamixel2Arduino& dxl) {
  // 각 모터에 설정
  for (int i = 0 ; i < MOBILE_DXL_ID_CNT ; i++) {
    // 토크 끄기
    dxl.writeControlTableItem(TORQUE_ENABLE, MOBILE_DXL_IDS[i], TORQUE_OFF);// 각 모터에 설정
    // 모드 설정
    if (MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_FL ||
        MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_BL) { // 좌측 바퀴
      dxl.writeControlTableItem(DRIVE_MODE, MOBILE_DXL_IDS[i], NORMAL_MODE + VELOCITY_BASED_PROFILE);
    } else if (MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_FR ||
               MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_BR) { // 우측 바퀴
      dxl.writeControlTableItem(DRIVE_MODE, MOBILE_DXL_IDS[i], REVERSE_MODE + VELOCITY_BASED_PROFILE);
    }
    dxl.writeControlTableItem(OPERATING_MODE, MOBILE_DXL_IDS[i], VELOCITY_CONTROL_MODE);
    
    // 토크 켜기
    dxl.writeControlTableItem(TORQUE_ENABLE, MOBILE_DXL_IDS[i], TORQUE_ON);
  }
}

void ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(Dynamixel2Arduino& dxl) {
  // 각 모터에 설정
  for (int i = 0 ; i < MOBILE_DXL_ID_CNT ; i++) {
    // 토크 끄기
    dxl.writeControlTableItem(TORQUE_ENABLE, MOBILE_DXL_IDS[i], TORQUE_OFF);
    // 모드 설정
    if (MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_FL ||
        MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_BL) { // 좌측 바퀴
      dxl.writeControlTableItem(DRIVE_MODE, MOBILE_DXL_IDS[i], NORMAL_MODE + TIME_BASED_PROFILE);
    } else if (MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_FR ||
               MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_BR) { // 우측 바퀴
      dxl.writeControlTableItem(DRIVE_MODE, MOBILE_DXL_IDS[i], REVERSE_MODE + TIME_BASED_PROFILE);
    }
    dxl.writeControlTableItem(OPERATING_MODE, MOBILE_DXL_IDS[i], EXTENDED_POSITION_CONTROL_MODE);
    // 토크 켜기
    dxl.writeControlTableItem(TORQUE_ENABLE, MOBILE_DXL_IDS[i], TORQUE_ON);
  }
}

bool FindMobileBaseServos(Dynamixel2Arduino& dxl)
{
  uint8_t ids_pinged[10] = {0,};
  const uint8_t count_pinged = dxl.ping(DXL_BROADCAST_ID, ids_pinged, sizeof(ids_pinged), 100);
  bool allFound = true;
  for (uint8_t expected = 0; expected < MOBILE_DXL_ID_CNT; ++expected) {
    bool found = false;
    for (uint8_t received = 0; received < count_pinged && received < sizeof(ids_pinged); ++received) {
      if (ids_pinged[received] == MOBILE_DXL_IDS[expected]) {
        found = true;
        break;
      }
    }
    if (!found) {
#ifdef DEBUG
      DEBUG_SERIAL.print(F("Missing motor ID: "));
      DEBUG_SERIAL.println(MOBILE_DXL_IDS[expected]);
#endif
      allFound = false;
    }
  }
  return allFound;
}

bool CheckIfMobilebaseIsInPosition(Dynamixel2Arduino& dxl)
{
  for (uint8_t attempt = 0; attempt < MOBILE_READ_RETRY_LIMIT; ++attempt) {
    if (dxl.syncRead(&sr_mobile_moving_infos) == MOBILE_DXL_ID_CNT) {
      for (uint8_t i = 0; i < MOBILE_DXL_ID_CNT; ++i) {
        if (!(sr_wheel_moving_data[i].moving_status & 0x01))
          return false;
      }
      return true;
    }
  }
  return false;  // 통신 실패는 도착 성공이 아님. 바깥 도착 대기 시간 검사로 복귀.
}


float wheelRate[4][4] = {
  { WHEEL_RATIO_FWD[0], WHEEL_RATIO_FWD[1], WHEEL_RATIO_FWD[2], WHEEL_RATIO_FWD[3] },
  { WHEEL_RATIO_BWD[0], WHEEL_RATIO_BWD[1], WHEEL_RATIO_BWD[2], WHEEL_RATIO_BWD[3] },
  { WHEEL_RATIO_LEFT[0], WHEEL_RATIO_LEFT[1], WHEEL_RATIO_LEFT[2], WHEEL_RATIO_LEFT[3] },
  { WHEEL_RATIO_RIGHT[0], WHEEL_RATIO_RIGHT[1], WHEEL_RATIO_RIGHT[2], WHEEL_RATIO_RIGHT[3] }
};

void DriveDistanceSmoothWithInternalAcceleration( Dynamixel2Arduino& dxl,
                  float distance, uint8_t drivingDirection, int32_t mmPerSec )
{
  int32_t motorValueForRotation = RADIANS_2_MOTOR_VALUE(distance / WHEEL_RADIUS_MM);
  uint32_t moveTime = (distance / mmPerSec) * S_TO_MILLIS_RATIO;

  // 1. 가속/속도 프로파일 설정 (내부 가속 곡선 사용)
  // 시간 기반 모드에서 PROFILE_ACCELERATION은 가속 시간(ms), PROFILE_VELOCITY는 전체 이동 시간(ms).

  const uint32_t profileAcceleration = MOBILEBASE_PROFILE_ACCELERATION_MS;  // motorAdjust와 같은 100 ms
  const uint32_t profileVelocity = moveTime; // 이동 시간과 동일하게 설정

  for (int i = 0 ; i < MOBILE_DXL_ID_CNT ; i++) {
    dxl.writeControlTableItem(PROFILE_ACCELERATION, MOBILE_DXL_IDS[i], profileAcceleration);
    dxl.writeControlTableItem(PROFILE_VELOCITY, MOBILE_DXL_IDS[i], profileVelocity);
  }

  // 2. 주행 명령 실행
  bool commandSent = false;
  switch( drivingDirection )
  {
    case MOVE_FORWARD:
      commandSent = SetMobileRelativePositionForSyncWrite( dxl,
        motorValueForRotation * wheelRate[0][0], motorValueForRotation * wheelRate[0][1],
        motorValueForRotation * wheelRate[0][2], motorValueForRotation * wheelRate[0][3], moveTime );
      break;
    case MOVE_BACKWARD:
      commandSent = SetMobileRelativePositionForSyncWrite( dxl,
        -motorValueForRotation * wheelRate[1][0], -motorValueForRotation * wheelRate[1][1],
        -motorValueForRotation * wheelRate[1][2], -motorValueForRotation * wheelRate[1][3], moveTime );
      break;
    case MOVE_LEFTWARD:
      commandSent = SetMobileRelativePositionForSyncWrite( dxl,
        -motorValueForRotation * wheelRate[2][0],  motorValueForRotation * wheelRate[2][1],
         motorValueForRotation * wheelRate[2][2], -motorValueForRotation * wheelRate[2][3], moveTime );
      break;
    case MOVE_RIGHTWARD:
      commandSent = SetMobileRelativePositionForSyncWrite( dxl,
         motorValueForRotation * wheelRate[3][0], -motorValueForRotation * wheelRate[3][1],
        -motorValueForRotation * wheelRate[3][2],  motorValueForRotation * wheelRate[3][3], moveTime );
      break;
    case ROTATE_CCW:  // 우측 모터는 REVERSE_MODE: 좌측 후진 / 우측 전진
      commandSent = SetMobileRelativePositionForSyncWrite( dxl,
          -motorValueForRotation, motorValueForRotation,
          -motorValueForRotation, motorValueForRotation, moveTime );
      break;
    case ROTATE_CW:
      commandSent = SetMobileRelativePositionForSyncWrite( dxl,
          motorValueForRotation, -motorValueForRotation,
          motorValueForRotation, -motorValueForRotation, moveTime );
      break;
  }

  if (!commandSent)
    return;  // 보내지 못한 명령의 도착을 기다리지 않고 미션 순서로 복귀

  // 3. 도착 대기. 초과 시 대기만 종료하며 모터 목표를 취소하지는 않음.
  const uint32_t waitStarted = millis();
  const uint32_t waitLimit = moveTime + MOBILE_MOVE_TIMEOUT_MARGIN_MS + moveTime / 4;
  while (!CheckIfMobilebaseIsInPosition(dxl)) {
    yield();
    if (uint32_t(millis() - waitStarted) >= waitLimit) {
      tone(BUZZER_PIN, 784, 100);  // 도착 시간 초과: 높은 음, 대기 없이 진행
#ifdef DEBUG
      DEBUG_SERIAL.println(F("Move wait timed out; continuing."));
#endif
      break;
    }
  }
}


// 메카넘휠 회전의 바퀴 이동 거리 = (앞뒤 간격 + 좌우 간격) / 2 * 회전각(rad).
static void RotateInPlace( Dynamixel2Arduino& dxl, uint8_t direction, int32_t mmPerSec, float adjustDegrees )
{
  if( mmPerSec <= 0 ) return;
  const float wheelTravelMm = (MOBILEBASE_WHEEL_BASE_MM + MOBILEBASE_TRACK_WIDTH_MM) * 0.5f
      * ((90.0f + adjustDegrees) * PI / 180.0f);
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode( dxl );
  DriveDistanceSmoothWithInternalAcceleration( dxl, wheelTravelMm, direction, mmPerSec );
}

void RotateLeft90Degree( Dynamixel2Arduino& dxl, int32_t mmPerSec, float adjustDegrees )
{
  RotateInPlace( dxl, ROTATE_CCW, mmPerSec, adjustDegrees );
}

void RotateRight90Degree( Dynamixel2Arduino& dxl, int32_t mmPerSec, float adjustDegrees )
{
  RotateInPlace( dxl, ROTATE_CW, mmPerSec, adjustDegrees );
}


bool RotateAlignWithFront2PSD( Dynamixel2Arduino& dxl,
                              int16_t moveSensorError, int16_t rotationSensorError,
                              int16_t moveToleranceValue, int16_t rotationToleranceValue,
                              float moveControlRatio, float roatingControlRatio,
                              bool resetStableCount, int32_t drivingSpeed )
{
  static uint8_t stableCount = 0;
  
  if( resetStableCount ) stableCount = 0;

  // 오차가 허용 범위 내인지 확인 (안정화 카운트 사용)
  if( abs(moveSensorError) <= moveToleranceValue  &&  abs(rotationSensorError) <= rotationToleranceValue )
  {
    if( ++stableCount >= ALIGN_STABLE_COUNT ) {
      SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );
      return false;  // 완전히 안정화됨
    }
  } else {
    stableCount = 0;  // 조건 불만족 시 리셋
  }

  // 제어값 계산 (게인 조정)
  float moveDrivingControl = (float)moveSensorError * moveControlRatio;
  float rotatingControl = (float)rotationSensorError * roatingControlRatio;

  // 제어값 클램핑 [-1.0 ~ 1.0]
  moveDrivingControl = constrain(moveDrivingControl, -1.0f, 1.0f);
  rotatingControl = constrain(rotatingControl, -1.0f, 1.0f);

  // 모터 속도 계산
  int32_t fl_velocity = round((moveDrivingControl + rotatingControl) * drivingSpeed);
  int32_t fr_velocity = round((moveDrivingControl - rotatingControl) * drivingSpeed);
  int32_t bl_velocity = round((moveDrivingControl + rotatingControl) * drivingSpeed);
  int32_t br_velocity = round((moveDrivingControl - rotatingControl) * drivingSpeed);

  // 속도 제한 적용
  fl_velocity = constrain(fl_velocity, -drivingSpeed, drivingSpeed);
  fr_velocity = constrain(fr_velocity, -drivingSpeed, drivingSpeed);
  bl_velocity = constrain(bl_velocity, -drivingSpeed, drivingSpeed);
  br_velocity = constrain(br_velocity, -drivingSpeed, drivingSpeed);

  // 유의미한 움직임 여부 확인
  bool willMove = (abs(fl_velocity) > movelessSpeed) || (abs(fr_velocity) > movelessSpeed) ||
                  (abs(bl_velocity) > movelessSpeed) || (abs(br_velocity) > movelessSpeed) ;

  if(willMove)
    SetMobileGoalVelocityForSyncWrite(dxl, fl_velocity, fr_velocity, bl_velocity, br_velocity);
  else
    SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);

  delay( alignInnerLoopDelay );
  return true;
}


bool  MoveAlignWithFrontRightPSD( Dynamixel2Arduino& dxl,
                                  int psdError, int tolerance, float controlRatio,
                                  bool resetStableCount, int32_t moveSpeed )
{
  static uint8_t stableCount = 0;
  
  if( resetStableCount ) stableCount = 0;

  // 오차가 허용 범위 내인지 확인 (안정화 카운트 사용)
  if( abs(psdError) <= tolerance )
  {
    if( ++stableCount >= ALIGN_STABLE_COUNT ) {
      SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );
      return false;  // 완전히 안정화됨
    }
  } else {
    stableCount = 0;  // 조건 불만족 시 리셋
  }

  // 이동 방향 결정
  uint8_t direction = (psdError > 0) ? MOVE_FORWARD : MOVE_BACKWARD;

  // 속도 비례 계산
  float ratio = controlRatio * psdError;
  if (abs(ratio) > 1.0) ratio /= abs(ratio);  // 최대값 ±1 제한

  int32_t speed = round(moveSpeed * abs(ratio));  // 항상 양수로 사용

  // 허용 범위 밖이면 작은 비례 출력도 최소 유효 속도로 보정한다 (SL 보정과 같은 규칙).
  // 현재 호출 속도 160의 상한은 유지하며, 허용 범위 안의 정지는 그대로 둔다.
  if( speed < movelessSpeed )
  {
    if( abs(psdError) > tolerance && moveSpeed >= movelessSpeed ) {
      speed = movelessSpeed;
    } else {
      SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
      return false;
    }
  }

  // 주행 명령
  if (direction == MOVE_FORWARD) {
    SetMobileGoalVelocityForSyncWrite( dxl, speed, speed, speed, speed );  // 전진
  } else {
    SetMobileGoalVelocityForSyncWrite( dxl, -speed, -speed, -speed, -speed );  // 후진
  }

  delay( alignInnerLoopDelay );  // 적절한 딜레이 (너무 길면 overshoot 유발)
  return  true;
}

bool  MoveAlignWithLeftSidePSD( Dynamixel2Arduino& dxl,
                                  int psdError, int tolerance, float controlRatio,
                                  bool resetStableCount, int32_t moveSpeed )
{
  static uint8_t stableCount = 0;
  
  if( resetStableCount ) stableCount = 0;

  // 오차가 허용 범위 내인지 확인 (안정화 카운트 사용)
  if( abs(psdError) <= tolerance )
  {
    if( ++stableCount >= ALIGN_STABLE_COUNT ) {
      SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );
      return false;  // 완전히 안정화됨
    }
  } else {
    stableCount = 0;  // 조건 불만족 시 리셋
  }

  // 이동 방향 결정
  uint8_t direction = (psdError > 0) ? MOVE_LEFTWARD : MOVE_RIGHTWARD;

  // 속도 비례 계산
  float ratio = controlRatio * psdError;
  if (abs(ratio) > 1.0) ratio /= abs(ratio);  // 최대값 ±1 제한

  int32_t speed = round(moveSpeed * abs(ratio));  // 항상 양수로 사용

  // 허용 범위 밖이면 작은 비례 출력도 최소 유효 속도로 보정한다.
  // 현재 호출 속도 100/160의 상한은 유지하며, 허용 범위 안의 정지는 그대로 둔다.
  if( speed < movelessSpeed )
  {
    if( abs(psdError) > tolerance && moveSpeed >= movelessSpeed ) {
      speed = movelessSpeed;
    } else {
      SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
      return false;
    }
  }

  // 주행 명령
  if (direction == MOVE_LEFTWARD) {
    SetMobileGoalVelocityForSyncWrite(dxl, -speed, speed, speed, -speed);  // 좌진
  } else {
    SetMobileGoalVelocityForSyncWrite(dxl, speed, -speed, -speed, speed);  // 우진
  }

  delay( alignInnerLoopDelay );  // 적절한 딜레이 (너무 길면 overshoot 유발)
  return  true;
}


bool  MinusMoveAlignWithLeftSidePSD( Dynamixel2Arduino& dxl,
                                     int psdError, int tolerance, float controlRatio, bool resetStableCount )  // int32_t moveSpeed
{
  static uint8_t stableCount = 0;
  
  if( resetStableCount ) stableCount = 0;

  // 오차가 허용 범위 내인지 확인 (안정화 카운트 사용)
  if( abs(psdError) <= tolerance )
  {
    if( ++stableCount >= ALIGN_STABLE_COUNT ) {
      SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );
      return false;  // 완전히 안정화됨
    }
  } else {
    stableCount = 0;  // 조건 불만족 시 리셋
  }

  // 이동 방향 결정
  uint8_t direction = (psdError > 0) ? MOVE_RIGHTWARD : MOVE_LEFTWARD;

  // 속도 비례 계산
  float ratio = controlRatio * psdError;
  if (abs(ratio) > 1.0) ratio /= abs(ratio);  // 최대값 ±1 제한

  int32_t speed = round(MOBILEBASE_SPEED * abs(ratio));  // 항상 양수로 사용, MOBILEBASE_SPEED로 고정!

  // 유의미한 움직임 여부 확인
  if( speed < movelessSpeed )
  {
    SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
    return false;  // 너무 미세한 조정은 생략
  }

  // 주행 명령
  if (direction == MOVE_LEFTWARD) {
    SetMobileGoalVelocityForSyncWrite(dxl, -speed, speed, speed, -speed);  // 좌진
  } else {
    SetMobileGoalVelocityForSyncWrite(dxl, speed, -speed, -speed, speed);  // 우진
  }

  delay( alignInnerLoopDelay );  // 적절한 딜레이 (너무 길면 overshoot 유발)

  return  true;
}


bool SetMobileRelativePositionForSyncWrite(Dynamixel2Arduino& dxl,
                                            int32_t fl_relative_position, int32_t fr_relative_position,
                                            int32_t bl_relative_position, int32_t br_relative_position,
                                            int32_t drivingTime)
{
  bool positionsRead = false;
  for (uint8_t attempt = 0; attempt < MOBILE_POSITION_READ_RETRY_LIMIT; ++attempt) {
    if (dxl.syncRead(&sr_mobile_position_infos) == MOBILE_DXL_ID_CNT) {
      positionsRead = true;
      break;
    }
  }
  if (!positionsRead) {
    tone(BUZZER_PIN, 330, 300);  // 이동 생략: 낮은 음, 대기 없이 복귀
#ifdef DEBUG
    DEBUG_SERIAL.println(F("Move skipped: position read failed."));
#endif
    return false;  // 부분 수신/이전 위치로 새 목표를 만들지 않음
  }

  if (abs(sr_wheel_position_data[0].present_position + fl_relative_position) > 1048575
      || abs(sr_wheel_position_data[1].present_position + fr_relative_position) > 1048575
      || abs(sr_wheel_position_data[2].present_position + bl_relative_position) > 1048575
      || abs(sr_wheel_position_data[3].present_position + br_relative_position) > 1048575) {
    tone(BUZZER_PIN, 330, 300);
#ifdef DEBUG
    DEBUG_SERIAL.println(F("Move skipped: position limit."));
#endif
    return false;  // 이전 목표 재전송 방지
  }
  sw_wheel_position_data[0].profile_velocity = drivingTime;
  sw_wheel_position_data[0].goal_position = sr_wheel_position_data[0].present_position + fl_relative_position;
  sw_wheel_position_data[1].profile_velocity = drivingTime;
  sw_wheel_position_data[1].goal_position = sr_wheel_position_data[1].present_position + fr_relative_position;
  sw_wheel_position_data[2].profile_velocity = drivingTime;
  sw_wheel_position_data[2].goal_position = sr_wheel_position_data[2].present_position + bl_relative_position;
  sw_wheel_position_data[3].profile_velocity = drivingTime;
  sw_wheel_position_data[3].goal_position = sr_wheel_position_data[3].present_position + br_relative_position;

  sw_mobile_position_infos.is_info_changed = true;
  for (uint8_t attempt = 0; attempt <= 10; ++attempt) {  // 최대 11회 전송 시도
    if (dxl.syncWrite(&sw_mobile_position_infos))
      return true;
  }
  tone(BUZZER_PIN, 330, 300);
#ifdef DEBUG
  DEBUG_SERIAL.println(F("Move skipped: command write failed."));
#endif
  return false;
}

void SetMobileGoalVelocityForSyncWrite( Dynamixel2Arduino& dxl,
                                        int32_t fl_goal_velocity, int32_t fr_goal_velocity,
                                        int32_t bl_goal_velocity, int32_t br_goal_velocity )
{
  sw_wheel_velocity_data[0].goal_velocity = fl_goal_velocity;
  sw_wheel_velocity_data[1].goal_velocity = fr_goal_velocity;
  sw_wheel_velocity_data[2].goal_velocity = bl_goal_velocity;
  sw_wheel_velocity_data[3].goal_velocity = br_goal_velocity;

  sw_mobile_velocity_infos.is_info_changed = true;


  int count = 0;
  while (!dxl.syncWrite(&sw_mobile_velocity_infos)) {
    count++;
    delay( 1 );
    if (count > 10) break;
  }
}


