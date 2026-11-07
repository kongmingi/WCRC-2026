#ifndef MOTOR_H
#define MOTOR_H

//////////////  모터 구동 관련 선언
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial1

const int DXL_DIR_PIN = 2; // DIR PIN
const float DXL_PROTOCOL_VERSION = 2.0;

// poseMon 전용 설정. Mission/MissionConfig.h와 독립적이며 모터 EEPROM 속도는 변경하지 않는다.
// false: 57600 bps(기본값), true: 1M bps. 팔 모터 ID 5~8의 실제 속도와 맞출 것.

#define USE_DXL_1M_BPS    true

const unsigned long DXL_BAUDRATE = USE_DXL_1M_BPS ? 1000000UL : 57600UL;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// TORQUE_ENABLE 관련
#define TORQUE_ON                 0x01
#define TORQUE_OFF                0x00

// DRIVE_MODE 관련
#define NORMAL_MODE                 0x00 // drive mode의 방향 모드
#define REVERSE_MODE                0x01

#define VELOCITY_BASED_PROFILE      0x00 // profile configuration
#define TIME_BASED_PROFILE          0x04

// OPERATING_MODE 관련
#define VELOCITY_CONTROL_MODE       0x01
#define POSITION_CONTROL_MODE       0x03
#define EXTENDED_POSITION_CONTROL_MODE      0x04


// 컨트롤 테이블 아이템의 이름을 사용하기 위해 이 네임스페이스가 필요함
using namespace ControlTableItem;

/*
 * 모터 통신을 초기화 하는 함수
 */
void initMotorCommunication() {
  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
}

#endif
