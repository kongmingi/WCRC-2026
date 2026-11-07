#ifndef MOTOR_H
#define MOTOR_H

#include <Dynamixel2Arduino.h>

//////////////  모터 구동 관련 선언

#define DXL_SERIAL   Serial1

const int DXL_DIR_PIN = 2; // DIR PIN

const float DXL_PROTOCOL_VERSION = 2.0;
const unsigned long DXL_BAUDRATE = 1000000;   // 57600 또는 1000000;

// BAUD_RATE 관련
#define BAUD_RATE_57600           0x01
#define BAUD_RATE_1M              0x03

// TORQUE_ENABLE 관련
#define TORQUE_ON                 0x01
#define TORQUE_OFF                0x00

// DRIVE_MODE 관련
#define NORMAL_MODE                 0x00 // drive mode의 방향 모드
#define REVERSE_MODE                0x01

#define VELOCITY_BASED_PROFILE      0x00 // profile configuration
#define TIME_BASED_PROFILE          0x04

// OPERATING_MODE 관련
#define VELOCITY_CONTROL_MODE               0x01
#define POSITION_CONTROL_MODE               0x03
#define EXTENDED_POSITION_CONTROL_MODE      0x04


// 컨트롤 테이블 아이템의 이름을 사용하기 위해 이 네임스페이스가 필요함
using namespace ControlTableItem;

/*
 * 모터 통신을 초기화 하는 함수
 */
void InitMotorCommunication(Dynamixel2Arduino& dxl);

#endif
