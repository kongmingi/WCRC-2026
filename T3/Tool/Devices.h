#ifndef DEVICES_H
#define DEVICES_H

#include "Arduino.h"
#include <Pixy2.h> // SPI 통신하는 Pixy2 라이브러리를 포함시킴
#include <Dynamixel2Arduino.h>

/////////////////////////////////////////////  사용 핀 정의

/////////////  사용자 RED LED
#define RED_LED_PIN   4

//////////////  사용자 Swtich
#define SW1_PIN       41
#define SW2_PIN       40

//////////////  부저
#define BUZZER_PIN    5

//////////////  RGB LED
#define RGB_LED_PIN   3

//////////////  PSD
#define PIN_FRONT_LEFT_PSD    A0 // 전방 좌측 PSD센서 연결 핀
#define PIN_SIDE_LEFT_PSD     A1 // 좌측면 PSD센서 연결 핀
#define PIN_FRONT_RIGHT_PSD   A2 // 전방 우측 PSD센서 연결 
#define PIN_SIDE_RIGHT_PSD    A3 // 우측면 PSD센서 연결 핀

/////////////////////////////////////////////  그리퍼 관련

#define GRIP_ANGLE_CLOSE          955   // 그리퍼를 반쯤 닫아서 블록 잡기 (최대로 꽉 닫기는 1000)
#define GRIP_ANGLE_OPEN           100   // 그리퍼 열리는 값 (더 작으면 소리가 나고, 기구적 문제로 모터에 무리가 감)

void OpenGripper(Pixy2& pixy);
void CloseGripper(Pixy2& pixy);

/////////////////////////////////////////////  PSD  관련

#define MAX_PSD_MEASURE_COUNT   11  // 배열 변수는 최대 갯수로 잡아두자
#define PSD_MEASURE_COUNT       11  // 반드시 3 이상 일 것 (중앙값 필터 사용 시는 홀수값 권장)
#define PSD_MEASURE_DELAY       1   // ms

void InitPSD();

int GetMedian( int* values, int count );

/* 
 * 전방 두 개의 PSD 센서에서 값을 받아 업데이트하는 함수. 중앙값 필터 적용
 * params :
 *    flPSDValuePtr : 전방좌측 PSD 센서값을 업데이트할 변수 포인터
 *    frPSDValuePtr : 전방우측 PSD 센서값을 업데이트할 변수 포인터
 */
void GetValueFromFrontPSDSensors( int16_t* flPSDValuePtr, int16_t* frPSDValuePtr, int nRepeat= PSD_MEASURE_COUNT );

/* 
 * 전방좌측 PSD 센서에서 값을 받아 업데이트하는 함수. 중앙값 필터 적용
 * params :
 *    flPSDValuePtr : 전방좌측 PSD 센서값을 업데이트할 변수 포인터
 */
void GetValueFromFrontLeftPSDSensor( int16_t* flPSDValuePtr, int nRepeat= PSD_MEASURE_COUNT );

/* 
 * 전방우측 PSD 센서에서 값을 받아 업데이트하는 함수. 중앙값 필터 적용
 * params :
 *    frPSDValuePtr : 전방우측 PSD 센서값을 업데이트할 변수 포인터
 */
void GetValueFromFrontRightPSDSensor( int16_t* frPSDValuePtr, int nRepeat= PSD_MEASURE_COUNT );

/* 
 * 좌측면 PSD 센서에서 값을 받아 업데이트하는 함수. 중앙값 필터 적용
 * params :
 *    slPSDValuePtr : 좌측면 PSD 센서값을 업데이트할 변수 포인터
 */
void GetValueFromSideLeftPSDSensor( int16_t* slPSDValuePtr, int nRepeat= PSD_MEASURE_COUNT );

// 우측면 PSD 센서값을 중앙값 필터로 읽는다.
void GetValueFromSideRightPSDSensor( int16_t* srPSDValuePtr, int nRepeat= PSD_MEASURE_COUNT );  // MODIFIED


#define PIXY_CCC_X_MIN      0
#define PIXY_CCC_X_MAX      315

struct BlockPosition {
  int x;
  int y;
  int w;
  int h;
};

//////////////  Pixy 함수 선언
/* 
 * Pixy를 초기화 하는 함수
 */
void InitPixy(Pixy2& pixy);


#endif
