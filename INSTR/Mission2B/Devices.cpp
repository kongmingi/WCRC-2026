#include "Arduino.h"
#include "Debug.h"
#include "Motor.h"
#include "Devices.h"

/////////////////////////////////////////////  디버그 관련

void InitDebug() {
#ifdef DEBUG
  DEBUG_SERIAL.begin( SERIAL_BAUD_RATE);
#endif
}

/////////////////////////////////////////////  그리퍼 관련

void OpenGripper(Pixy2SPI_SS& pixy) {
  pixy.setServos(0, GRIP_ANGLE_OPEN);
}

void CloseGripper(Pixy2SPI_SS& pixy) {
  pixy.setServos(0, GRIP_ANGLE_CLOSE);
}


/////////////////////////////////////////////  PSD 관련

static int lValues[MAX_PSD_MEASURE_COUNT];
static int rValues[MAX_PSD_MEASURE_COUNT];


void InitPSD() {
  // 아날로그 핀은 핀모드 설정 필요 없음
}

///// 중앙값 필터(median filter)

int GetMedian( int* values, int count )
{
  // 버블 정렬로 오름차순 정렬
  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (values[i] > values[j]) {
        int temp = values[i];
        values[i] = values[j];
        values[j] = temp;
      }
    }
  }

  // 중앙값 반환
  if( count % 2 == 1 )
    return values[count / 2];  // 홀수: 가운데 값

  return (values[count / 2 - 1] + values[count / 2]) / 2;  // 짝수: 가운데 두 값의 평균
}

void GetValueFromFrontPSDSensors( int16_t* flPSDValuePtr, int16_t* frPSDValuePtr, int nRepeat )
{
  for (int i = 0; i < nRepeat; i++) {
    lValues[i] = analogRead(PIN_FRONT_LEFT_PSD);
    rValues[i] = analogRead(PIN_FRONT_RIGHT_PSD);
    delay( PSD_MEASURE_DELAY );
  }

  *flPSDValuePtr = GetMedian(lValues, nRepeat);
  *frPSDValuePtr = GetMedian(rValues, nRepeat);
}

void GetValueFromFrontLeftPSDSensor( int16_t* flPSDValuePtr, int nRepeat )
{
  for (int i = 0; i < nRepeat; i++) {
    lValues[i] = analogRead(PIN_FRONT_LEFT_PSD);
    delay( PSD_MEASURE_DELAY );
  }

  *flPSDValuePtr = GetMedian(lValues, nRepeat);
}

void GetValueFromFrontRightPSDSensor( int16_t* frPSDValuePtr, int nRepeat )
{
  for (int i = 0; i < nRepeat; i++) {
    rValues[i] = analogRead(PIN_FRONT_RIGHT_PSD);
    delay( PSD_MEASURE_DELAY );
  }

  *frPSDValuePtr = GetMedian(rValues, nRepeat);
}

void GetValueFromSideLeftPSDSensor( int16_t* slPSDValuePtr, int nRepeat )
{
  for (int i = 0; i < nRepeat; i++) {
    lValues[i] = analogRead(PIN_SIDE_LEFT_PSD);
    delay( PSD_MEASURE_DELAY );
  }

  *slPSDValuePtr = GetMedian(lValues, nRepeat);
}

void GetValueFromSideRightPSDSensor( int16_t* srPSDValuePtr, int nRepeat )  // FIXED
{
  for( int i = 0; i < nRepeat; i++ ) {
    rValues[i] = analogRead( PIN_SIDE_RIGHT_PSD );
    delay( PSD_MEASURE_DELAY );
  }

  *srPSDValuePtr = GetMedian( rValues, nRepeat );
}


void InitPixy(Pixy2SPI_SS& pixy) {
  pixy.init(); // pixy객체 초기화
}


/////////////////////////////////////////////  모터  관련

void InitMotorCommunication(Dynamixel2Arduino& dxl) {
  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
}
