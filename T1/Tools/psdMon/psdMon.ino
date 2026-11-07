//////////////  PSD 센서 모니터
//
//  전방 좌/우(FL/FR), 측면 좌/우(SL/SR) PSD의 아날로그 원시값을 Mission과 같은 방식
//  (표본 11개의 중앙값)으로 출력한다. [min~max]는 표본 11개의 흔들림 폭이다.
//  Mission.ino의 PSD_* 기준값(예: PSD_STORAGE_FR 535)은 모두 이 원시값 단위이므로
//  로봇을 목표 위치에 두고 읽은 중앙값을 그대로 옮겨 적는다.
//  SW1을 누를 때마다 시리얼 출력을 중지/계속한다. 시작 시에는 출력이 켜져 있다.

//////////////  PSD센서 핀 (Mission/Devices.h와 같음)
#define PIN_FRONT_LEFT_PSD    A0 // 전방 좌측 PSD센서 연결 핀
#define PIN_SIDE_LEFT_PSD     A1 // 좌측면 PSD센서 연결 핀
#define PIN_FRONT_RIGHT_PSD   A2 // 전방 우측 PSD센서 연결 핀
#define PIN_SIDE_RIGHT_PSD    A3 // 우측면 PSD센서 연결 핀

//////////////  사용자 버튼 (Mission/Devices.h와 같은 핀, 누르면 LOW)
#define SW1_PIN              41
#define BUTTON_DEBOUNCE_MS   30UL

//////////////  측정 설정
#define REPEAT_COUNT        11    // Mission/Devices.h의 PSD_MEASURE_COUNT와 같게 (중앙값 필터는 홀수)
#define SAMPLE_DELAY_MS     1     // Mission/Devices.h의 PSD_MEASURE_DELAY와 같게
#define PRINT_INTERVAL_MS   100   // 출력 주기

#define CAMERA_LAMP_ON      false  // Mission 탐색 중처럼 카메라 램프를 켠 상태로 측정. 카메라 없이 쓰려면 false

#if CAMERA_LAMP_ON
#include <Pixy2SPI_SS.h> // 램프를 켜기 위한 용도로만 사용
Pixy2SPI_SS pixy;
#endif

static bool serialOutputEnabled = true;
static int sw1RawState = HIGH;
static int sw1StableState = HIGH;
static unsigned long sw1LastChangeMs = 0;


//////////////  메인 프로그램

void setup()
{
  Serial.begin( 115200 );
  pinMode( SW1_PIN, INPUT_PULLUP );

#if CAMERA_LAMP_ON
  if (pixy.init() < 0)   // 카메라가 없으면 5초 뒤 음수 반환, 램프 없이 계속 진행
    Serial.println(F("Pixy not found. measuring without camera lamp."));
  else
    pixy.setLamp( 1, 1 ); // 카메라 램프 켜기
#endif

  Serial.print(F("PSD monitor: median of "));
  Serial.print( REPEAT_COUNT );
  Serial.println(F(" samples [min~max]"));
  Serial.println(F("SW1: pause/resume serial output"));

  sw1RawState = sw1StableState = digitalRead( SW1_PIN );
  sw1LastChangeMs = millis();
}


static int values[REPEAT_COUNT];   // 측정 표본 (정렬용)

void loop()
{
  int fl, fr, sl, sr;
  int flMin, flMax, frMin, frMax, slMin, slMax, srMin, srMax;

  fl = MeasurePSD( PIN_FRONT_LEFT_PSD,  &flMin, &flMax );
  fr = MeasurePSD( PIN_FRONT_RIGHT_PSD, &frMin, &frMax );
  sl = MeasurePSD( PIN_SIDE_LEFT_PSD,   &slMin, &slMax );
  sr = MeasurePSD( PIN_SIDE_RIGHT_PSD,  &srMin, &srMax );

  UpdateOutputToggle();
  if (serialOutputEnabled) {
    Serial.print(F("FL= "));  Serial.print( fl );  PrintRange( flMin, flMax );
    Serial.print(F(", FR= ")); Serial.print( fr ); PrintRange( frMin, frMax );
    Serial.print(F(", SL= ")); Serial.print( sl ); PrintRange( slMin, slMax );
    Serial.print(F(", SR= ")); Serial.print( sr ); PrintRange( srMin, srMax );
    Serial.println();
  }

  WaitWithButton( PRINT_INTERVAL_MS );
}


// 30ms 동안 유지된 눌림만 처리한다. 길게 눌러도 한 번만 전환한다.
void UpdateOutputToggle()
{
  const int rawState = digitalRead( SW1_PIN );
  const unsigned long now = millis();

  if (rawState != sw1RawState) {
    sw1RawState = rawState;
    sw1LastChangeMs = now;
  }
  if (rawState != sw1StableState && now - sw1LastChangeMs >= BUTTON_DEBOUNCE_MS) {
    sw1StableState = rawState;
    if (sw1StableState == LOW)
      serialOutputEnabled = !serialOutputEnabled;
  }
}

// 측정 간격과 출력 대기 중에도 버튼을 읽어 짧은 누름을 놓치지 않는다.
void WaitWithButton( unsigned long durationMs )
{
  const unsigned long startMs = millis();
  while (millis() - startMs < durationMs) {
    UpdateOutputToggle();
    delay( 1 );
  }
}


// 한 센서를 REPEAT_COUNT번 읽어 중앙값을 반환. minOut/maxOut에 표본의 최소/최대를 돌려줌
int MeasurePSD( uint8_t pin, int* minOut, int* maxOut )
{
  for (int i = 0; i < REPEAT_COUNT; i++) {
    values[i] = analogRead( pin );
    WaitWithButton( SAMPLE_DELAY_MS );
  }

  int median = GetMedian( values, REPEAT_COUNT );  // 정렬됨

  *minOut = values[0];
  *maxOut = values[REPEAT_COUNT - 1];

  return median;
}

void PrintRange( int minVal, int maxVal )
{
  Serial.print(F(" ["));
  Serial.print( minVal );
  Serial.print(F("~"));
  Serial.print( maxVal );
  Serial.print(F("]"));
}

// 중앙값 필터 함수 (Mission/Devices.cpp의 GetMedian과 같음). values를 오름차순으로 정렬한다.
int GetMedian( int* values, int count )
{
  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (values[i] > values[j]) {
        int temp = values[i];
        values[i] = values[j];
        values[j] = temp;
      }
    }
  }

  if (count % 2 == 1)
    return values[count / 2];  // 홀수

  return (values[count / 2 - 1] + values[count / 2]) / 2;  // 짝수 평균
}
