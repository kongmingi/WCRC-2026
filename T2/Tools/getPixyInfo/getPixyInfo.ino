//////////////  Pixy2 블록 인식 정보 출력 도구
//
//  Mission의 적재함 탐색(STORAGE_SEARCH 자세)에서 카메라가 보는 값을 그대로 보여 준다.
//  각 블록에 대해 Mission과 같은 기준으로 층(L1/L2/L3)과 ROI 안/밖을 함께 표시하므로
//  적재함 층 경계 교정과 그리퍼 중심 x 교정에 쓴다.
//  기본은 2초 주기의 자동 출력. SW1로 수동/자동 전환, 수동에서는 SW2를 누를 때만 출력한다.
//
//  아래 "Mission.ino와 같은 값" 상수는 Mission.ino를 바꾸면 여기도 같이 바꿀 것.

#include <Pixy2.h> // SPI 통신하는 Pixy2 라이브러리를 포함시킴

Pixy2 pixy;        // SPI 통신하는 Pixy2 객체 생성

//////////////  Mission.ino와 같은 값
#define px_PIXY_SET_POINT_X_ADJUST  8            // 그리퍼 중심이 블록의 중심과 일치할 때 화면 중심에서 우측으로 +8 픽셀 지점에 해당함
#define PIXY_SET_POINT_X            (158 + px_PIXY_SET_POINT_X_ADJUST)   // 166: 그리퍼 중심에 맞춘 화면의 X 중심
#define PIXY_SET_POINT_Y            (104 + 10)   // 114: L1(아래, y>114) / L2(가운데) 구분 기준 y
#define PIXY_SET_POINT_Y_L2L3       (104 - 50)   //  54: L2 / L3(맨 위, y<=54) 구분 기준 y (임시값, 교정 대상)
#define MIN_PIXY_SEARCH_X           (158 - 80)   //  78: 이 범위 밖의 블록은 Mission이 무시함
#define MAX_PIXY_SEARCH_X           (158 + 80)   // 238

#define PRINT_INTERVAL_MS           2000UL       // 자동 모드 출력 주기
#define BUTTON_DEBOUNCE_MS          30UL         // 버튼 상태가 30ms 유지되면 확정

//////////////  사용자 버튼 (Mission/Devices.h와 같은 핀, 누르면 LOW)
#define SW1_PIN                     41           // 수동/자동 모드 전환
#define SW2_PIN                     40           // 수동 모드에서 한 번 출력

struct ButtonState {
  uint8_t pin;
  bool rawState;
  bool stableState;
  unsigned long lastChangeMs;
};

ButtonState sw1 = { SW1_PIN, HIGH, HIGH, 0 };
ButtonState sw2 = { SW2_PIN, HIGH, HIGH, 0 };
bool autoMode = true;
unsigned long lastPrintMs = 0;

// 사용자 구조체를 사용하는 함수는 Arduino 자동 선언에 앞서 직접 선언한다.
void InitButton(ButtonState& button);
bool ButtonPressed(ButtonState& button, unsigned long now);

void InitButton(ButtonState& button)
{
  pinMode(button.pin, INPUT_PULLUP);
  button.rawState = digitalRead(button.pin);
  button.stableState = button.rawState;
  button.lastChangeMs = millis();
}

bool ButtonPressed(ButtonState& button, unsigned long now)
{
  const bool rawState = digitalRead(button.pin);
  if (rawState != button.rawState) {
    button.rawState = rawState;
    button.lastChangeMs = now;
  }
  if (rawState != button.stableState && now - button.lastChangeMs >= BUTTON_DEBOUNCE_MS) {
    button.stableState = rawState;
    return button.stableState == LOW;  // 누르는 순간만 한 번 발생. 길게 눌러도 반복하지 않음
  }
  return false;
}

void PrintMode()
{
  if (autoMode)
    Serial.println(F("Mode: AUTO (every 2s). SW1: manual/auto"));
  else
    Serial.println(F("Mode: MANUAL. SW2: print once; SW1: manual/auto"));
}


//////////////  메인 프로그램

void setup()
{
  Serial.begin(115200);
  InitButton(sw1);
  InitButton(sw2);

  int8_t res = pixy.init(); // pixy 객체 초기화 (카메라가 없으면 5초 뒤 음수 반환)

  if (res < 0) {
    Serial.print(F("Pixy init FAILED, code "));
    Serial.println(res);
    Serial.println(F("SPI 케이블과 전원을 확인하세요. 5초마다 재시도합니다..."));
    while (pixy.init() < 0) { }
  }

  Serial.print(F("Pixy OK. frame "));
  Serial.print(pixy.frameWidth);
  Serial.print(F(" x "));
  Serial.println(pixy.frameHeight);

  if (pixy.getVersion() >= 0)  // version 포인터는 수신 버퍼를 가리키므로 출력 직전에 다시 읽음
    pixy.version->print();

  pixy.setLamp( 1, 1 ); // 카메라 램프 켜기 (Mission 탐색 중과 같은 조명)

  Serial.println(F("layer: y>114 L1, 54<y<=114 L2, y<=54 L3 | ROI: x 78~238 | sig>7 = color code"));
  PrintMode();
  lastPrintMs = millis();  // 준비 완료 2초 후 첫 자동 출력
}


void loop()
{
  const unsigned long now = millis();
  const bool modePressed = ButtonPressed(sw1, now);
  const bool printPressed = ButtonPressed(sw2, now);  // 자동 모드에서도 상태를 갱신해 이전 누름을 재사용하지 않음

  if (modePressed) {
    autoMode = !autoMode;
    lastPrintMs = now;  // 자동 모드로 돌아오면 2초 후부터 출력
    PrintMode();
    return;  // 두 버튼을 동시에 누르면 모드 전환을 우선
  }

  if (autoMode) {
    if (now - lastPrintMs < PRINT_INTERVAL_MS)
      return;
    lastPrintMs = now;
  }
  else if (!printPressed)
    return;

  PrintPixyInfo();
}


void PrintPixyInfo()
{
  int8_t res = pixy.ccc.getBlocks();  // 모든 시그니처(컬러 코드 포함) 검색. 음수면 통신 오류

  if (res < 0) {
    Serial.print(F("Pixy comm error, code "));   // 케이블·전원 문제. 블록이 없는 경우(no block)와 구분됨
    Serial.println(res);
    return;
  }

  int n = pixy.ccc.numBlocks;  // 인식된 블록의 개수

  if (n <= 0) {
    Serial.println(F("no block"));
    return;
  }

  Serial.print(F("Detected "));
  Serial.println( n );

  for( int i = 0; i < n; i++ )
  {
    int sig = pixy.ccc.blocks[i].m_signature;
    int x = pixy.ccc.blocks[i].m_x;
    int y = pixy.ccc.blocks[i].m_y;

    Serial.print(F("  ["));
    Serial.print( i );
    Serial.print(F("] sig= "));
    Serial.print( sig );
    if (sig > 7) Serial.print(F("(color code)"));   // Mission은 시그니처 1~6만 사용
    Serial.print(F(", x= "));
    Serial.print( x );
    Serial.print(F(", y= "));
    Serial.print( y );
    Serial.print(F(", w= "));
    Serial.print( pixy.ccc.blocks[i].m_width );
    Serial.print(F(", h= "));
    Serial.print( pixy.ccc.blocks[i].m_height );
    Serial.print(F(", x-sx= "));
    Serial.print( x - PIXY_SET_POINT_X );     // 0이면 그리퍼 중심
    Serial.print(F(", y-sy= "));
    Serial.print( y - PIXY_SET_POINT_Y );

    // Mission.ino CamFindBlockAndPickup()과 같은 층 판정
    Serial.print(F("  -> "));
    if (y > PIXY_SET_POINT_Y)             Serial.print(F("L1"));
    else if (y <= PIXY_SET_POINT_Y_L2L3)  Serial.print(F("L3"));
    else                                  Serial.print(F("L2"));

    // Mission.ino CamFindBlockAndGetStablePosition()과 같은 x축 ROI 판정
    if (MIN_PIXY_SEARCH_X <= x && x <= MAX_PIXY_SEARCH_X)
      Serial.println(F(", in ROI"));
    else
      Serial.println(F(", OUT of ROI (Mission ignores)"));
  }
}
