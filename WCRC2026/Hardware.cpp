#include "Hardware.h"
#include <Adafruit_NeoPixel.h>

using namespace ControlTableItem;

Dynamixel2Arduino dxl(Serial1, 2);
Pixy2SPI_SS pixy;
jmp_buf gAbortJump;
bool gAbortArmed = false;

static Adafruit_NeoPixel rgb(1, PIN_RGB, NEO_GRBW + NEO_KHZ800);

static const uint8_t WHEEL_IDS[4] = { 1, 2, 3, 4 };
static const uint8_t JOINT_IDS[4] = { 5, 6, 7, 8 };
static const int32_t JOINT_HW_MIN[4] = { 0, 1024, 211, 722 };
static const int32_t JOINT_HW_MAX[4] = { 4095, 3474, 2640, 3161 };
static const int32_t JOINT_HOMING[4] = { 0, 238, 799, 0 };
static const float WHEEL_RADIUS_MM = 30.0;
static const float VEL_UNITS_PER_MMPS = 60.0 / (2.0 * PI * WHEEL_RADIUS_MM) / 0.229;
static const uint8_t MODE_VELOCITY = 1;
static const uint8_t MODE_POSITION = 4;

struct __attribute__((packed)) PosData { int32_t profile; int32_t goal; };
struct __attribute__((packed)) VelData { int32_t goal; };

static PosData wheelPos[4];
static VelData wheelVel[4];
static PosData jointPos[4];
static DYNAMIXEL::InfoSyncWriteInst_t swWheelPos, swWheelVel, swJoint;
static DYNAMIXEL::XELInfoSyncWrite_t xWheelPos[4], xWheelVel[4], xJoint[4];
static uint8_t baseMode = 0;

static void SetupSync(DYNAMIXEL::InfoSyncWriteInst_t& s, DYNAMIXEL::XELInfoSyncWrite_t* x,
                      const uint8_t* ids, uint8_t* data, uint16_t addr, uint16_t len)
{
  s.packet.p_buf = nullptr;
  s.packet.is_completed = false;
  s.addr = addr;
  s.addr_length = len;
  s.p_xels = x;
  s.xel_count = 0;
  for (uint8_t i = 0; i < 4; i++) {
    x[i].id = ids[i];
    x[i].p_data = data + i * len;
    s.xel_count++;
  }
  s.is_info_changed = true;
}

static bool SendSync(DYNAMIXEL::InfoSyncWriteInst_t& s)
{
  s.is_info_changed = true;
  for (uint8_t i = 0; i < 10; i++)
    if (dxl.syncWrite(&s)) return true;
  return false;
}

static void SetBaseMode(uint8_t mode)
{
  if (baseMode == mode) return;
  uint8_t profile = mode == MODE_VELOCITY ? 0 : 4;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t id = WHEEL_IDS[i];
    dxl.writeControlTableItem(TORQUE_ENABLE, id, 0);
    dxl.writeControlTableItem(DRIVE_MODE, id, ((id == 2 || id == 4) ? 1 : 0) + profile);
    dxl.writeControlTableItem(OPERATING_MODE, id, mode);
    dxl.writeControlTableItem(TORQUE_ENABLE, id, 1);
  }
  baseMode = mode;
}

void InitHardware()
{
  pinMode(PIN_SW1, INPUT_PULLUP);
  pinMode(PIN_SW2, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  rgb.begin();
  rgb.setBrightness(50);
  Led(0, 0, 255);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
  SetupSync(swWheelPos, xWheelPos, WHEEL_IDS, (uint8_t*)wheelPos, 112, 8);
  SetupSync(swWheelVel, xWheelVel, WHEEL_IDS, (uint8_t*)wheelVel, 104, 4);
  SetupSync(swJoint, xJoint, JOINT_IDS, (uint8_t*)jointPos, 112, 8);

  for (uint8_t i = 0; i < 4; i++) {
    uint8_t id = JOINT_IDS[i];
    dxl.writeControlTableItem(TORQUE_ENABLE, id, 0);
    dxl.writeControlTableItem(DRIVE_MODE, id, 1 + 4);
    dxl.writeControlTableItem(OPERATING_MODE, id, 3);
    dxl.writeControlTableItem(MIN_POSITION_LIMIT, id, JOINT_HW_MIN[i]);
    dxl.writeControlTableItem(MAX_POSITION_LIMIT, id, JOINT_HW_MAX[i]);
    dxl.writeControlTableItem(HOMING_OFFSET, id, JOINT_HOMING[i]);
    dxl.writeControlTableItem(TORQUE_ENABLE, id, 1);
  }

  baseMode = 0;
  SetBaseMode(MODE_VELOCITY);
  BaseStop();
  pixy.init();
  pixy.setLamp(0, 0);
}

bool SelfCheck()
{
  bool ok = true;
  for (uint8_t id = 1; id <= 8; id++) {
    if (!dxl.ping(id)) {
      Serial.print(F("motor missing: "));
      Serial.println(id);
      ok = false;
    }
  }
  float volt = dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, 1) / 10.0;
  Serial.print(F("battery V: "));
  Serial.println(volt);
  if (volt < LOW_VOLTAGE_V) ok = false;
  if (pixy.getVersion() < 0) {
    Serial.println(F("pixy missing"));
    ok = false;
  }
  Serial.print(F("PSD FL SL FR SR: "));
  Serial.print(PsdRead(PSD_FL)); Serial.print(' ');
  Serial.print(PsdRead(PSD_SL)); Serial.print(' ');
  Serial.print(PsdRead(PSD_FR)); Serial.print(' ');
  Serial.println(PsdRead(PSD_SR));
  Serial.println(ok ? F("SELF CHECK OK") : F("SELF CHECK FAIL"));
  if (ok) Led(0, 255, 0); else Led(255, 0, 0);
  Beep(ok ? 1 : 3);
  return ok;
}

void AbortCheck()
{
  if (!gAbortArmed) return;
  if (digitalRead(PIN_SW2) == LOW || Serial.available()) {
    gAbortArmed = false;
    BaseStop();
    while (Serial.available()) Serial.read();
    longjmp(gAbortJump, 1);
  }
}

void WaitMs(uint32_t ms)
{
  uint32_t start = millis();
  while (millis() - start < ms) {
    AbortCheck();
    delay(5);
  }
}

void Led(uint8_t r, uint8_t g, uint8_t b)
{
  rgb.setPixelColor(0, rgb.Color(r, g, b));
  rgb.show();
}

void Beep(uint8_t count)
{
  for (uint8_t i = 0; i < count; i++) {
    tone(PIN_BUZZER, 392);
    delay(60);
    noTone(PIN_BUZZER);
    delay(60);
  }
}

int PsdRead(uint8_t pin)
{
  int v[PSD_SAMPLES];
  for (uint8_t i = 0; i < PSD_SAMPLES; i++) {
    v[i] = analogRead(pin);
    delay(1);
  }
  for (uint8_t i = 1; i < PSD_SAMPLES; i++)
    for (uint8_t j = i; j > 0 && v[j - 1] > v[j]; j--) {
      int t = v[j]; v[j] = v[j - 1]; v[j - 1] = t;
    }
  return v[PSD_SAMPLES / 2];
}

void BaseVel(float fwdMmps, float leftMmps)
{
  SetBaseMode(MODE_VELOCITY);
  wheelVel[0].goal = (fwdMmps - leftMmps) * VEL_UNITS_PER_MMPS;
  wheelVel[1].goal = (fwdMmps + leftMmps) * VEL_UNITS_PER_MMPS;
  wheelVel[2].goal = (fwdMmps + leftMmps) * VEL_UNITS_PER_MMPS;
  wheelVel[3].goal = (fwdMmps - leftMmps) * VEL_UNITS_PER_MMPS;
  SendSync(swWheelVel);
}

void BaseStop()
{
  BaseVel(0, 0);
}

static bool BaseArrived()
{
  for (uint8_t i = 0; i < 4; i++)
    if (!(dxl.readControlTableItem(MOVING_STATUS, WHEEL_IDS[i]) & 0x01)) return false;
  return true;
}

void DriveMm(Dir dir, float mm, float mmps)
{
  if (mm <= 0 || mmps <= 0) return;
  static const int8_t SIGN[4][4] = { { 1, 1, 1, 1 }, { -1, -1, -1, -1 }, { -1, 1, 1, -1 }, { 1, -1, -1, 1 } };
  const float* ratio = dir == FWD ? WHEEL_RATIO_FWD : dir == BWD ? WHEEL_RATIO_BWD
                     : dir == LEFT ? WHEEL_RATIO_LEFT : WHEEL_RATIO_RIGHT;
  int32_t ticks = mm / WHEEL_RADIUS_MM * 4095.0 / (2.0 * PI);
  uint32_t ms = mm / mmps * 1000.0;

  SetBaseMode(MODE_POSITION);
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t id = WHEEL_IDS[i];
    dxl.writeControlTableItem(PROFILE_ACCELERATION, id, 100);
    int32_t now = 0;
    bool read = false;
    for (uint8_t k = 0; k < 5 && !read; k++) {
      now = dxl.readControlTableItem(PRESENT_POSITION, id);
      read = dxl.getLastLibErrCode() == DXL_LIB_OK;
    }
    if (!read) {
      Serial.println(F("move skipped: read fail"));
      Beep(2);
      return;
    }
    wheelPos[i].profile = ms;
    wheelPos[i].goal = now + (int32_t)(SIGN[dir][i] * ticks * ratio[i]);
  }
  if (!SendSync(swWheelPos)) {
    Serial.println(F("move skipped: write fail"));
    Beep(2);
    return;
  }
  uint32_t start = millis();
  uint32_t limit = ms + 1000 + ms / 4;
  WaitMs(20);
  while (!BaseArrived()) {
    AbortCheck();
    if (millis() - start > limit) {
      Serial.println(F("move timeout"));
      break;
    }
  }
}

bool AlignPsd(uint8_t pin, int targetAdc, int tolAdc)
{
  if (targetAdc <= 0) return true;
  uint32_t start = millis();
  uint8_t stable = 0;
  while (millis() - start < PSD_ALIGN_TIMEOUT_MS) {
    AbortCheck();
    int err = targetAdc - PsdRead(pin);
    if (abs(err) <= tolAdc) {
      BaseStop();
      if (++stable >= 3) return true;
      continue;
    }
    stable = 0;
    float v = constrain(abs(err) * PSD_ALIGN_GAIN, 0.0, 1.0) * PSD_ALIGN_MAX_MMPS;
    if (v < PSD_ALIGN_MIN_MMPS) v = PSD_ALIGN_MIN_MMPS;
    if (err < 0) v = -v;
    if (pin == PSD_FL || pin == PSD_FR) BaseVel(v, 0);
    else if (pin == PSD_SL) BaseVel(0, v);
    else BaseVel(0, -v);
    delay(10);
  }
  BaseStop();
  Serial.print(F("align timeout pin "));
  Serial.println(pin);
  return false;
}

bool ApproachFront(uint8_t pin, int detectAdc, float maxMm, float mmps)
{
  uint32_t limit = maxMm / mmps * 1000.0;
  uint32_t start = millis();
  BaseVel(mmps, 0);
  while (millis() - start < limit) {
    AbortCheck();
    if (PsdRead(pin) >= detectAdc) {
      BaseStop();
      return true;
    }
  }
  BaseStop();
  return false;
}

void FollowWallBack(float mm, float mmps, int slTargetAdc)
{
  uint32_t limit = mm / mmps * 1000.0;
  uint32_t start = millis();
  while (millis() - start < limit) {
    AbortCheck();
    int err = slTargetAdc - PsdRead(PSD_SL);
    float lat = 0;
    if (abs(err) <= FINISH_TRACK_BAND_ADC)
      lat = constrain(err * FINISH_TRACK_GAIN, -FINISH_TRACK_MAX_MMPS, FINISH_TRACK_MAX_MMPS);
    BaseVel(-mmps, lat);
  }
  BaseStop();
}

bool PoseValid(const Pose& p)
{
  return !(p.j1 == 0 && p.j2 == 0 && p.j3 == 0 && p.j4 == 0);
}

bool ArmMove(const int16_t* joints, uint16_t ms)
{
  for (uint8_t i = 0; i < 4; i++) {
    if (joints[i] < JOINT_MIN[i] || joints[i] > JOINT_MAX[i]) {
      Serial.print(F("joint limit: j"));
      Serial.println(i + 1);
      return false;
    }
  }
  for (uint8_t i = 0; i < 4; i++) {
    jointPos[i].profile = ms;
    jointPos[i].goal = joints[i];
  }
  SendSync(swJoint);
  WaitMs(ms + POSE_SETTLE_MS);
  return true;
}

bool ArmPose(const Pose& p, uint16_t ms)
{
  if (!PoseValid(p)) {
    Serial.println(F("pose not set"));
    return false;
  }
  int16_t j[4] = { p.j1, p.j2, p.j3, p.j4 };
  return ArmMove(j, ms);
}

void ArmRead(int16_t* joints)
{
  for (uint8_t i = 0; i < 4; i++)
    joints[i] = dxl.readControlTableItem(PRESENT_POSITION, JOINT_IDS[i]);
}

void Grip(int16_t value)
{
  pixy.setServos(0, constrain(value, 0, 1000));
}
