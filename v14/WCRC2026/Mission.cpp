#include "Mission.h"
#include "Hardware.h"
#include "Vision.h"

static uint8_t gInv[4][3];
static uint8_t gScanPtr = 0;
static uint8_t gCol = 0;
static float gLateralMm = 0;
static uint32_t gStartMs = 0;
static uint8_t gOrder[MAX_ORDER];
static uint8_t gOrderCount = 0;
static bool gUsed[MAX_ORDER];

static const Pose& ScanPose(uint8_t layer)
{
  return POSE_SCAN[layer];
}

static uint8_t FrontPin()
{
  return gCol >= 2 ? PSD_FL : PSD_FR;
}

static int FrontTarget(uint8_t layer)
{
  return layer == 0 ? PSD_STORAGE_L1_FR_ADC : PSD_STORAGE_FR_ADC;
}

static void Shift(Dir plus, Dir minus, float mm, float mmps)
{
  if (mm > 0.5) DriveMm(plus, mm, mmps);
  else if (mm < -0.5) DriveMm(minus, -mm, mmps);
}

static bool TimeUp()
{
  return millis() - gStartMs > MATCH_LIMIT_MS - RETURN_RESERVE_MS;
}

static void PrintInventory()
{
  Serial.println(F("inventory (col1..4, L3/L2/L1): 0=?, 7=empty, 8=fail"));
  for (int8_t l = 2; l >= 0; l--) {
    Serial.print('L');
    Serial.print(l + 1);
    Serial.print(':');
    for (uint8_t c = 0; c < 4; c++) {
      Serial.print(' ');
      Serial.print(gInv[c][l]);
    }
    Serial.println();
  }
}

void ResetMission()
{
  memset(gInv, INV_UNKNOWN, sizeof(gInv));
  memset(gUsed, 0, sizeof(gUsed));
  gScanPtr = 0;
  gCol = 0;
  gLateralMm = 0;
  gOrderCount = 0;
  gStartMs = millis();
}

void SegStartToLine()
{
  ArmPose(POSE_TRAVEL);
  Grip(GRIP_OPEN);
  DriveMm(FWD, DIST_START_TO_LINE_MM, SPEED_NORMAL_MMPS);
  AlignPsd(PSD_FR, PSD_LINE_FR_ADC, PSD_ALIGN_TOL_ADC);
}

void SegLineToOrder()
{
  DriveMm(LEFT, DIST_LINE_TO_ORDER_MM, SPEED_NORMAL_MMPS);
  AlignPsd(PSD_SL, PSD_ORDER_SL_ADC, PSD_ALIGN_TOL_ADC);
  gOrderCount = 0;
  if (ORDER_READ_ENABLE && ArmPose(POSE_ORDER_VIEW))
    gOrderCount = ReadOrder(gOrder);
  if (gOrderCount == 0) {
    for (uint8_t i = 0; i < MISSION_COUNT; i++) gOrder[i] = MISSION[i].color;
    gOrderCount = MISSION_COUNT;
    Serial.println(F("order: table fallback"));
    Beep(3);
  }
  Serial.print(F("order:"));
  for (uint8_t i = 0; i < gOrderCount; i++) {
    Serial.print(' ');
    Serial.print(gOrder[i]);
  }
  Serial.println();
  ArmPose(POSE_TRAVEL);
}

static void AlignAnchor()
{
  AlignPsd(PSD_FR, PSD_STORAGE_FR_ADC, PSD_ALIGN_TOL_ADC);
  AlignPsd(PSD_SL, PSD_STORAGE_COL1_SL_ADC, PSD_ALIGN_TOL_ADC);
  gCol = 0;
  gLateralMm = 0;
}

void SegOrderToStorage()
{
  ArmPose(POSE_HUB);
  DriveMm(FWD, DIST_ORDER_TO_STORAGE_MM, SPEED_FAST_MMPS);
  ApproachFront(PSD_FR, PSD_STORAGE_DETECT_ADC, STORAGE_APPROACH_MAX_MM, SPEED_SLOW_MMPS);
  AlignAnchor();
}

static void GoColumn(uint8_t c)
{
  float now = gCol * STORAGE_COL_PITCH_MM + gLateralMm;
  Shift(RIGHT, LEFT, c * STORAGE_COL_PITCH_MM - now, SPEED_NORMAL_MMPS);
  gCol = c;
  gLateralMm = 0;
}

static void GoAnchor()
{
  ArmPose(POSE_TRAVEL);
  AlignPsd(FrontPin(), PSD_STORAGE_FR_ADC, PSD_ALIGN_TOL_ADC);
  GoColumn(0);
  AlignAnchor();
}

static void ScanColumn(uint8_t c)
{
  GoColumn(c);
  ArmPose(POSE_HUB);
  for (int8_t l = 2; l >= 0; l--) {
    if (ArmPose(ScanPose(l))) {
      WaitMs(SCAN_SETTLE_MS);
      ScanView(l, gInv[c]);
    }
  }
  ArmPose(POSE_HUB);
  PrintInventory();
}

void ScanAllColumns()
{
  ResetMission();
  AlignAnchor();
  for (uint8_t c = 0; c < 4; c++) ScanColumn(c);
  GoAnchor();
}

static bool FindInInventory(uint8_t sig, uint8_t& col, uint8_t& layer)
{
  for (uint8_t c = 0; c < 4; c++)
    for (int8_t l = 2; l >= 0; l--)
      if (gInv[c][l] == sig) {
        col = c;
        layer = l;
        return true;
      }
  return false;
}

static bool GripSequence(uint8_t layer, uint8_t sig)
{
  for (uint8_t attempt = 0; attempt <= PICK_RETRY; attempt++) {
    Grip(GRIP_OPEN);
    WaitMs(GRIP_WAIT_MS);
    ArmPose(POSE_HUB);
    ArmPose(POSE_PICK_GRIP[layer], true);
    AlignPsd(FrontPin(), PSD_GRIP_FR_ADC, PSD_ALIGN_TOL_ADC);
    WaitMs(PICK_SETTLE_MS);
    Grip(GRIP_CLOSE);
    WaitMs(GRIP_WAIT_MS);
    AlignPsd(FrontPin(), FrontTarget(layer), PSD_ALIGN_TOL_ADC);
    ArmPose(POSE_HUB);
    if (!PICK_VERIFY_ENABLE) return true;
    ArmPose(ScanPose(layer));
    WaitMs(SCAN_SETTLE_MS);
    if (!CellHas(sig, layer)) return true;
    Serial.println(F("pick failed, retry"));
  }
  Grip(GRIP_OPEN);
  return false;
}

uint8_t PickHere(uint8_t layer, uint8_t sig)
{
  AlignPsd(FrontPin(), FrontTarget(layer), PSD_ALIGN_TOL_ADC);
  ArmPose(POSE_HUB);
  if (!ArmPose(ScanPose(layer))) return PICK_FAIL;
  WaitMs(SCAN_SETTLE_MS);
  if (!CellHas(sig, layer)) {
    Serial.println(F("verify failed"));
    return PICK_NOT_SEEN;
  }
  float moved = 0;
  bool centered = CenterOnBlock(sig, layer, moved);
  gLateralMm += moved;
  if (!centered) return PICK_FAIL;
  AlignPsd(FrontPin(), FrontTarget(layer), PSD_ALIGN_TOL_ADC);
  return GripSequence(layer, sig) ? PICK_OK : PICK_FAIL;
}

static bool PickAt(uint8_t col, uint8_t layer, uint8_t sig)
{
  GoColumn(col);
  uint8_t result = PickHere(layer, sig);
  if (result == PICK_NOT_SEEN) {
    ScanView(layer, gInv[col]);
    if (gInv[col][layer] == sig) gInv[col][layer] = INV_FAILED;
  } else {
    gInv[col][layer] = result == PICK_OK ? INV_EMPTY : INV_FAILED;
  }
  PrintInventory();
  ArmPose(POSE_HUB);
  ArmPose(POSE_TRAVEL);
  return result == PICK_OK;
}

void DropHere(char zone, uint8_t cell)
{
  if (cell < 1 || cell > 8) return;
  const Pose& p = zone == 'A' ? POSE_ZONE_A[cell - 1] : POSE_ZONE_B[cell - 1];
  ArmPose(p);
  Grip(GRIP_RELEASE);
  WaitMs(GRIP_WAIT_MS);
  ArmPose(POSE_TRAVEL);
}

void Deliver(char zone, uint8_t cell)
{
  if (cell < 1 || cell > 8) return;
  ArmPose(POSE_TRAVEL);
  if (zone == 'A') {
    float shift = ZONE_A_CELL_SHIFT_MM[cell - 1];
    DriveMm(LEFT, ZONE_A_LEFT_MM, SPEED_NORMAL_MMPS);
    DriveMm(FWD, ZONE_A_FWD_MM, SPEED_NORMAL_MMPS);
    AlignPsd(PSD_SL, PSD_ZONE_A_SL_ADC, PSD_ALIGN_TOL_ADC);
    Shift(FWD, BWD, shift, SPEED_SLOW_MMPS);
    DropHere('A', cell);
    Shift(BWD, FWD, shift, SPEED_SLOW_MMPS);
    DriveMm(BWD, ZONE_A_FWD_MM, SPEED_NORMAL_MMPS);
    DriveMm(RIGHT, ZONE_A_RETURN_RIGHT_MM, SPEED_NORMAL_MMPS);
  } else {
    float shift = ZONE_B_CELL_SHIFT_MM[cell - 1];
    DriveMm(RIGHT, ZONE_B_LANE_RIGHT_MM, SPEED_NORMAL_MMPS);
    DriveMm(BWD, ZONE_B_BACK_MM, SPEED_FAST_MMPS);
    AlignPsd(PSD_SR, PSD_ZONE_B_SR_ADC, PSD_ALIGN_TOL_ADC);
    Shift(RIGHT, LEFT, shift, SPEED_SLOW_MMPS);
    DropHere('B', cell);
    Shift(LEFT, RIGHT, shift, SPEED_SLOW_MMPS);
    DriveMm(FWD, ZONE_B_BACK_MM, SPEED_FAST_MMPS);
    DriveMm(LEFT, ZONE_B_LANE_RIGHT_MM, SPEED_NORMAL_MMPS);
  }
  ArmPose(POSE_HUB);
  ApproachFront(PSD_FR, PSD_STORAGE_DETECT_ADC, STORAGE_APPROACH_MAX_MM, SPEED_SLOW_MMPS);
  AlignAnchor();
}

void SegFinish()
{
  ArmPose(POSE_TRAVEL);
  DriveMm(RIGHT, FINISH_LANE_RIGHT_MM, SPEED_NORMAL_MMPS);
  FollowWallBack(FINISH_BACK_MM, FINISH_SPEED_MMPS, PSD_FINISH_SL_ADC);
  DriveMm(LEFT, FINISH_LEFT_MM, SPEED_NORMAL_MMPS);
}

static bool TakeTask(uint8_t sig, Task& task)
{
  for (uint8_t i = 0; i < MISSION_COUNT; i++)
    if (!gUsed[i] && MISSION[i].color == sig) {
      gUsed[i] = true;
      task = MISSION[i];
      return true;
    }
  return false;
}

static void ProcessTarget(uint8_t sig)
{
  Task task;
  if (!TakeTask(sig, task)) {
    Serial.println(F("no task for color"));
    return;
  }
  for (uint8_t tries = 0; tries < 2; tries++) {
    uint8_t col, layer;
    bool found = FindInInventory(sig, col, layer);
    while (!found && gScanPtr < 4 && !TimeUp()) {
      ScanColumn(gScanPtr++);
      found = FindInInventory(sig, col, layer);
    }
    if (!found) {
      Serial.println(F("block not found"));
      Beep(2);
      break;
    }
    if (PickAt(col, layer, sig)) {
      GoAnchor();
      Deliver(task.zone, task.cell);
      return;
    }
  }
  GoAnchor();
}

void RunMission()
{
  ResetMission();
  Led(0, 0, 255);
  SegStartToLine();
  SegLineToOrder();
  SegOrderToStorage();
  for (uint8_t i = 0; i < gOrderCount; i++) {
    if (TimeUp()) {
      Serial.println(F("time up"));
      break;
    }
    Serial.print(F("target "));
    Serial.print(i + 1);
    Serial.print(F(" color "));
    Serial.println(gOrder[i]);
    ProcessTarget(gOrder[i]);
  }
  SegFinish();
  Serial.print(F("done ms: "));
  Serial.println(millis() - gStartMs);
  Led(255, 255, 0);
  Beep(2);
}
