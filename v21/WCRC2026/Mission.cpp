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
static uint8_t gUserOrder[MAX_ORDER];
static uint8_t gUserOrderCount = 0;
static uint8_t gZoneColor[2][8];
static bool gZoneUsed[2][8];
static uint8_t gNextIdx = 0;
static bool gCurValid = false;
static char gCurZone = 'A';
static uint8_t gCurCell = 1;
static bool gHolding = false;
static uint8_t gStage = 0;
static uint8_t gKind = 0;
static bool gActive = false;
static uint32_t gPausedAt = 0;

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
  Con.println(F("inventory (col1..4, L3/L2/L1): 0=?, 7=empty, 8=fail"));
  for (int8_t l = 2; l >= 0; l--) {
    Con.print('L');
    Con.print(l + 1);
    Con.print(':');
    for (uint8_t c = 0; c < 4; c++) {
      Con.print(' ');
      Con.print(gInv[c][l]);
    }
    Con.println();
  }
}

void ResetMission()
{
  memset(gInv, INV_UNKNOWN, sizeof(gInv));
  memset(gZoneUsed, 0, sizeof(gZoneUsed));
  gScanPtr = 0;
  gCol = 0;
  gLateralMm = 0;
  gOrderCount = 0;
  gNextIdx = 0;
  gCurValid = false;
  gHolding = false;
  gStage = 0;
  gActive = false;
  gStartMs = millis();
}

static void PrintOrder(const uint8_t* o, uint8_t n)
{
  for (uint8_t i = 0; i < n; i++) {
    Con.print(' ');
    Con.print(o[i]);
  }
  Con.println();
}

void SetUserOrder(const uint8_t* colors, uint8_t n)
{
  if (n > MAX_ORDER) n = MAX_ORDER;
  memcpy(gUserOrder, colors, n);
  gUserOrderCount = n;
}

bool SetZone(char zone, uint8_t cell, uint8_t color)
{
  if ((zone != 'A' && zone != 'B') || cell < 1 || cell > 8 || color > 6) return false;
  gZoneColor[zone == 'A' ? 0 : 1][cell - 1] = color;
  return true;
}

void ClearPlan()
{
  gUserOrderCount = 0;
  memset(gZoneColor, 0, sizeof(gZoneColor));
}

void PrintPlan()
{
  Con.print(F("order:"));
  PrintOrder(gUserOrder, gUserOrderCount);
  for (uint8_t z = 0; z < 2; z++) {
    Con.print(z == 0 ? F("zone A:") : F("zone B:"));
    for (uint8_t c = 0; c < 8; c++) {
      Con.print(' ');
      Con.print(gZoneColor[z][c]);
    }
    Con.println();
  }
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
  DriveMm(BWD, DIST_LINE_TO_ORDER_BACK_MM, SPEED_NORMAL_MMPS);
  AlignPsd(PSD_SL, PSD_ORDER_SL_ADC, PSD_ALIGN_TOL_ADC);
  gOrderCount = 0;
  if (ORDER_READ_ENABLE && ArmPose(POSE_TRAVEL))
    gOrderCount = ReadOrder(gOrder);
  if (gOrderCount > 0) {
    Beep(1);
  } else {
    memcpy(gOrder, gUserOrder, gUserOrderCount);
    gOrderCount = gUserOrderCount;
    Con.println(F("order: camera failed, using input order"));
    Beep(3);
  }
  Con.print(F("order:"));
  PrintOrder(gOrder, gOrderCount);
  ArmPose(POSE_ORDER_VIEW);
}

static void AlignAnchor()
{
  AlignPsd(PSD_FR, PSD_STORAGE_FR_ADC, PSD_ALIGN_TOL_ADC);
  AlignPsd(PSD_SL, PSD_STORAGE_COL1_SL_ADC, PSD_ALIGN_TOL_ADC);
  gCol = 0;
  gLateralMm = 0;
}

static void GoColumn(uint8_t c)
{
  float now = gCol * STORAGE_COL_PITCH_MM + gLateralMm;
  Shift(RIGHT, LEFT, c * STORAGE_COL_PITCH_MM - now, SPEED_NORMAL_MMPS);
  gCol = c;
  gLateralMm = 0;
}

static void AnchorWithHeading()
{
  AlignPsd(FrontPin(), PSD_STORAGE_FR_ADC, PSD_ALIGN_TOL_ADC);
  if (HEADING_ENABLE) {
    GoColumn(1);
    AlignHeading();
  }
  GoColumn(0);
  AlignAnchor();
}

void SegOrderToStorage()
{
  ArmPose(POSE_ORDER_VIEW);
  DriveMm(FWD, DIST_ORDER_TO_STORAGE_MM, SPEED_FAST_MMPS);
  DriveMm(RIGHT, ORDER_TO_COL1_RIGHT_MM, SPEED_NORMAL_MMPS);
  ApproachFront(PSD_FR, PSD_STORAGE_DETECT_ADC, STORAGE_APPROACH_MAX_MM, SPEED_SLOW_MMPS);
  gCol = 0;
  gLateralMm = 0;
  AnchorWithHeading();
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
  if (c == 1) AlignHeading();
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
    Shift(FWD, BWD, PICK_FWD_MM, SPEED_SLOW_MMPS);
    WaitMs(PICK_SETTLE_MS);
    Grip(GRIP_CLOSE);
    WaitMs(GRIP_WAIT_MS);
    ArmPose(POSE_HUB);
    AlignPsd(FrontPin(), FrontTarget(layer), PSD_ALIGN_TOL_ADC);
    if (!PICK_VERIFY_ENABLE) return true;
    ArmPose(ScanPose(layer));
    WaitMs(SCAN_SETTLE_MS);
    if (!CellHas(sig, layer)) return true;
    Con.println(F("pick failed, retry"));
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
    Con.println(F("verify failed"));
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
    Shift(FWD, BWD, ZONE_A_FWD_MM, SPEED_NORMAL_MMPS);
    AlignPsd(PSD_SL, PSD_ZONE_A_SL_ADC, PSD_ALIGN_TOL_ADC);
    Shift(FWD, BWD, shift, SPEED_SLOW_MMPS);
    DropHere('A', cell);
    Shift(BWD, FWD, shift, SPEED_SLOW_MMPS);
    Shift(BWD, FWD, ZONE_A_FWD_MM, SPEED_NORMAL_MMPS);
    DriveMm(RIGHT, ZONE_A_RETURN_RIGHT_MM, SPEED_NORMAL_MMPS);
  } else {
    float shift = ZONE_B_CELL_SHIFT_MM[cell - 1];
    GoColumn(3);
    AlignPsd(FrontPin(), PSD_STORAGE_FR_ADC, PSD_ALIGN_TOL_ADC);
    DriveMm(RIGHT, ZONE_B_LANE_RIGHT_MM, SPEED_NORMAL_MMPS);
    DriveMm(BWD, ZONE_B_BACK_MM, SPEED_FAST_MMPS);
    AlignPsd(PSD_SR, PSD_ZONE_B_SR_ADC, PSD_ALIGN_TOL_ADC);
    Shift(RIGHT, LEFT, shift, SPEED_SLOW_MMPS);
    DropHere('B', cell);
    Shift(LEFT, RIGHT, shift, SPEED_SLOW_MMPS);
    DriveMm(FWD, ZONE_B_BACK_MM, SPEED_FAST_MMPS);
    DriveMm(LEFT, ZONE_B_LANE_RIGHT_MM, SPEED_NORMAL_MMPS);
    GoColumn(0);
  }
  ArmPose(POSE_HUB);
  ApproachFront(PSD_FR, PSD_STORAGE_DETECT_ADC, STORAGE_APPROACH_MAX_MM, SPEED_SLOW_MMPS);
  gCol = 0;
  gLateralMm = 0;
  AnchorWithHeading();
}

void SegFinish()
{
  ArmPose(POSE_TRAVEL);
  DriveMm(RIGHT, FINISH_LANE_RIGHT_MM, SPEED_NORMAL_MMPS);
  FollowWallBack(FINISH_BACK_MM, FINISH_SPEED_MMPS, PSD_FINISH_SL_ADC);
  DriveMm(LEFT, FINISH_LEFT_MM, SPEED_NORMAL_MMPS);
}

static bool TakeSlot(uint8_t sig)
{
  for (uint8_t z = 0; z < 2; z++)
    for (uint8_t c = 0; c < 8; c++)
      if (gZoneColor[z][c] == sig && !gZoneUsed[z][c]) {
        gZoneUsed[z][c] = true;
        gCurZone = z == 0 ? 'A' : 'B';
        gCurCell = c + 1;
        return true;
      }
  return false;
}

static void ReleaseSlot()
{
  gZoneUsed[gCurZone == 'A' ? 0 : 1][gCurCell - 1] = false;
}

static bool ProcessTarget(uint8_t sig)
{
  if (!gCurValid) {
    if (!TakeSlot(sig)) {
      Con.println(F("no zone for color"));
      Beep(2);
      return false;
    }
    gCurValid = true;
  }
  if (!gHolding) {
    bool picked = false;
    for (uint8_t tries = 0; tries < 2 && !picked; tries++) {
      uint8_t col, layer;
      bool found = FindInInventory(sig, col, layer);
      while (!found && gScanPtr < 4 && !TimeUp()) {
        ScanColumn(gScanPtr++);
        found = FindInInventory(sig, col, layer);
      }
      if (!found) {
        Con.println(F("block not found"));
        break;
      }
      picked = PickAt(col, layer, sig);
    }
    if (!picked) {
      Beep(2);
      ReleaseSlot();
      gCurValid = false;
      GoAnchor();
      return false;
    }
    gHolding = true;
  }
  Con.print(F("deliver "));
  Con.print(gCurZone);
  Con.println(gCurCell);
  if (gCurZone == 'A') GoAnchor();
  Deliver(gCurZone, gCurCell);
  gHolding = false;
  gCurValid = false;
  return true;
}

static void StorageLoop()
{
  for (; gNextIdx < gOrderCount; gNextIdx++) {
    if (TimeUp()) {
      Con.println(F("time up"));
      BeepLong();
      break;
    }
    Con.print(F("target "));
    Con.print(gNextIdx + 1);
    Con.print(F(" color "));
    Con.println(gOrder[gNextIdx]);
    ProcessTarget(gOrder[gNextIdx]);
  }
}

static void RunFrom(uint8_t stage, bool resumed)
{
  gActive = true;
  Led(0, 0, 255);
  if (stage <= 1) { gStage = 1; SegStartToLine(); }
  if (stage <= 2) { gStage = 2; SegLineToOrder(); }
  if (stage <= 3) { gStage = 3; SegOrderToStorage(); }
  if (stage <= 4) {
    gStage = 4;
    if (resumed || gKind == 6) {
      ArmPose(POSE_TRAVEL);
      gCol = 0;
      gLateralMm = 0;
      AnchorWithHeading();
    }
    StorageLoop();
  }
  if (gKind == 7 && stage <= 5) { gStage = 5; SegFinish(); }
  gActive = false;
  Con.print(F("done ms: "));
  Con.println(millis() - gStartMs);
  Led(255, 255, 0);
  Beep(2);
}

void RunMission()
{
  ResetMission();
  gKind = 7;
  RunFrom(1, false);
}

void RunStorage()
{
  if (gUserOrderCount == 0) {
    Con.println(F("order empty: order 5 1 6 3"));
    return;
  }
  ResetMission();
  memcpy(gOrder, gUserOrder, gUserOrderCount);
  gOrderCount = gUserOrderCount;
  gKind = 6;
  RunFrom(4, false);
}

void MissionPaused()
{
  gPausedAt = millis();
}

bool MissionResumable()
{
  return gActive;
}

void ResumeMission()
{
  if (!gActive) return;
  gStartMs += millis() - gPausedAt;
  Con.print(F("resume stage "));
  Con.println(gStage);
  uint8_t s = gStage;
  if (s == 1) Con.println(F("robot must be at start"));
  else if (s == 2) Con.println(F("robot must be on the line"));
  else if (s == 3) Con.println(F("robot must be at order zone"));
  else Con.println(F("robot must be at col1 front"));
  RunFrom(s, true);
}
