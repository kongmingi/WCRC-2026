#include "Console.h"
#include "Hardware.h"
#include "Mission.h"
#include "Vision.h"

static char line[48];

static void ArmTorque(bool on)
{
  for (uint8_t id = 5; id <= 8; id++)
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, id, on ? 1 : 0);
}

static void Arm()
{
  delay(50);
  while (Serial.available()) Serial.read();
  gAbortArmed = true;
}

static void WaitKey()
{
  Serial.println(F("press enter"));
  while (!Serial.available()) delay(10);
  while (Serial.available()) { Serial.read(); delay(2); }
}

static const Pose* PoseByName(const char* n)
{
  if (!strcmp(n, "travel")) return &POSE_TRAVEL;
  if (!strcmp(n, "order")) return &POSE_ORDER_VIEW;
  if (!strcmp(n, "hub")) return &POSE_HUB;
  if (strlen(n) < 3) return nullptr;
  uint8_t k = n[strlen(n) - 1] - '1';
  if (!strncmp(n, "scan", 4) && strlen(n) == 5 && k < 3) return &POSE_SCAN[k];
  if (strlen(n) != 3) return nullptr;
  if (n[0] == 'p' && k < 3) {
    if (n[1] == 'a') return &POSE_PICK_APPROACH[k];
    if (n[1] == 'g') return &POSE_PICK_GRIP[k];
    if (n[1] == 'o') return &POSE_PICK_OUT[k];
  }
  if (n[0] == 'z' && k < 8) {
    if (n[1] == 'a') return &POSE_ZONE_A[k];
    if (n[1] == 'b') return &POSE_ZONE_B[k];
  }
  return nullptr;
}

static Dir DirByChar(char c)
{
  if (c == 'B' || c == 'b') return BWD;
  if (c == 'L' || c == 'l') return LEFT;
  if (c == 'R' || c == 'r') return RIGHT;
  return FWD;
}

void ConsolePrintHelp()
{
  Serial.println(F("grip 650 | j3 +20 | j3 1500 | arm a b c d | pose | p travel | free | lock"));
  Serial.println(F("move F 300 (F/B/L/R) | psd | check"));
  Serial.println(F("t1 sensor | t2 move | t3 2 4 (layer color) | t3 A 5 (drop)"));
  Serial.println(F("t4 scan | t5 1~6 segment | t6 full mission"));
  Serial.println(F("pose names: travel order hub scan1-3 pa1-3 pg1-3 po1-3 za1-8 zb1-8"));
  Serial.println(F("stop: any key or SW2"));
}

static void PrintPose()
{
  int16_t j[4];
  ArmRead(j);
  Serial.print(F("{ "));
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(j[i]);
    Serial.print(i < 3 ? F(", ") : F(" }"));
  }
  Serial.println();
}

static void SensorMonitor()
{
  while (true) {
    AbortCheck();
    Serial.print(F("PSD FL SL FR SR: "));
    Serial.print(PsdRead(PSD_FL)); Serial.print(' ');
    Serial.print(PsdRead(PSD_SL)); Serial.print(' ');
    Serial.print(PsdRead(PSD_FR)); Serial.print(' ');
    Serial.println(PsdRead(PSD_SR));
    if (pixy.ccc.getBlocks(true, PIXY_SIGMAP) >= 0) {
      for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++) {
        Block& b = pixy.ccc.blocks[i];
        Serial.print(F("  sig ")); Serial.print(b.m_signature);
        Serial.print(F(" x ")); Serial.print(b.m_x);
        Serial.print(F(" y ")); Serial.print(b.m_y);
        Serial.print(F(" w ")); Serial.print(b.m_width);
        Serial.print(F(" h ")); Serial.println(b.m_height);
      }
    }
    WaitMs(500);
  }
}

static void MoveTest()
{
  gAbortArmed = false;
  const char names[4] = { 'F', 'B', 'L', 'R' };
  const float mm[4] = { 500, 500, 300, 300 };
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(names[i]);
    Serial.print(' ');
    Serial.print(mm[i]);
    Serial.println(F(" mm"));
    WaitKey();
    Arm();
    DriveMm(DirByChar(names[i]), mm[i], SPEED_NORMAL_MMPS);
    gAbortArmed = false;
    Serial.println(F("measure real distance"));
  }
}

static void SegmentTest(uint8_t n)
{
  if (n == 1) SegStartToLine();
  else if (n == 2) SegLineToOrder();
  else if (n == 3) SegOrderToStorage();
  else if (n == 4) Deliver('A', 1);
  else if (n == 5) Deliver('B', 1);
  else if (n == 6) SegFinish();
}

void ConsoleRunLine()
{
  uint8_t len = Serial.readBytesUntil('\n', line, sizeof(line) - 1);
  line[len] = 0;
  char* cmd = strtok(line, " \r");
  if (!cmd) return;
  char* a1 = strtok(nullptr, " \r");
  char* a2 = strtok(nullptr, " \r");
  char* a3 = strtok(nullptr, " \r");
  char* a4 = strtok(nullptr, " \r");

  if (!strcmp(cmd, "help")) { ConsolePrintHelp(); return; }
  if (!strcmp(cmd, "pose")) { PrintPose(); return; }
  if (!strcmp(cmd, "check")) { SelfCheck(); return; }
  if (!strcmp(cmd, "psd")) {
    Serial.print(PsdRead(PSD_FL)); Serial.print(' ');
    Serial.print(PsdRead(PSD_SL)); Serial.print(' ');
    Serial.print(PsdRead(PSD_FR)); Serial.print(' ');
    Serial.println(PsdRead(PSD_SR));
    return;
  }
  if (!strcmp(cmd, "free")) {
    ArmTorque(false);
    Serial.println(F("arm free"));
    return;
  }
  if (!strcmp(cmd, "lock")) {
    int16_t j[4];
    ArmRead(j);
    ArmTorque(true);
    ArmMove(j);
    PrintPose();
    return;
  }
  if (!strcmp(cmd, "grip") && a1) {
    Grip(atoi(a1));
    Serial.print(F("grip = "));
    Serial.println(constrain(atoi(a1), 0, 1000));
    return;
  }

  Arm();
  if (cmd[0] == 'j' && cmd[1] >= '1' && cmd[1] <= '4' && !cmd[2] && a1) {
    int16_t j[4];
    ArmRead(j);
    uint8_t k = cmd[1] - '1';
    j[k] = (a1[0] == '+' || a1[0] == '-') ? j[k] + atoi(a1) : atoi(a1);
    ArmMove(j);
    PrintPose();
  } else if (!strcmp(cmd, "arm") && a4) {
    int16_t j[4] = { (int16_t)atoi(a1), (int16_t)atoi(a2), (int16_t)atoi(a3), (int16_t)atoi(a4) };
    ArmMove(j);
    PrintPose();
  } else if (!strcmp(cmd, "p") && a1) {
    const Pose* p = PoseByName(a1);
    if (p) ArmPose(*p); else Serial.println(F("unknown pose"));
  } else if (!strcmp(cmd, "move") && a2) {
    DriveMm(DirByChar(a1[0]), atof(a2), SPEED_NORMAL_MMPS);
  } else if (!strcmp(cmd, "t1")) {
    SensorMonitor();
  } else if (!strcmp(cmd, "t2")) {
    MoveTest();
  } else if (!strcmp(cmd, "t3") && a2) {
    if (a1[0] == 'A' || a1[0] == 'B') {
      DropHere(a1[0], atoi(a2));
    } else {
      uint8_t r = PickHere(atoi(a1) - 1, atoi(a2));
      Serial.println(r == PICK_OK ? F("pick ok") : r == PICK_NOT_SEEN ? F("not seen") : F("pick fail"));
    }
  } else if (!strcmp(cmd, "t4")) {
    ScanAllColumns();
  } else if (!strcmp(cmd, "t5") && a1) {
    SegmentTest(atoi(a1));
  } else if (!strcmp(cmd, "t6")) {
    RunMission();
  } else {
    Serial.println(F("? (help)"));
  }
  gAbortArmed = false;
  BaseStop();
}
