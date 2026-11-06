#include "Console.h"
#include "Hardware.h"
#include "Mission.h"
#include "Vision.h"

static char line[64];

static void ArmTorque(bool on)
{
  for (uint8_t id = 5; id <= 8; id++)
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, id, on ? 1 : 0);
}

static void Arm()
{
  delay(50);
  while (Con.available()) Con.read();
  gAbortArmed = true;
}

static void WaitKey()
{
  Con.println(F("press enter"));
  while (!Con.available()) delay(10);
  while (Con.available()) { Con.read(); delay(2); }
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
    if (n[1] == 'g') return &POSE_PICK_GRIP[k];
    if (n[1] == 'a') return &POSE_PICK_APPROACH[k];
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
  Con.println(F("grip 650 | j3 +20 | j3 1500 | arm a b c d | pose | p travel | free | lock"));
  Con.println(F("move F 300 (F/B/L/R) | fr 183 | fl 183 | sl 637 | psd | check"));
  Con.println(F("t1 sensor | t2 move | t3 2 4 (layer color) | t3 A 5 (drop)"));
  Con.println(F("t4 scan | t5 1~6 segment (t5 4 3 = zone A cell 3) | rot"));
  Con.println(F("order 5 1 6 3 | zone A 1 5 | list | clear"));
  Con.println(F("t6 storage mission from col1 | t7 full mission | go = resume"));
  Con.println(F("pose names: travel order hub scan1-3 pa1-3 pg1-3 za1-8 zb1-8"));
  Con.println(F("stop: any key or SW2"));
}

static void PrintPose()
{
  int16_t j[4];
  ArmRead(j);
  Con.print(F("{ "));
  for (uint8_t i = 0; i < 4; i++) {
    Con.print(j[i]);
    Con.print(i < 3 ? F(", ") : F(" }"));
  }
  Con.println();
}

static void SensorMonitor()
{
  while (true) {
    AbortCheck();
    Con.print(F("PSD FL SL FR SR: "));
    Con.print(PsdRead(PSD_FL)); Con.print(' ');
    Con.print(PsdRead(PSD_SL)); Con.print(' ');
    Con.print(PsdRead(PSD_FR)); Con.print(' ');
    Con.println(PsdRead(PSD_SR));
    if (pixy.ccc.getBlocks(true, PIXY_SIGMAP) >= 0) {
      for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++) {
        Block& b = pixy.ccc.blocks[i];
        Con.print(F("  sig ")); Con.print(b.m_signature);
        Con.print(F(" x ")); Con.print(b.m_x);
        Con.print(F(" y ")); Con.print(b.m_y);
        Con.print(F(" w ")); Con.print(b.m_width);
        Con.print(F(" h ")); Con.println(b.m_height);
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
    Con.print(names[i]);
    Con.print(' ');
    Con.print(mm[i]);
    Con.println(F(" mm"));
    WaitKey();
    Arm();
    DriveMm(DirByChar(names[i]), mm[i], SPEED_NORMAL_MMPS);
    gAbortArmed = false;
    Con.println(F("measure real distance"));
  }
}

static void SegmentTest(uint8_t n, uint8_t cell)
{
  if (n == 1) SegStartToLine();
  else if (n == 2) SegLineToOrder();
  else if (n == 3) SegOrderToStorage();
  else if (n == 4) Deliver('A', cell);
  else if (n == 5) Deliver('B', cell);
  else if (n == 6) SegFinish();
}

void ConsoleRunLine()
{
  uint8_t len = Con.readBytesUntil('\n', line, sizeof(line) - 1);
  line[len] = 0;
  char* cmd = strtok(line, " \r");
  if (!cmd) return;
  char* arg[8];
  uint8_t argc = 0;
  for (char* p = strtok(nullptr, " \r"); p && argc < 8; p = strtok(nullptr, " \r")) arg[argc++] = p;
  for (uint8_t i = argc; i < 8; i++) arg[i] = nullptr;
  char* a1 = arg[0];
  char* a2 = arg[1];
  char* a3 = arg[2];
  char* a4 = arg[3];

  if (!strcmp(cmd, "help")) { ConsolePrintHelp(); return; }
  if (!strcmp(cmd, "pose")) { PrintPose(); return; }
  if (!strcmp(cmd, "list")) { PrintPlan(); return; }
  if (!strcmp(cmd, "clear")) { ClearPlan(); PrintPlan(); return; }
  if (!strcmp(cmd, "order")) {
    uint8_t colors[8];
    uint8_t n = 0;
    for (uint8_t i = 0; i < argc; i++) {
      uint8_t c = atoi(arg[i]);
      if (c >= 1 && c <= 6) colors[n++] = c;
    }
    SetUserOrder(colors, n);
    PrintPlan();
    return;
  }
  if (!strcmp(cmd, "zone") && a3) {
    if (!SetZone(toupper(a1[0]), atoi(a2), atoi(a3))) Con.println(F("? zone A 1 5 (zone cell color, color 0 = clear)"));
    PrintPlan();
    return;
  }
  if (!strcmp(cmd, "check")) { SelfCheck(); return; }
  if (!strcmp(cmd, "psd")) {
    Con.print(PsdRead(PSD_FL)); Con.print(' ');
    Con.print(PsdRead(PSD_SL)); Con.print(' ');
    Con.print(PsdRead(PSD_FR)); Con.print(' ');
    Con.println(PsdRead(PSD_SR));
    return;
  }
  if (!strcmp(cmd, "free")) {
    ArmTorque(false);
    Con.println(F("arm free"));
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
    Con.print(F("grip = "));
    Con.println(constrain(atoi(a1), 0, 1000));
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
    if (p) ArmPose(*p); else Con.println(F("unknown pose"));
  } else if (!strcmp(cmd, "move") && a2) {
    DriveMm(DirByChar(a1[0]), atof(a2), SPEED_NORMAL_MMPS);
  } else if (!strcmp(cmd, "fr") && a1) {
    AlignPsd(PSD_FR, atoi(a1), PSD_ALIGN_TOL_ADC);
    Con.println(PsdRead(PSD_FR));
  } else if (!strcmp(cmd, "sl") && a1) {
    AlignPsd(PSD_SL, atoi(a1), PSD_ALIGN_TOL_ADC);
    Con.println(PsdRead(PSD_SL));
  } else if (!strcmp(cmd, "fl") && a1) {
    AlignPsd(PSD_FL, atoi(a1), PSD_ALIGN_TOL_ADC);
    Con.println(PsdRead(PSD_FL));
  } else if (!strcmp(cmd, "t1")) {
    SensorMonitor();
  } else if (!strcmp(cmd, "t2")) {
    MoveTest();
  } else if (!strcmp(cmd, "t3") && a2) {
    char z = toupper(a1[0]);
    uint8_t l = atoi(a1);
    uint8_t c = atoi(a2);
    if (z == 'A' || z == 'B') {
      DropHere(z, c);
    } else if (l < 1 || l > 3 || c < 1 || c > 6) {
      Con.println(F("? t3 1~3 1~6 | t3 A/B 1~8"));
    } else {
      uint8_t r = PickHere(l - 1, c);
      Con.println(r == PICK_OK ? F("pick ok") : r == PICK_NOT_SEEN ? F("not seen") : F("pick fail"));
    }
  } else if (!strcmp(cmd, "t4")) {
    ScanAllColumns();
  } else if (!strcmp(cmd, "t5") && a1) {
    SegmentTest(atoi(a1), a2 ? atoi(a2) : 1);
  } else if (!strcmp(cmd, "rot")) {
    AlignHeading();
    Con.print(PsdRead(PSD_FL)); Con.print(' ');
    Con.println(PsdRead(PSD_FR));
  } else if (!strcmp(cmd, "t6")) {
    RunStorage();
  } else if (!strcmp(cmd, "t7")) {
    RunMission();
  } else if (!strcmp(cmd, "go")) {
    if (MissionResumable()) ResumeMission(); else Con.println(F("nothing to resume"));
  } else {
    Con.println(F("? (help)"));
  }
  gAbortArmed = false;
  BaseStop();
}
