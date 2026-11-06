#include "Hardware.h"
#include "Mission.h"
#include "Console.h"

void setup()
{
  Con.begin(115200);
  Con.setTimeout(50);
  InitHardware();
  SelfCheck();
  Grip(GRIP_OPEN);
  ConsolePrintHelp();
  Con.println(F("SW1: full mission (or resume after stop)"));
}

void loop()
{
  if (setjmp(gAbortJump)) {
    BaseStop();
    Led(255, 0, 0);
    Beep(3);
    MissionPaused();
    Con.println(F("STOPPED"));
    if (MissionResumable()) Con.println(F("SW1 or go: resume"));
  }
  if (digitalRead(PIN_SW1) == LOW) {
    while (digitalRead(PIN_SW1) == LOW) delay(10);
    delay(200);
    while (Con.available()) Con.read();
    gAbortArmed = true;
    if (MissionResumable()) ResumeMission(); else RunMission();
    gAbortArmed = false;
  }
  if (Con.available()) ConsoleRunLine();
}
