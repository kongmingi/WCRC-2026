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
  Con.println(F("SW1: start mission"));
}

void loop()
{
  if (setjmp(gAbortJump)) {
    BaseStop();
    Led(255, 0, 0);
    Beep(3);
    Con.println(F("STOPPED"));
  }
  if (digitalRead(PIN_SW1) == LOW) {
    while (digitalRead(PIN_SW1) == LOW) delay(10);
    delay(200);
    while (Con.available()) Con.read();
    gAbortArmed = true;
    RunMission();
    gAbortArmed = false;
  }
  if (Con.available()) ConsoleRunLine();
}
