#include "Hardware.h"
#include "Mission.h"
#include "Console.h"

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(50);
  InitHardware();
  SelfCheck();
  Grip(GRIP_OPEN);
  if (PoseValid(POSE_TRAVEL)) ArmPose(POSE_TRAVEL);
  ConsolePrintHelp();
  Serial.println(F("SW1: start mission"));
}

void loop()
{
  if (setjmp(gAbortJump)) {
    BaseStop();
    Led(255, 0, 0);
    Beep(3);
    Serial.println(F("STOPPED"));
  }
  if (digitalRead(PIN_SW1) == LOW) {
    while (digitalRead(PIN_SW1) == LOW) delay(10);
    delay(200);
    while (Serial.available()) Serial.read();
    gAbortArmed = true;
    RunMission();
    gAbortArmed = false;
  }
  if (Serial.available()) ConsoleRunLine();
}
