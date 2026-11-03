#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>
#include <setjmp.h>
#include <Dynamixel2Arduino.h>
#include <Pixy2SPI_SS.h>
#include "Tuning.h"

enum Dir : uint8_t { FWD, BWD, LEFT, RIGHT };

const uint8_t PIN_SW1 = 41;
const uint8_t PIN_SW2 = 40;
const uint8_t PIN_BUZZER = 5;
const uint8_t PIN_RGB = 3;
const uint8_t PSD_FL = A0;
const uint8_t PSD_SL = A1;
const uint8_t PSD_FR = A2;
const uint8_t PSD_SR = A3;

extern Dynamixel2Arduino dxl;
extern Pixy2SPI_SS pixy;
extern jmp_buf gAbortJump;
extern bool gAbortArmed;

void InitHardware();
bool SelfCheck();
void AbortCheck();
void WaitMs(uint32_t ms);
void Led(uint8_t r, uint8_t g, uint8_t b);
void Beep(uint8_t count);

int PsdRead(uint8_t pin);

void BaseVel(float fwdMmps, float leftMmps);
void BaseStop();
void DriveMm(Dir dir, float mm, float mmps);
bool AlignPsd(uint8_t pin, int targetAdc, int tolAdc);
bool ApproachFront(uint8_t pin, int detectAdc, float maxMm, float mmps);
void FollowWallBack(float mm, float mmps, int slTargetAdc);

bool PoseValid(const Pose& p);
bool ArmMove(const int16_t* joints, bool slow = false);
bool ArmPose(const Pose& p, bool slow = false);
void ArmRead(int16_t* joints);
void Grip(int16_t value);

#endif
