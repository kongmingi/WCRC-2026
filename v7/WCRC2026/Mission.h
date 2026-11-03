#ifndef MISSION_H
#define MISSION_H

#include <Arduino.h>

void ResetMission();
void RunMission();

void SegStartToLine();
void SegLineToOrder();
void SegOrderToStorage();
void SegFinish();

const uint8_t PICK_OK = 0;
const uint8_t PICK_NOT_SEEN = 1;
const uint8_t PICK_FAIL = 2;

void ScanAllColumns();
uint8_t PickHere(uint8_t layer, uint8_t sig);
void DropHere(char zone, uint8_t cell);
void Deliver(char zone, uint8_t cell);

#endif
