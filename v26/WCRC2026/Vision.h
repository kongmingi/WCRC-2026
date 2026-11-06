#ifndef VISION_H
#define VISION_H

#include <Arduino.h>

const uint8_t INV_UNKNOWN = 0;
const uint8_t INV_EMPTY = 7;
const uint8_t INV_FAILED = 8;
const uint8_t MAX_ORDER = 12;

uint8_t ReadOrderFrame(uint8_t* seq);
uint8_t ReadOrder(uint8_t* out);
void ScanView(uint8_t layer, uint8_t* column);
bool FindBlockX(uint8_t sig, uint8_t layer, int16_t& x);
bool CellHas(uint8_t sig, uint8_t layer);
bool CenterOnBlock(uint8_t sig, uint8_t layer, float& movedRightMm);

#endif
