#ifndef TUNING_H
#define TUNING_H

#include <Arduino.h>

struct Task { uint8_t color; char zone; uint8_t cell; };
struct Pose { int16_t j1, j2, j3, j4; };

const Task MISSION[] = {
  { 1, 'A', 1 },
  { 1, 'A', 2 },
  { 3, 'A', 5 },
  { 4, 'B', 1 },
  { 4, 'B', 2 },
  { 6, 'B', 5 },
};
const uint8_t MISSION_COUNT = sizeof(MISSION) / sizeof(MISSION[0]);

const bool ORDER_READ_ENABLE = true;
const bool ORDER_REVERSE = false;
const bool PICK_VERIFY_ENABLE = false;
const uint8_t PICK_RETRY = 1;

const uint32_t MATCH_LIMIT_MS = 115000;
const uint32_t RETURN_RESERVE_MS = 15000;


const float SPEED_FAST_MMPS = 200;
const float SPEED_NORMAL_MMPS = 144;
const float SPEED_SLOW_MMPS = 54;
const float SPEED_CENTER_MMPS = 35;
const float SPEED_SIDE_MMPS = 100;

const float WHEEL_RATIO_FWD[4]   = { 1.000, 0.988, 1.000, 0.988 };
const float WHEEL_RATIO_BWD[4]   = { 0.998, 1.000, 0.998, 1.000 };
const float WHEEL_RATIO_LEFT[4]  = { 0.960, 0.975, 0.975, 0.995 };
const float WHEEL_RATIO_RIGHT[4] = { 0.980, 1.000, 0.990, 0.990 };


const uint8_t PSD_SAMPLES = 11;
const int PSD_ALIGN_TOL_ADC = 3;
const float PSD_ALIGN_GAIN = 0.01;
const float PSD_ALIGN_MAX_MMPS = 120;
const float PSD_ALIGN_MIN_MMPS = 8;
const uint32_t PSD_ALIGN_TIMEOUT_MS = 3000;


const float DIST_START_TO_LINE_MM = 330;
const int PSD_LINE_FR_ADC = 536;

const float DIST_LINE_TO_ORDER_MM = 340;
const int PSD_ORDER_SL_ADC = 495;

const float DIST_ORDER_TO_STORAGE_MM = 858;
const float ORDER_TO_COL1_RIGHT_MM = 94;
const int PSD_STORAGE_DETECT_ADC = 150;
const float STORAGE_APPROACH_MAX_MM = 300;
const int PSD_STORAGE_FR_ADC = 183;
const int PSD_STORAGE_L1_FR_ADC = 183;
const int PSD_GRIP_FR_ADC = 219;
const int PSD_STORAGE_COL1_SL_ADC = 0;
const float STORAGE_COL_PITCH_MM = 67;

const float ZONE_A_LEFT_MM = 120;
const float ZONE_A_FWD_MM = 10;
const float ZONE_A_RETURN_RIGHT_MM = 120;
const int PSD_ZONE_A_SL_ADC = 634;
const float ZONE_A_CELL_SHIFT_MM[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

const float ZONE_B_LANE_RIGHT_MM = 30;
const float ZONE_B_BACK_MM = 350;
const int PSD_ZONE_B_SR_ADC = 0;
const float ZONE_B_CELL_SHIFT_MM[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

const float FINISH_LANE_RIGHT_MM = 100;
const float FINISH_BACK_MM = 1310;
const float FINISH_LEFT_MM = 0;
const float FINISH_SPEED_MMPS = 144;
const int PSD_FINISH_SL_ADC = 476;
const int FINISH_TRACK_BAND_ADC = 80;
const float FINISH_TRACK_GAIN = 0.5;
const float FINISH_TRACK_MAX_MMPS = 30;


const uint8_t PIXY_SIGMAP = 0x3F;
const bool CAM_LAMP_ON = false;

const uint8_t ORDER_FRAMES = 5;
const uint8_t ORDER_MIN_AGREE = 2;
const int16_t ORDER_X_MIN_PX = 0;
const int16_t ORDER_X_MAX_PX = 315;
const int16_t ORDER_Y_MIN_PX = 0;
const int16_t ORDER_Y_MAX_PX = 207;
const int16_t ORDER_BLOCK_WIDTH_PX = 45;
const int16_t ORDER_DUP_X_PX = 10;

const uint8_t SCAN_FRAMES = 8;
const uint8_t SCAN_MIN_HITS = 4;
const uint16_t SCAN_SETTLE_MS = 600;
const int16_t CAM_GRIP_X_PX[3] = { 170, 171, 163 };
const int16_t CAM_COLUMN_HALF_WIDTH_PX = 45;
const int16_t SCAN_Y_MIN_PX[3] = { 70, 60, 50 };
const int16_t SCAN_Y_MAX_PX[3] = { 150, 160, 140 };

const int16_t CENTER_TOL_PX = 4;
const float CENTER_MM_PER_PX = 0.3;
const float CENTER_GAIN = 0.8;
const float CENTER_MIN_STEP_MM = 1;
const float CENTER_MAX_STEP_MM = 6;
const uint32_t CENTER_TIMEOUT_MS = 4000;


const int16_t GRIP_OPEN = 250;
const int16_t GRIP_CLOSE = 740;
const int16_t GRIP_RELEASE = 450;
const uint16_t GRIP_WAIT_MS = 350;

const float ARM_MS_PER_TICK = 1;
const float ARM_SLOW_MS_PER_TICK = 1.5;
const uint16_t ARM_MIN_MS = 300;
const float ARM_ACCEL_RATIO = 0.5;
const uint16_t POSE_SETTLE_MS = 100;
const uint16_t PICK_SETTLE_MS = 500;

const uint16_t ARM_P_GAIN = 1200;
const uint16_t ARM_I_GAIN = 200;

const int16_t JOINT_MIN[4] = { -1500, 0, 2650, 0 };
const int16_t JOINT_MAX[4] = { 2600, 4095, 5600, 4095 };

const Pose POSE_TRAVEL     = { 631, 1469, 4278, 1344 };
const Pose POSE_ORDER_VIEW = { 454, 1459, 4453, 1351 };
const Pose POSE_HUB        = { 1623, 1350, 4852, 2039 };

const Pose POSE_SCAN[3]      = { { 1602, 2395, 5012, 3151 }, { 1665, 1921, 4985, 2742 }, { 1657, 1658, 4697, 2205 } };
const Pose POSE_PICK_GRIP[3] = { { 1601, 2200, 4852, 2936 }, { 1598, 1770, 4694, 2377 }, { 1638, 1756, 4230, 1902 } };

const Pose POSE_ZONE_A[8] = {
  { 230, 2321, 3889, 1989 }, { 341, 2125, 4140, 1943 }, { 503, 2045, 4250, 1943 }, { 681, 2017, 4257, 1942 },
  { 140, 2117, 4260, 2075 }, { 267, 1682, 4613, 2080 }, { 472, 1621, 4748, 2080 }, { 729, 1642, 4687, 2076 },
};
const Pose POSE_ZONE_B[8] = {
  { -594, 2726, 3225, 1857 }, { -469, 2690, 3225, 1673 }, { -320, 2701, 3226, 1674 }, { -184, 2694, 3225, 1675 },
  { -643, 2703, 2726, 1151 }, { -492, 2476, 3262, 1123 }, { -272, 2468, 3262, 1123 }, { -133, 2468, 3258, 1154 },
};

const float LOW_VOLTAGE_V = 11.0;

#endif
