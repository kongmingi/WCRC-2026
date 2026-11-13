#ifndef TUNING_H
#define TUNING_H

#include <Arduino.h>

struct Pose { int16_t j1, j2, j3, j4; };

#define REAL_WCRC_GAME_MODE   true
#define USE_SHORT_CUT_PASS    false
#define STOP_BEFORE_DROP      false
#define PAUSE_AT_EACH_STEP    false
#define RUN_CAM_X_ALIGN       true
const bool RUN_TOP_ROW_SEARCH = true;
const bool CAM_LAMP_ON = true;
const int SEARCH_SWEEP_LIMIT = 1;


struct MissionBlockAssignment { uint8_t signature; uint8_t dropCell; };
enum { RED = 1, ORANGE = 2, YELLOW = 3, GREEN = 4, BLUE = 5, PURPLE =6 };
#define ZA(n) (n)
#define ZB(n) (8 + (n))

#if USE_SHORT_CUT_PASS
const MissionBlockAssignment missionAssignments[] = {
  { 1, 8+1 }, { 3, 8+2 }, { 4, 8+3 }, { 5, 8+4 },
};
#else
const MissionBlockAssignment missionAssignments[] = {
  { 1, ZA(1) }, { 2, ZA(5) }, { 3, ZA(4) }, { 4, ZA(2) }, { 5, ZA(7) }, { 6, ZA(6) },
};
#endif


const int32_t MOBILEBASE_SPEED = 140;
const int32_t MOBILEBASE_SPEED_FAST = 160;
const int32_t MOVE_MM_PER_SEC = 140;
const int32_t SIDE_MM_PER_SEC = 140;
const int32_t SEARCH_MM_PER_SEC = 140;
const int32_t MOVE_ALIGN_SPEED = 160;

const float WHEEL_RATIO_FWD[4]   = { 1.000, 1.000, 1.000, 1.000 };
const float WHEEL_RATIO_BWD[4]   = { 1.000, 1.000, 1.000, 1.000 };
const float WHEEL_RATIO_LEFT[4]  = { 0.990, 0.990, 1.000, 1.000 };
const float WHEEL_RATIO_RIGHT[4] = { 0.990, 0.990, 1.000, 1.000 };


const uint8_t PSD_SAMPLES = 11;
const float PSD_MOVE_CONTROL_RATIO = 0.01;
const float PSD_ROTATE_CONTROL_RATIO = 0.015;
const uint32_t ALIGN_TIMEOUT = 3000;
const int MAX_MOV_ALIGN_REPEAT_COUNT = 10;
const int MAX_MOV_ALIGN_1TIME_RETRY = 5;
const int ALIGN_OUT_REPEAT_DELAY_MS = 10;

const int PSD_OBSTACLE_FR = 590;
const int PSD_OBSTACLE_MOV_TOLERANCE = 2;
const int PSD_OBSTACLE_AVG = 560;
const int PSD_OBSTACLE_ROT_ADJUST = -25;
const int PSD_MISSION_SL = 476;
const int PSD_MISSION_MOV_TOLERANCE = 3;
const int PSD_DROPBOXA_SEARCH_SL = 607;
const int PSD_SEARCH_WALL_AVG = 182;
const int PSD_SEARCH_WALL_ROT_ADJUST = -3;
const int PSD_SEARCH_WALL_TOLERANCE = 3;
const int PSD_DROPBOXA_DROP_SL = 611;
const int PSD_DROPBOXA_DROP_SPEED = 100;
const int PSD_STORAGE_FR = 312;
const int PSD_STORAGE_MOV_TOLERANCE = 2;
const int PSD_STORAGE_FRONT_AVG = 311;
const int PSD_STORAGE_COL_ROT_ADJUST[4] = { 0, -3, -3, 0 };
const int PSD_STORAGE_ROT_TOLERANCE = 3;
const int PSD_STORAGE_CELL2_SL = 200;
const int PSD_STORAGE_CELL2_SL_TOLERANCE = 2;
const int PSD_DROPBOX_B_FRONT_AVG = 554;
const int PSD_DROPBOX_B_ROT_ADJUST = -29;
const int PSD_DROPBOX_B_MOV_TOLERANCE = 2;
const int PSD_DROPBOX_B_ROT_TOLERANCE = 2;
const int PSD_DROPBOXA_FRONT_AVG = 300;
const int PSD_DROPBOXA_ROT_ADJUST = -2;
const int PSD_RETURN_A_FR = 194;
const int PSD_RETURN_STORAGE_SR = 275;


const float mm_F_BLINKING_to_OBSTACLE = 462;
const float mm_L_OBSTACLE_to_MISSION = 350;
const float mm_B_OBSTACLE_to_MISSION = 20;
const float mm_F_MISSION_to_DROPBOX = 858;
const float mm_R_DROPBOX_to_CELL1 = 130;
const float mm_R_CELL1_to_CELL2 = 60;
const float mm_R_CELL2_to_CELL3 = 70;
const float mm_R_CELL3_to_CELL4 = 70;
const float mm_L_CELL4_to_CELL3 = 65;
const float mm_L_CELL3_to_CELL2 = 78;
const float mm_L_CELL2_to_CELL1 = 70;
const float mm_L_CELL1_to_DROPSIDE = 130;
const float mm_B_ZONE_A1 = 10;
const float mm_R_DROPSIDE_to_CELL1 = mm_L_CELL1_to_DROPSIDE;
const int PSD_DROPSIDE_REF_SL = 572;
const float mm_R_REF_to_CELL1 = 110;
const float mm_R_ARRIVE_SHIFT = 30;
const float mm_L_REF_to_DROP = 7;
const float mm_B_DROPBAX_to_FINISH = 1320;
const float mm_F_TO_DROPBOX_B = 366;
const float mm_F_DROPBOX_B_to_CELL2 = 139;
const float B_RETURN_TURN1_ADJ_DEG = 0;
const float B_RETURN_TURN2_ADJ_DEG = 0;
const int32_t ROTATE_MM_PER_SEC = 100;
const float B_GO_TURN1_ADJ_DEG = 2;
const float B_GO_TURN2_ADJ_DEG = 2;
const float mm_R_DROPBOX_B_to_FINISH = 290;
const float mm_F_DROPBOX_B_to_FINISH = 950;


const int CAM_FIND_REPEAT_COUNT = 3;
const int CAM_MISSION_READ_ATTEMPTS = 3;
const int CAM_SEARCH_REPEAT_COUNT = 5;
const int CAM_DETECTING_LOOP_DELAY = 30;
const int MISSION_BLOCK_DUPLICATE_X_TOLERANCE = 10;
const int MISSION_1ST_CAM_SINGLE_BLOCK_WIDTH_PX = 45;
const int MISSION_JOINED_BLOCK_WIDTH_PERCENT = 160;
const int PIXY_SET_POINT_X[3] = { 150, 169, 168 };
const float PIXY_TOLERANCE_MM = 2;
const float SETPOINT_X_PX_TO_MM = 0.48;
const int CAM_X_ALIGN_MAX_TRY = 3;
const int MIN_PIXY_SEARCH_X = 78;
const int MAX_PIXY_SEARCH_X = 238;
const int MIN_PIXY_SEARCH_Y = 52;
const int MAX_PIXY_SEARCH_Y = 145;
const int LAYER_POSE_SETTLE_MS = 80;
const bool STORAGE_LAMP_FOR_COLOR[7] = { 0, 0, 0, 0, 0, 0, 0 };
const int STORAGE_LAMP_SETTLE_MS = 500;


const int16_t GRIP_OPEN = 250;
const int16_t GRIP_CLOSE = 740;

const int PICK_ACT_TIME = 350;
const int MOVE_LAYER_POSE_TIME = 350;
const int GRIP_ACT_TIME = 350;
const int DROP_INDEX_POS_TIME = 500;
const int CHANGE_NEXT_POS_TIME = 500;


const uint16_t ARM_P_GAIN = 1200;
const uint16_t ARM_I_GAIN = 200;
const int16_t J1_FRONT = 2046;

const int16_t JOINT_MIN[4] = { -1500, 0, 2650, 0 };
const int16_t JOINT_MAX[4] = { 2600, 4095, 6400, 4095 };

const Pose POSE_TRAVEL     = {  1001, 2970, 5442, 1468 };
const Pose POSE_CARRY      = {  2052, 2965, 5617, 1415 };
const Pose POSE_ORDER_VIEW = { 775, 2856, 5463, 1439 };
const Pose POSE_ORDER_L = {  747, 2403, 5805, 1381   };
const Pose POSE_ORDER_R = {  1049, 2944, 5174, 1797  };
const int ORDER_FRAMES = 5;
const int ORDER_MIN_VOTES = 3;
const int ORDER_POSE_MS = 800;
const int ORDER_POSE_SETTLE_MS = 300;
const int ORDER_SLOT_X_MIN[6] = {  60, 177, 249,  57, 129, 216 };
const int ORDER_SLOT_X_MAX[6] = { 176, 248, 316, 128, 215, 316 };
const int ORDER_SLOT_Y_MIN[6] = {   0,  30,  90, 115, 115, 115 };
const int ORDER_SLOT_Y_MAX[6] = { 100, 120, 200, 180, 180, 180 };
const int ORDER_MIN_BLOCK_H = 15;
const bool ORDER_GUESS_IF_UNSURE = true;
const Pose POSE_DRIVE      = { 2013, 3130, 5233, 1470 };
const Pose POSE_HUB        = { 2013, 3130, 5233, 1470 };

const Pose POSE_SCAN[3]      = { { 2056, 2305, 5085, 2495 }, { 2036, 2659, 5733, 1364 }, { 2039, 2477, 6063, 1362 } };
const Pose POSE_PICK_APPROACH[3] = { { 2085, 2584, 5209, 2261 }, { 2057, 2729, 5636, 1662 }, { 2038, 2586, 6188, 1361 } };
const Pose POSE_PICK_GRIP[3]     = { { 2079, 1963, 5505, 2659 }, { 2060, 2205, 5803, 2127 }, { 2063, 2161, 6428, 1533 } };

const Pose POSE_ZONE_A[8] = {
  { 683, 1856, 6317, 1836 }, {823, 1905, 6316, 1555 }, { 971, 1931, 6314, 1462  }, { 1154, 1953, 6313, 1462 },
  {  593, 2027, 5991, 1849 }, { 730, 2211, 5844, 1792  }, { 921, 2399, 5815, 1623 }, { 1154, 2336, 5819, 1620 },
};
const Pose POSE_ZONE_B[8] = {
  { 557, 2316, 3523, 1345 }, { 745, 1975, 4217, 1778 }, { 941, 1875, 4308, 1777 }, { 1170, 1907, 4295, 1755 },
  { 463, 2001, 4003, 1401 }, { 657, 1701, 4559, 1757 }, { 918, 1501, 4621, 1683 }, { 1224, 1530, 4561, 1661 },
};

#endif
