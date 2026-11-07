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
const bool CAM_LAMP_ON = false;
const int SEARCH_SWEEP_LIMIT = 1;


struct MissionBlockAssignment { uint8_t signature; uint8_t dropCell; };

#if USE_SHORT_CUT_PASS
const MissionBlockAssignment missionAssignments[] = {
  { 1, 8+1 }, { 3, 8+2 }, { 4, 8+3 }, { 5, 8+4 },
};
#else
const MissionBlockAssignment missionAssignments[] = {
  { 1, 1 }, { 1, 8+5 }, { 3, 4 }, { 4, 8+2 }, { 4, 5 }, { 5, 6 },
};
#endif


const int32_t MOBILEBASE_SPEED = 100;
const int32_t MOBILEBASE_SPEED_FAST = 160;
const int32_t MOVE_MM_PER_SEC = 100;
const int32_t SIDE_MM_PER_SEC = 100;
const int32_t SEARCH_MM_PER_SEC = 100;
const int32_t MOVE_ALIGN_SPEED = 160;

const float WHEEL_RATIO_FWD[4]   = { 1.000, 1.000, 1.000, 1.000 };
const float WHEEL_RATIO_BWD[4]   = { 1.000, 1.000, 1.000, 1.000 };
const float WHEEL_RATIO_LEFT[4]  = { 1.000, 1.000, 1.000, 1.000 };
const float WHEEL_RATIO_RIGHT[4] = { 1.000, 1.000, 1.000, 1.000 };


const uint8_t PSD_SAMPLES = 11;
const float PSD_MOVE_CONTROL_RATIO = 0.01;
const float PSD_ROTATE_CONTROL_RATIO = 0.015;
const uint32_t ALIGN_TIMEOUT = 3000;
const int MAX_MOV_ALIGN_REPEAT_COUNT = 3;
const int MAX_MOV_ALIGN_1TIME_RETRY = 5;
const int ALIGN_OUT_REPEAT_DELAY_MS = 10;

const int PSD_OBSTACLE_FR = 590;
const int PSD_OBSTACLE_MOV_TOLERANCE = 2;
const int PSD_MISSION_SL = 476;
const int PSD_MISSION_MOV_TOLERANCE = 3;
const int PSD_DROPBOXA_SEARCH_SL = 558;
const int PSD_DROPBOXA_DROP_SL = 632;
const int PSD_DROPBOXA_DROP_SPEED = 100;
const int PSD_STORAGE_FR = 304;
const int PSD_STORAGE_MOV_TOLERANCE = 2;
const int PSD_STORAGE_FRONT_AVG = 311;
const int PSD_STORAGE_ROT_ADJUST = 8;
const int PSD_STORAGE_ROT_TOLERANCE = 2;
const int PSD_STORAGE_CELL2_SL = 200;
const int PSD_STORAGE_CELL2_SL_TOLERANCE = 2;
const int PSD_DROPBOX_B_FRONT_AVG = 591;
const int PSD_DROPBOX_B_ROT_ADJUST = 6;
const int PSD_DROPBOX_B_MOV_TOLERANCE = 2;
const int PSD_DROPBOX_B_ROT_TOLERANCE = 2;
const int PSD_DROPBOXA_FRONT_AVG = 285;
const int PSD_DROPBOXA_ROT_ADJUST = 13;
const int PSD_RETURN_A_FR = 194;
const int PSD_RETURN_STORAGE_SR = 275;


const float mm_F_BLINKING_to_OBSTACLE = 462;
const float mm_L_OBSTACLE_to_MISSION = 350;
const float mm_B_OBSTACLE_to_MISSION = 50;
const float mm_F_MISSION_to_DROPBOX = 858;
const float mm_R_DROPBOX_to_CELL1 = 85;
const float mm_R_CELL1_to_CELL2 = 67;
const float mm_R_CELL2_to_CELL3 = 68;
const float mm_R_CELL3_to_CELL4 = 67;
const float mm_L_CELL4_to_CELL3 = mm_R_CELL3_to_CELL4 - 3;
const float mm_L_CELL3_to_CELL2 = mm_R_CELL2_to_CELL3;
const float mm_L_CELL2_to_CELL1 = mm_R_CELL1_to_CELL2;
const float mm_L_CELL1_to_DROPSIDE = 124;
const float mm_R_DROPSIDE_to_CELL1 = mm_L_CELL1_to_DROPSIDE;
const float mm_B_DROPBAX_to_FINISH = 1320;
const float mm_F_TO_DROPBOX_B = 366;
const float mm_F_DROPBOX_B_to_CELL2 = mm_L_CELL4_to_CELL3 + mm_L_CELL3_to_CELL2 - 8;
const float mm_R_DROPBOX_B_to_FINISH = 290;
const float mm_F_DROPBOX_B_to_FINISH = 950;


const int CAM_FIND_REPEAT_COUNT = 3;
const int CAM_MISSION_READ_ATTEMPTS = 3;
const int CAM_SEARCH_REPEAT_COUNT = 5;
const int CAM_DETECTING_LOOP_DELAY = 30;
const int MISSION_BLOCK_DUPLICATE_X_TOLERANCE = 10;
const int MISSION_1ST_CAM_SINGLE_BLOCK_WIDTH_PX = 45;
const int MISSION_JOINED_BLOCK_WIDTH_PERCENT = 160;
const int PIXY_SET_POINT_X = 166;
const float PIXY_TOLERANCE_MM = 2;
const float SETPOINT_X_PX_TO_MM = 0.30;
const int CAM_X_ALIGN_MAX_TRY = 3;
const int MIN_PIXY_SEARCH_X = 78;
const int MAX_PIXY_SEARCH_X = 238;
const int MIN_PIXY_SEARCH_Y = 52;
const int MAX_PIXY_SEARCH_Y = 156;
const int LAYER_POSE_SETTLE_MS = 80;


const int16_t GRIP_OPEN = 250;
const int16_t GRIP_CLOSE = 740;

const int PICK_ACT_TIME = 350;
const int MOVE_LAYER_POSE_TIME = 350;
const int GRIP_ACT_TIME = 350;
const int DROP_INDEX_POS_TIME = 500;
const int CHANGE_NEXT_POS_TIME = 500;


const uint16_t ARM_P_GAIN = 1200;
const uint16_t ARM_I_GAIN = 200;
const int16_t J1_FRONT = 1013;

const int16_t JOINT_MIN[4] = { -1500, 0, 2650, 0 };
const int16_t JOINT_MAX[4] = { 2600, 4095, 5600, 4095 };

const Pose POSE_TRAVEL     = { -16, 1220, 5004, 1843 };
const Pose POSE_ORDER_VIEW = { 1016, 1073, 4658, 1408 };
const Pose POSE_DRIVE      = { 1013, 1145, 4701, 1557 };
const Pose POSE_HUB        = { 1623, 1350, 4852, 2039 };

const Pose POSE_SCAN[3]      = { { 1013, 1838, 5138, 2559 }, { 1013, 1622, 4916, 2267 }, { 1013, 1565, 4366, 1648 } };
const Pose POSE_PICK_APPROACH[3] = { { 1013, 1765, 5118, 2728 }, { 1013, 1489, 4642, 1971 }, { 1013, 1653, 3947, 1439 } };
const Pose POSE_PICK_GRIP[3]     = { { 1013, 2086, 4793, 2726 }, { 1013, 1829, 4458, 2104 }, { 1013, 1852, 3940, 1643 } };

const Pose POSE_ZONE_A[8] = {
  { 230, 2321, 3889, 1989 }, { 341, 2125, 4140, 1943 }, { 503, 2045, 4250, 1943 }, { 681, 2017, 4257, 1942 },
  { 140, 2117, 4260, 2075 }, { 267, 1682, 4613, 2080 }, { 472, 1621, 4748, 2080 }, { 729, 1642, 4687, 2076 },
};
const Pose POSE_ZONE_B[8] = {
  { -594, 2726, 3225, 1857 }, { -469, 2690, 3225, 1673 }, { -320, 2701, 3226, 1674 }, { -184, 2694, 3225, 1675 },
  { -643, 2703, 2726, 1151 }, { -492, 2476, 3262, 1123 }, { -272, 2468, 3262, 1123 }, { -133, 2468, 3258, 1154 },
};

#endif
