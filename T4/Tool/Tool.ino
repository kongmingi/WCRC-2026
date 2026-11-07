#include "Debug.h"
#include "Devices.h"
#include "Manipulator.h"
#include "Mobilebase.h"
#include <EEPROM.h>

Pixy2 pixy;
Dynamixel2Arduino dxl( DXL_SERIAL, DXL_DIR_PIN );

const int32_t MOVE_ALIGN_SPEED = MOBILEBASE_SPEED_FAST;
const int32_t MOVE_SPEED = MOBILEBASE_SPEED_FAST;
const int POSE_TIME = 1000;
const int PICK_TIME = 350;
const int GRIP_TIME = 350;

#define MAX_MOV_ALIGN_REPEAT_COUNT  3
#define MAX_MOV_ALIGN_1TIME_RETRY   5
#define ALIGN_OUT_REPEAT_DELAY_MS   10

#define PIXY_SET_POINT_X   (158 + 8)
#define MIN_PIXY_SEARCH_X  (158 - 80)
#define MAX_PIXY_SEARCH_X  (158 + 80)
#define MIN_PIXY_SEARCH_Y  (208 / 4)
#define MAX_PIXY_SEARCH_Y  (208 * 3 / 4)

const uint8_t SEARCH_POSE = 4;
const uint8_t APP_ID[3]  = { 8, 5, 28 };
const uint8_t GRIP_ID[3] = { 9, 6, 29 };
const uint8_t OUT_ID[3]  = { 10, 7, 30 };

const char N1[] PROGMEM = "FIND1";
const char N2[] PROGMEM = "FIND2";
const char N3[] PROGMEM = "FRONT";
const char N4[] PROGMEM = "SEARCH*";
const char N5[] PROGMEM = "L2_APP*";
const char N6[] PROGMEM = "L2_GRIP*";
const char N7[] PROGMEM = "L2_OUT*";
const char N8[] PROGMEM = "L1_APP*";
const char N9[] PROGMEM = "L1_GRIP*";
const char N10[] PROGMEM = "L1_OUT*";
const char N11[] PROGMEM = "A1";
const char N12[] PROGMEM = "A2";
const char N13[] PROGMEM = "A3";
const char N14[] PROGMEM = "A4";
const char N15[] PROGMEM = "A5";
const char N16[] PROGMEM = "A6";
const char N17[] PROGMEM = "A7";
const char N18[] PROGMEM = "A8";
const char N19[] PROGMEM = "B1";
const char N20[] PROGMEM = "B2";
const char N21[] PROGMEM = "B3";
const char N22[] PROGMEM = "B4";
const char N23[] PROGMEM = "B5";
const char N24[] PROGMEM = "B6";
const char N25[] PROGMEM = "B7";
const char N26[] PROGMEM = "B8";
const char N27[] PROGMEM = "ORDER";
const char N28[] PROGMEM = "L3_APP*";
const char N29[] PROGMEM = "L3_GRIP*";
const char N30[] PROGMEM = "L3_OUT*";
const char N31[] PROGMEM = "SCAN_L3";
const char N32[] PROGMEM = "SCAN_L2";
const char N33[] PROGMEM = "SCAN_L1";
const char N34[] PROGMEM = "START";
const char N35[] PROGMEM = "FIN1";
const char N36[] PROGMEM = "FIN2";
const char* const POSE_NAME[36] PROGMEM = { N1, N2, N3, N4, N5, N6, N7, N8, N9, N10, N11, N12, N13, N14, N15, N16, N17, N18,
  N19, N20, N21, N22, N23, N24, N25, N26, N27, N28, N29, N30, N31, N32, N33, N34, N35, N36 };

const int16_t OURS[23][5] PROGMEM = {
  { 4, 1051, 2116, 417, 930 }, { 34, 1051, 2116, 417, 930 }, { 27, 2006, 1376, 1186, 1314 },
  { 33, 2010, 2076, 1841, 2559 }, { 32, 1999, 1860, 1619, 2267 }, { 31, 1994, 1803, 1069, 1648 },
  { 8, 2059, 2003, 1821, 2728 }, { 9, 2058, 2324, 1496, 2726 }, { 10, 2059, 2003, 1821, 2728 },
  { 5, 2045, 1727, 1345, 1971 }, { 6, 2042, 2067, 1161, 2104 }, { 7, 2045, 1727, 1345, 1971 },
  { 28, 2045, 1891, 650, 1439 }, { 29, 2046, 2090, 643, 1643 }, { 30, 2045, 1891, 650, 1439 },
  { 11, 610, 2559, 592, 1989 }, { 12, 721, 2363, 843, 1943 }, { 13, 883, 2283, 953, 1943 }, { 14, 1061, 2255, 960, 1942 },
  { 15, 520, 2355, 963, 2075 }, { 16, 647, 1920, 1316, 2080 }, { 17, 852, 1859, 1451, 2080 }, { 18, 1109, 1880, 1390, 2076 },
};

class Dual : public Print {
public:
  size_t write( uint8_t c ) { Serial.write( c ); Serial2.write( c ); return 1; }
};
Dual out;

int16_t slPSD, valPSD;
bool pixyOk = false;

bool CenterJ1( uint8_t id )
{
  return id == SEARCH_POSE || ( id >= 5 && id <= 10 ) || ( id >= 28 && id <= 30 );
}

void PrintName( uint8_t id )
{
  char buf[10];
  strcpy_P( buf, (PGM_P) pgm_read_word( &POSE_NAME[id - 1] ) );
  out.print( buf );
}

void PerformLeftMoveAlign( int psdTarget, int psdTolerance )
{
  ChangeMobilebaseMode2VelocityControlMode( dxl );
  const unsigned long startTime = millis();
  bool bTimeout = false;
  for( int i = 0; i < MAX_MOV_ALIGN_REPEAT_COUNT; i++ ) {
    int nAdjusted = 0;
    for( int j = 0; j < MAX_MOV_ALIGN_1TIME_RETRY; j++ ) {
      if( millis() - startTime >= ALIGN_TIMEOUT ) { bTimeout = true; break; }
      GetValueFromSideLeftPSDSensor( &slPSD );
      if( ! MoveAlignWithLeftSidePSD( dxl, psdTarget - slPSD, psdTolerance, PSD_MOVE_CONTROL_RATIO, i == 0 && j == 0, MOVE_ALIGN_SPEED ) ) break;
      nAdjusted++;
    }
    if( bTimeout || nAdjusted == 0 ) break;
    delay( ALIGN_OUT_REPEAT_DELAY_MS );
  }
  SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode( dxl );
}

void PerformRightMoveAlign( int psdTarget, int psdTolerance )
{
  ChangeMobilebaseMode2VelocityControlMode( dxl );
  const unsigned long startTime = millis();
  bool bTimeout = false;
  int16_t srPSD = 0;
  for( int i = 0; i < MAX_MOV_ALIGN_REPEAT_COUNT; i++ ) {
    int nAdjusted = 0;
    for( int j = 0; j < MAX_MOV_ALIGN_1TIME_RETRY; j++ ) {
      if( millis() - startTime >= ALIGN_TIMEOUT ) { bTimeout = true; break; }
      GetValueFromSideRightPSDSensor( &srPSD );
      if( ! MoveAlignWithLeftSidePSD( dxl, srPSD - psdTarget, psdTolerance, PSD_MOVE_CONTROL_RATIO, i == 0 && j == 0, MOBILEBASE_SPEED ) ) break;
      nAdjusted++;
    }
    if( bTimeout || nAdjusted == 0 ) break;
    delay( ALIGN_OUT_REPEAT_DELAY_MS );
  }
  SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode( dxl );
}

void PerformFrontMoveAlign( int psdTarget, int psdTolerance )
{
  ChangeMobilebaseMode2VelocityControlMode( dxl );
  const unsigned long startTime = millis();
  bool bTimeout = false;
  for( int i = 0; i < MAX_MOV_ALIGN_REPEAT_COUNT; i++ ) {
    int nAdjusted = 0;
    for( int j = 0; j < MAX_MOV_ALIGN_1TIME_RETRY; j++ ) {
      if( millis() - startTime >= ALIGN_TIMEOUT ) { bTimeout = true; break; }
      GetValueFromFrontRightPSDSensor( &valPSD );
      if( ! MoveAlignWithFrontRightPSD( dxl, psdTarget - valPSD, psdTolerance, PSD_MOVE_CONTROL_RATIO, i == 0 && j == 0, MOVE_ALIGN_SPEED ) ) break;
      nAdjusted++;
    }
    if( bTimeout || nAdjusted == 0 ) break;
    delay( ALIGN_OUT_REPEAT_DELAY_MS );
  }
  SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode( dxl );
}

void PerformMoveRotateAlign( int frontAvgTarget, int moveTolerance, int rotateTolerance, int rotationAdjust )
{
  ChangeMobilebaseMode2VelocityControlMode( dxl );
  const unsigned long startTime = millis();
  bool bTimeout = false;
  int16_t flPSD, frPSD;
  for( int i = 0; i < MAX_MOV_ALIGN_REPEAT_COUNT; i++ ) {
    int nAdjusted = 0;
    for( int j = 0; j < MAX_MOV_ALIGN_1TIME_RETRY; j++ ) {
      if( millis() - startTime >= ALIGN_TIMEOUT ) { bTimeout = true; break; }
      GetValueFromFrontPSDSensors( &flPSD, &frPSD );
      if( ! RotateAlignWithFront2PSD( dxl, frontAvgTarget - (flPSD + frPSD) / 2, (frPSD - flPSD) - rotationAdjust,
              moveTolerance, rotateTolerance, PSD_MOVE_CONTROL_RATIO, PSD_ROTATE_CONTROL_RATIO, i == 0 && j == 0, MOVE_ALIGN_SPEED ) ) break;
      nAdjusted++;
    }
    if( bTimeout || nAdjusted == 0 ) break;
    delay( ALIGN_OUT_REPEAT_DELAY_MS );
  }
  SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode( dxl );
}

bool ReadArm( int32_t* v )
{
  for( uint8_t i = 0; i < ARM_DXL_ID_CNT; i++ ) {
    v[i] = dxl.readControlTableItem( PRESENT_POSITION, ARM_DXL_IDS[i] );
    if( dxl.getLastLibErrCode() != DXL_LIB_OK ) {
      out.print( F("read fail ID ") );
      out.println( ARM_DXL_IDS[i] );
      return false;
    }
  }
  return true;
}

bool ArmTorqueOn()
{
  return dxl.readControlTableItem( TORQUE_ENABLE, ARM_DXL_IDS[0] ) == TORQUE_ON;
}

void SetArmTorque( bool on )
{
  if( on ) {
    int32_t v[4];
    if( ReadArm( v ) )
      for( uint8_t i = 0; i < ARM_DXL_ID_CNT; i++ ) dxl.writeControlTableItem( GOAL_POSITION, ARM_DXL_IDS[i], v[i] );
  }
  for( uint8_t i = 0; i < ARM_DXL_ID_CNT; i++ ) dxl.writeControlTableItem( TORQUE_ENABLE, ARM_DXL_IDS[i], on ? TORQUE_ON : TORQUE_OFF );
  out.println( on ? F("arm torque ON") : F("arm torque OFF") );
}

void CmdRead()
{
  int32_t v[4];
  if( ! ReadArm( v ) ) return;
  out.print( F("arm ") );
  for( uint8_t i = 0; i < 4; i++ ) { out.print( v[i] ); out.print( i < 3 ? F(", ") : F("\n") ); }
}

bool PlayPose( uint8_t id, int ms )
{
  ManipulatorPose p = ReadManipulatorPresentPoseToEEPROM( id );
  if( p.isTherePoseData != 1 ) {
    out.print( F("no pose ") );
    out.println( id );
    return false;
  }
  if( ! ArmTorqueOn() ) SetArmTorque( true );
  RunManipulatorPoseWithPoseDataInEEPROM( dxl, id, ms, CenterJ1( id ) ? 0.0 : -360.0 );
  delay( ms );
  return true;
}

void CmdSave( int id )
{
  if( id < 1 || id > MANIPULATOR_POSE_ID_MAX_CNT ) { out.println( F("id 1~36") ); return; }
  int32_t v[4];
  if( ! ReadArm( v ) ) return;
  ManipulatorPose p = {};
  p.isTherePoseData = 1;
  p.id = id;
  p.manipulatorMotor1Value = v[0];
  p.manipulatorMotor2Value = v[1];
  p.manipulatorMotor3Value = v[2];
  p.manipulatorMotor4Value = v[3];
  EEPROM.put( MANIPULATOR_POSE_DATA_SIZE * (id - 1), p );
  out.print( F("saved ") );
  out.print( id );
  out.print( ' ' );
  PrintName( id );
  out.print( F(" = ") );
  for( uint8_t i = 0; i < 4; i++ ) { out.print( v[i] ); out.print( i < 3 ? F(", ") : F("\n") ); }
}

void CmdDelete( int id )
{
  if( id < 1 || id > MANIPULATOR_POSE_ID_MAX_CNT ) { out.println( F("id 1~36") ); return; }
  EEPROM.update( MANIPULATOR_POSE_DATA_SIZE * (id - 1), 0 );
  out.print( F("deleted ") );
  out.println( id );
}

void CmdList()
{
  uint8_t missing = 0;
  for( uint8_t id = 1; id <= MANIPULATOR_POSE_ID_MAX_CNT; id++ ) {
    ManipulatorPose p = ReadManipulatorPresentPoseToEEPROM( id );
    if( id < 10 ) out.print( ' ' );
    out.print( id );
    out.print( ' ' );
    PrintName( id );
    out.print( F("\t") );
    if( p.isTherePoseData != 1 ) {
      out.println( '-' );
      if( id != 1 && id != 2 && id != 3 && id != 35 && id != 36 ) missing++;
      continue;
    }
    out.print( p.manipulatorMotor1Value ); out.print( F(", ") );
    out.print( p.manipulatorMotor2Value ); out.print( F(", ") );
    out.print( p.manipulatorMotor3Value ); out.print( F(", ") );
    out.println( p.manipulatorMotor4Value );
  }
  out.print( F("missing (mission) ") );
  out.println( missing );
}

void CmdOurs()
{
  for( uint8_t r = 0; r < 23; r++ ) {
    ManipulatorPose p = {};
    p.isTherePoseData = 1;
    p.id = pgm_read_word( &OURS[r][0] );
    p.manipulatorMotor1Value = pgm_read_word( &OURS[r][1] );
    p.manipulatorMotor2Value = pgm_read_word( &OURS[r][2] );
    p.manipulatorMotor3Value = pgm_read_word( &OURS[r][3] );
    p.manipulatorMotor4Value = pgm_read_word( &OURS[r][4] );
    EEPROM.put( MANIPULATOR_POSE_DATA_SIZE * (p.id - 1), p );
  }
  out.println( F("ours: 23 poses saved (B 19~26 empty)") );
  CmdList();
}

void CmdPsd( int n )
{
  if( n < 1 ) n = 5;
  int16_t fl, fr, sl, sr;
  for( int i = 0; i < n; i++ ) {
    GetValueFromFrontPSDSensors( &fl, &fr );
    GetValueFromSideLeftPSDSensor( &sl );
    GetValueFromSideRightPSDSensor( &sr );
    out.print( F("FL ") ); out.print( fl );
    out.print( F("  FR ") ); out.print( fr );
    out.print( F("  SL ") ); out.print( sl );
    out.print( F("  SR ") ); out.print( sr );
    out.print( F("  avg ") ); out.print( (fl + fr) / 2 );
    out.print( F("  FR-FL ") ); out.println( fr - fl );
    delay( 200 );
  }
}

void CmdCam()
{
  if( ! pixyOk ) { out.println( F("no pixy") ); return; }
  int8_t r = pixy.ccc.getBlocks( true, 0x3F );
  if( r < 0 ) { out.println( F("pixy error") ); return; }
  out.print( F("blocks ") );
  out.print( pixy.ccc.numBlocks );
  out.print( F("  set x ") );
  out.println( PIXY_SET_POINT_X );
  for( int i = 0; i < pixy.ccc.numBlocks; i++ ) {
    int x = pixy.ccc.blocks[i].m_x, y = pixy.ccc.blocks[i].m_y;
    bool in = x >= MIN_PIXY_SEARCH_X && x <= MAX_PIXY_SEARCH_X && y >= MIN_PIXY_SEARCH_Y && y <= MAX_PIXY_SEARCH_Y;
    out.print( F(" sig ") ); out.print( pixy.ccc.blocks[i].m_signature );
    out.print( F(" x ") ); out.print( x );
    out.print( F(" y ") ); out.print( y );
    out.print( F(" w ") ); out.print( pixy.ccc.blocks[i].m_width );
    out.print( F(" h ") ); out.print( pixy.ccc.blocks[i].m_height );
    out.print( F(" dx ") ); out.print( x - PIXY_SET_POINT_X );
    out.print( F(" mm ") ); out.print( (x - PIXY_SET_POINT_X) * 0.30f, 1 );
    out.println( in ? F(" IN") : F(" out") );
  }
}

void CmdPick( int layer )
{
  if( layer < 1 || layer > 3 ) { out.println( F("k 1~3") ); return; }
  int k = layer - 1;
  if( ! ArmTorqueOn() ) SetArmTorque( true );
  OpenGripper( pixy );
  delay( GRIP_TIME );
  if( ! PlayPose( APP_ID[k], PICK_TIME ) ) return;
  if( ! PlayPose( GRIP_ID[k], PICK_TIME ) ) return;
  CloseGripper( pixy );
  delay( GRIP_TIME );
  if( ! PlayPose( OUT_ID[k], PICK_TIME ) ) return;
  PlayPose( SEARCH_POSE, PICK_TIME );
  out.println( F("pick done") );
}

void CmdDrop( int cell )
{
  if( cell < 1 || cell > 16 ) { out.println( F("d 1~16 (A 1~8, B 9~16)") ); return; }
  bool b = cell > 8;
  uint8_t id = (b ? 18 : 10) + (b ? cell - 8 : cell);
  int t = 500;
  if( ! b ) {
    if( cell == 1 || cell == 4 ) t += 200;
    else if( cell == 5 || cell == 8 ) t += 100;
  }
  if( ! ArmTorqueOn() ) SetArmTorque( true );
  ManipulatorPose p = RunManipulatorPoseWithPoseDataInEEPROM( dxl, id, t );
  if( p.isTherePoseData != 1 ) { out.print( F("no pose ") ); out.println( id ); return; }
  delay( t );
  OpenGripper( pixy );
  delay( GRIP_TIME / 2 );
  if( ! b ) {
    RunManipulatorPoseWithPoseDataInEEPROM( dxl, SEARCH_POSE, 500, -90.0 );
    delay( 500 );
  }
  RunManipulatorPoseWithPoseDataInEEPROM( dxl, SEARCH_POSE, 500, 0.0 );
  delay( 500 );
  out.println( F("drop done") );
}

void PrintRates()
{
  const char* nm[4] = { "wf", "wb", "wl", "wr" };
  for( uint8_t d = 0; d < 4; d++ ) {
    out.print( nm[d] );
    for( uint8_t i = 0; i < 4; i++ ) { out.print( ' ' ); out.print( wheelRate[d][i], 3 ); }
    out.println();
  }
}

void CmdId()
{
  uint8_t ids[16];
  uint8_t n = dxl.ping( DXL_BROADCAST_ID, ids, sizeof(ids), 100 );
  out.print( F("motors @") );
  out.print( DXL_BAUDRATE );
  out.print( F(": ") );
  for( uint8_t i = 0; i < n; i++ ) { out.print( ids[i] ); out.print( ' ' ); }
  out.println();
}

void CmdBaud()
{
  dxl.begin( 57600 );
  dxl.setPortProtocolVersion( DXL_PROTOCOL_VERSION );
  uint8_t ids[16];
  uint8_t n = dxl.ping( DXL_BROADCAST_ID, ids, sizeof(ids), 100 );
  out.print( F("57600 found ") );
  out.println( n );
  for( uint8_t i = 0; i < n; i++ ) {
    dxl.torqueOff( ids[i] );
    dxl.writeControlTableItem( BAUD_RATE, ids[i], BAUD_RATE_1M );
  }
  delay( 100 );
  InitMotorCommunication( dxl );
  CmdId();
  out.println( F("reset board after baud change") );
}

void Help()
{
  out.println( F("r  arm values          t  torque on/off") );
  out.println( F("p N play pose          s N save pose  x N delete") );
  out.println( F("l  pose list           o/c gripper    g N grip raw") );
  out.println( F("psd [n]  cam") );
  out.println( F("f/b/lf/rt N mm         rl/rr [deg] 90 turn") );
  out.println( F("fa T [tol] FR          sa T [tol] SL   ra T [tol] SR") );
  out.println( F("ma T adj [mt rt] FL+FR avg & FR-FL") );
  out.println( F("k N pick layer         d N drop cell (B=9~16)") );
  out.println( F("w  ratios  wf/wb/wl/wr a b c d   id   baud") );
  out.println( F("ours  save our v27 arm poses to EEPROM") );
}

void Handle( char* line )
{
  char* cmd = strtok( line, " \t" );
  if( ! cmd ) return;
  char* s1 = strtok( NULL, " \t" );
  char* s2 = strtok( NULL, " \t" );
  char* s3 = strtok( NULL, " \t" );
  char* s4 = strtok( NULL, " \t" );
  float a1 = s1 ? atof( s1 ) : 0, a2 = s2 ? atof( s2 ) : 0, a3 = s3 ? atof( s3 ) : 0, a4 = s4 ? atof( s4 ) : 0;
  int i1 = (int) a1;

  if( ! strcmp( cmd, "h" ) ) Help();
  else if( ! strcmp( cmd, "r" ) ) CmdRead();
  else if( ! strcmp( cmd, "t" ) ) SetArmTorque( ! ArmTorqueOn() );
  else if( ! strcmp( cmd, "p" ) ) { if( i1 >= 1 && i1 <= 36 && PlayPose( i1, POSE_TIME ) ) { out.print( F("pose ") ); out.println( i1 ); } }
  else if( ! strcmp( cmd, "s" ) ) CmdSave( i1 );
  else if( ! strcmp( cmd, "x" ) ) CmdDelete( i1 );
  else if( ! strcmp( cmd, "l" ) ) CmdList();
  else if( ! strcmp( cmd, "ours" ) ) CmdOurs();
  else if( ! strcmp( cmd, "o" ) ) { OpenGripper( pixy ); out.println( F("open") ); }
  else if( ! strcmp( cmd, "c" ) ) { CloseGripper( pixy ); out.println( F("close") ); }
  else if( ! strcmp( cmd, "g" ) ) { pixy.setServos( 0, constrain( i1, 0, 1000 ) ); out.print( F("grip ") ); out.println( constrain( i1, 0, 1000 ) ); }
  else if( ! strcmp( cmd, "psd" ) ) CmdPsd( i1 );
  else if( ! strcmp( cmd, "cam" ) ) CmdCam();
  else if( ! strcmp( cmd, "f" ) ) DriveDistanceSmoothWithInternalAcceleration( dxl, a1, MOVE_FORWARD, MOVE_SPEED );
  else if( ! strcmp( cmd, "b" ) ) DriveDistanceSmoothWithInternalAcceleration( dxl, a1, MOVE_BACKWARD, MOVE_SPEED );
  else if( ! strcmp( cmd, "lf" ) ) DriveDistanceSmoothWithInternalAcceleration( dxl, a1, MOVE_LEFTWARD, MOVE_SPEED );
  else if( ! strcmp( cmd, "rt" ) ) DriveDistanceSmoothWithInternalAcceleration( dxl, a1, MOVE_RIGHTWARD, MOVE_SPEED );
  else if( ! strcmp( cmd, "rl" ) ) RotateLeft90Degree( dxl, MOVE_SPEED, a1 );
  else if( ! strcmp( cmd, "rr" ) ) RotateRight90Degree( dxl, MOVE_SPEED, a1 );
  else if( ! strcmp( cmd, "fa" ) ) PerformFrontMoveAlign( i1, s2 ? (int) a2 : 2 );
  else if( ! strcmp( cmd, "sa" ) ) PerformLeftMoveAlign( i1, s2 ? (int) a2 : 3 );
  else if( ! strcmp( cmd, "ra" ) ) PerformRightMoveAlign( i1, s2 ? (int) a2 : 2 );
  else if( ! strcmp( cmd, "ma" ) ) PerformMoveRotateAlign( i1, s3 ? (int) a3 : 2, s4 ? (int) a4 : 2, (int) a2 );
  else if( ! strcmp( cmd, "k" ) ) CmdPick( i1 );
  else if( ! strcmp( cmd, "d" ) ) CmdDrop( i1 );
  else if( ! strcmp( cmd, "w" ) ) PrintRates();
  else if( cmd[0] == 'w' && cmd[2] == 0 && strchr( "fblr", cmd[1] ) ) {
    uint8_t d = strchr( "fblr", cmd[1] ) - "fblr";
    if( s4 ) { wheelRate[d][0] = a1; wheelRate[d][1] = a2; wheelRate[d][2] = a3; wheelRate[d][3] = a4; }
    PrintRates();
  }
  else if( ! strcmp( cmd, "id" ) ) CmdId();
  else if( ! strcmp( cmd, "baud" ) ) CmdBaud();
  else { out.print( F("? ") ); out.println( cmd ); return; }

  if( ! strcmp( cmd, "fa" ) || ! strcmp( cmd, "sa" ) || ! strcmp( cmd, "ra" ) || ! strcmp( cmd, "ma" ) ) CmdPsd( 1 );
}

char lineBuf[40];
uint8_t lineLen = 0;

void Feed( Stream& s )
{
  while( s.available() ) {
    char ch = s.read();
    if( ch == '\n' || ch == '\r' ) {
      if( lineLen ) { lineBuf[lineLen] = 0; lineLen = 0; Handle( lineBuf ); }
    } else if( lineLen < sizeof(lineBuf) - 1 ) {
      lineBuf[lineLen++] = ch;
    }
  }
}

void setup()
{
  InitDebug();
  Serial2.begin( 115200 );
  pinMode( SW1_PIN, INPUT );
  pinMode( SW2_PIN, INPUT );

  InitMotorCommunication( dxl );
  while( ! InitManipulator( dxl ) ) { out.println( F("arm ID 5~8 not found") ); delay( 1000 ); }
  while( ! InitMobilebase( dxl ) ) { out.println( F("wheel ID 1~4 not found") ); delay( 1000 ); }
  InitPSD();
  pixyOk = pixy.init() >= 0;
  if( pixyOk ) {
    pixy.setLamp( 0, 0 );
    OpenGripper( pixy );
  }
  SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode( dxl );

  out.println( F("< T3 Tool >") );
  Help();
}

void loop()
{
  Feed( Serial );
  Feed( Serial2 );
  if( digitalRead( SW1_PIN ) == LOW ) { CmdRead(); delay( 400 ); }
  if( digitalRead( SW2_PIN ) == LOW ) { SetArmTorque( ! ArmTorqueOn() ); delay( 400 ); }
}
