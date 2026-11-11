#include <EEPROM.h>

struct __attribute__((packed)) Pose {
  uint8_t valid;
  uint8_t id;
  int16_t j[4];
  char desc[30];
};

const int POSE_COUNT = 36;
char line[64];
uint8_t len = 0;

void Dump()
{
  Serial.println( F("=== POSES (copy and keep these lines) ===") );
  for( int n = 1; n <= POSE_COUNT; n++ ) {
    Pose p;
    EEPROM.get( 40 * ( n - 1 ), p );
    if( p.valid == 1 && p.id == n ) {
      Serial.print( F("W ") ); Serial.print( n );
      for( int k = 0; k < 4; k++ ) { Serial.print( ' ' ); Serial.print( p.j[k] ); }
      Serial.println();
    } else {
      Serial.print( F("# ") ); Serial.print( n ); Serial.println( F(" empty") );
    }
  }
  Serial.println( F("=== RAW 0..1439 ===") );
  for( int a = 0; a < 40 * POSE_COUNT; a++ ) {
    uint8_t b = EEPROM.read( a );
    if( b < 16 ) Serial.print( '0' );
    Serial.print( b, HEX );
    if( a % 40 == 39 ) Serial.println();
  }
  Serial.println( F("=== END ===  d = dump again | W id j1 j2 j3 j4 = write one pose") );
}

void Write( char* s )
{
  int v[5];
  int n = 0;
  for( char* t = strtok( s, " " ); t && n < 5; t = strtok( nullptr, " " ) ) v[n++] = atoi( t );
  if( n != 5 || v[0] < 1 || v[0] > POSE_COUNT ) { Serial.println( F("format: W id j1 j2 j3 j4") ); return; }
  Pose p;
  memset( &p, 0, sizeof(p) );
  p.valid = 1;
  p.id = v[0];
  for( int k = 0; k < 4; k++ ) p.j[k] = v[k + 1];
  EEPROM.put( 40 * ( v[0] - 1 ), p );
  Serial.print( F("written pose ") ); Serial.println( v[0] );
}

void setup()
{
  Serial.begin( 115200 );
  delay( 500 );
  Dump();
}

void loop()
{
  while( Serial.available() ) {
    char c = Serial.read();
    if( c == '\n' || c == '\r' ) {
      line[len] = 0;
      if( len ) {
        if( line[0] == 'd' ) Dump();
        else if( line[0] == 'W' ) Write( line + 1 );
      }
      len = 0;
    } else if( len < sizeof(line) - 1 ) line[len++] = c;
  }
}
