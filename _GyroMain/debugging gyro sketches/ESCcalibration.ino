#include <SoftwareSerial.h>
#include <Arduino.h>

/*
   realized by Vincenzo G.
   for a lot of HK esc, check it:
   http://www.hobbyking.com/hobbyking/store/__17204__HobbyKing_ESC_Programming_Card.html
*/

// select which setting do you wish to upload
// choose only a define for every group
// default settings:


#define brake_off
//#define brake_on

//#define battType_lipo
#define battType_NiMH

#define timing_auto
//#define timing_7-22
//#define timing_22-30

#define rotation_normal
//#define rotation_reversed

//#define protection_reduced
#define protection_stop

//#define battProtection_3V
//#define battProtection_2_8V
#define battProtection_3_2V

//#define start_soft
#define start_ACC
//#define start_verySoft


void setup() {

  Serial1.begin(4800);

  delay(5000);

  program();

}

void loop() {}


void program () {
  init_esc();
  set_battType();
  set_protection();
  Serial1.write(248);
  set_battProtection();
  Serial1.write(128);
  set_rotation();
  set_timing();
  Serial1.write(248);
  set_start();
  Serial1.write(120);
  set_brake();
  end_esc();
}


void init_esc() {
  Serial1.write ((byte)0);
  Serial1.write (248);
  Serial1.write (248);
  Serial1.write (120);
  Serial1.write (248);
  Serial1.write (120);
  Serial1.write (248);
}

void set_battType() {
  int battType;
  int battType2;

#ifdef battType_lipo
  battType = 120;
  battType2 = 128;
#else battType_NiMH
  battType = 0;
  battType2 = 0;
#endif

  Serial1.write(battType);
  Serial1.write(128);
  if (battType2 != 0)  Serial1.write(battType2);
}

void set_protection() {
  int protection;

#ifdef protection_reduced
  protection = 120;
#else protection_stop
  protection = 0;
#endif

  Serial1.write(protection);
}

void set_battProtection() {
  int battProtection;
  int battProtection2;

#ifdef battProtection_3V
  battProtection = 248;
  battProtection2 = 248;
#endif

#ifdef battProtection_2_8V
  battProtection = 120;
  battProtection2 = 248;
#endif

#ifdef battProtection_3_2V
  battProtection = 120;
  battProtection2 = 0;
#endif

  Serial1.write(battProtection);
  if (battProtection2 == 248) Serial1.write(battProtection2);
}

void set_rotation() {
  int rotation;

#ifdef rotation_normal
  rotation = 0;
#else rotation_reversed
  rotation = 120;
#endif

  Serial1.write(rotation);
}

void set_timing() {
  int timing;

#ifdef timing_auto
  timing = 128;
#endif

#ifdef timing_7-22
  timing = 0;
#endif

#ifdef timing_22-30
  timing = 120;
#endif

  Serial1.write(timing);
  if (timing == 128) Serial1.write((byte)0);
}

void set_start() {
  int start;
  int start2;

#ifdef start_soft
  start = 120;
  start2 = 0;
#endif

#ifdef start_ACC
  start = 128;
  start2 = 128;
#endif

#ifdef start_verySoft
  start = 0;
  start2 = 0;
#endif

  Serial1.write(start);
  if (start2 != 0)  Serial1.write(start2);
}

void set_brake() {
  int brake;

#ifdef brake_off
  brake = 0;
#else brake_on
  brake = 120;
#endif

  Serial1.write(brake);
}

void end_esc() {
  Serial1.write (248);
  Serial1.write (128);
  Serial1.write (248);
  Serial1.write ((byte)0);
}