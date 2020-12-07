/*

   Copyright J'm f5mmx, France, 2020 (jmb91650@gmail.com)
   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   This is a beerware; if you like it and if we meet some day, you can pay me a beer in return!
   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   --------------------------------------------------
   the MPU6050 version.
   Have fun!
   --------------------------------------------------
   V1.0 Nov. 2020: initial release
   --------------------------------------------------
*/

#include <MPU6050.h>                  // MPU6050 libray by Jarzebski from https://github.com/jarzebski/Arduino-MPU6050/archive/master.zip
#include <U8g2lib.h>                  // Oled U8g2 library           from https://github.com/olikraus/U8g2_Arduino/archive/master.zip
#include "RunningMedian.h"            // By Ron Tillaart             from https://github.com/RobTillaart/Arduino/tree/master/libraries/RunningMedian
#include <movingAvg.h>                // By J Christensen            from https://github.com/JChristensen/movingAvg
#define DELAY_DEBOUNCE 10             // Debounce delay for Push Button
#define DELAY_START_INIT 1000         // delay to start calibration when PB is pressed 
RunningMedian R_Angle = RunningMedian(5);
movingAvg ll_angle(10);
//----------------------------------------------------------------------------------------------------------------------------------
// Menu text
const String Txt1 = "Erreur MPU";     // MPU6050 sensor didn't start error string
const String Txt2 = "Corde : ";       // Chord
const String Txt3 = "INIT EN COURS";  // Initializing
const String Txt4 = "Angle deg -->";  // Angle value
const String Txt5 = "Corde ";         // Chord
const String Txt6 = "Debat mm --->";  // Throw value milimiters
int action = 0;
bool app = false;
float corde,  ref_angle;
const float pi = M_PI;
const int buttonPin = 2;
volatile int bp_pushed = 0;
volatile  unsigned long bp_down = 0 ;
volatile  unsigned long bp_up = 0;
volatile  unsigned long bp_ = 0;
long lastDebounceTimePB_INIT = 0;
boolean lastStatePB_INIT = HIGH;
boolean triggerPB_INIT = 0;
boolean buttonStatePB_INIT = HIGH;
boolean readingPB_INIT = HIGH;
int maxx = -10000, minx = 10000, maxy = -10000, miny = 10000, maxz = -10000, minz = 10000;
//----------------------------------------------------------------------------------------------------------------------------------
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);   // pin remapping with ESP8266 HW I2C
//----------------------------------------------------------------------------------------------------------------------------------
MPU6050 mma;                                      // could have been called MPU :D
//----------------------------------------------------------------------------------------------------------------------------------
float read_angle() {                             // Function returning the current rotation value along Y axis - in degrees was double
  //----------------------------------------------------------------------------------------------------------------------------------
  float l_angle = 0;
  double lll_angle = 0;
  float roll = 0;
  int mm = 19 * 3  , mmm = 3;
  for (int nnn = 1;  nnn <= mmm; nnn++) {             // Average value computed over a few 100ms
    for (int nn = 1;  nn <= mm; nn++) {               // Running median fill running median
      Vector norm = mma.readNormalizeAccel();
      roll = atan2(norm.XAxis, norm.ZAxis) * 180 / pi;
      R_Angle.add(roll);                              // Add value in running median table
    }
    lll_angle = ll_angle.reading(R_Angle.getMedian() * 100); //Add value in Moving average object
  }
  l_angle = round(lll_angle / 10);
  lll_angle = l_angle / 10;
  return lll_angle;
}
//----------------------------------------------------------------------------------------------------------------------------------
void init_angle() {                                // Initialize the actual angle as the reference angle
  //----------------------------------------------------------------------------------------------------------------------------------
  float ra = 0;
  mma.calibrateGyro();
  mma.setThreshold(3);
  delay(200);
  R_Angle.clear();
  ll_angle.reset();
  ra = read_angle();
  ref_angle = ra;
}
//----------------------------------------------------------------------------------------------------------------------------------
void setup() {
  //----------------------------------------------------------------------------------------------------------------------------------
  delay(300);
  ll_angle.begin();
  corde = 68;                                     // Chord width value
  Serial.begin(9600);
  //  Serial.println("Init done");
  u8g2.begin(); // Start Oled
  delay(200);
  if (! mma.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G, 0x68)) {   // Try to start MMA, if fails then display error message
    u8g2.firstPage();
    do {
      u8g2.setFontDirection(0);
      u8g2.setFont(u8g2_font_t0_14_tf);
      u8g2.setCursor(18, 15);
      u8g2.print(Txt1);
    } while ( u8g2.nextPage() );
    while (1);
  }
  mma.calibrateGyro();
  mma.setThreshold(2);
  pinMode(buttonPin, INPUT_PULLUP);                      // define the Push Button input
  init_angle();
}
//----------------------------------------------------------------------------------------------------------------------------------
void lecture_bp() { // Push button read function with debouncing and long push detection
  //----------------------------------------------------------------------------------------------------------------------------------
  int  lbp = 0;
  readingPB_INIT = !digitalRead(buttonPin);
  triggerPB_INIT = 0;
  if (readingPB_INIT != lastStatePB_INIT) lastDebounceTimePB_INIT = millis();
  if ((millis() - lastDebounceTimePB_INIT) > DELAY_DEBOUNCE)
  {
    if (readingPB_INIT != buttonStatePB_INIT)
    {
      buttonStatePB_INIT = readingPB_INIT;
      if (buttonStatePB_INIT == LOW)

        if ((millis() - lastDebounceTimePB_INIT) < DELAY_START_INIT)
        {
          lbp = 2;
          triggerPB_INIT = 1;
        }
    }
  }
  if (((millis() - lastDebounceTimePB_INIT) >= DELAY_START_INIT) and (buttonStatePB_INIT == HIGH))  lbp = 1;
  lastStatePB_INIT = readingPB_INIT;
  action = lbp;
}
//----------------------------------------------------------------------------------------------------------------------------------
String cnv_flt2str(float num, int car, int digit) { // Convert a float variable into a string with a specific number of digits
  //----------------------------------------------------------------------------------------------------------------------------------
  String str = "";
  if (digit > 0) {
    str = String(num, digit);
  } else {
    str = String(int(num));
  }
  while (str.length() < car) {
    str = " " + str;
  }
  return str;
}
//----------------------------------------------------------------------------------------------------------------------------------
void aff_menu() {
  //----------------------------------------------------------------------------------------------------------------------------------
  int action = 0,  l_pas = 1;
  float l_ang = 0, l_act = 0, l_posi = 0, l_ref = 0;
  u8g2.firstPage();                                                 // Display values
  do {
    u8g2.setFontDirection(0);
    u8g2.setFont(u8g2_font_t0_14_tf);
    u8g2.setCursor(18, 15);
    u8g2.print(Txt2 + cnv_flt2str(corde, 4, 1) + "mm");
    u8g2.drawBox(64 - 7, 22, 14, 10);
    u8g2.drawFrame(64 - 20, 22, 40, 10);
    u8g2.drawFrame(64 - 45, 22, 90, 10);
    u8g2.drawFrame(0, 22, 128, 10);
    u8g2.setFont(u8g2_font_micro_tr);
    u8g2.setCursor(64 - 1, 30);
    u8g2.print("0");
    u8g2.setCursor(64 - 19, 30);
    u8g2.print("-.1");
    u8g2.setCursor(64 - 45 + 12, 30);
    u8g2.print("-1");
    u8g2.setCursor(64 - 63 + 5, 30);
    u8g2.print("-10");
    u8g2.setCursor(64 + 19 - 9, 30);
    u8g2.print(".1");
    u8g2.setCursor(64 + 45 - 14, 30);
    u8g2.print("1");
    u8g2.setCursor(64 + 63 - 14, 30);
    u8g2.print("10");
  } while ( u8g2.nextPage() );
  do {
    delay(10);
    lecture_bp();
  } while (triggerPB_INIT != 1);
  delay(50);
  l_ref = read_angle();
  do {
    l_ang = read_angle() - l_ref;                                       // read current angle
    if (abs(l_ang) >= 7) {                                              // If angle over 7 degrees
      if (abs(l_ang) < 20) {
        l_act = 0.1;
      }
      else {
        if (abs(l_ang) < 45) l_act = 1;
        else {
          l_act = 10;
        }
      }
      if (l_ang > 0) l_pas = 1; else l_pas = -1;
    }
    else l_act = 0;
    //  Serial.println("angle :" + String(abs(ang)) + " act " + String(act));
    if (l_act != 0) {
      corde = corde + (l_act * l_pas);
    }
    l_posi = l_ang;

    if (l_posi >= 64) {
      l_posi = 64;
    }
    else {
      if (l_posi < -64) {
        l_posi = -64;
      }
    }
    l_pas = 64 + l_posi;
    u8g2.firstPage();                                                 // Display values
    do {
      u8g2.setFontDirection(0);
      u8g2.setFont(u8g2_font_t0_14_tf);
      u8g2.setCursor(18, 15);
      u8g2.print(Txt2 + cnv_flt2str(corde, 4, 1) + "mm");
      u8g2.drawBox(64 - 7, 22, 14, 10);
      u8g2.drawFrame(64 - 20, 22, 40, 10);
      u8g2.drawFrame(64 - 45, 22, 90, 10);
      u8g2.drawFrame(0, 22, 128, 10);
      u8g2.setFont(u8g2_font_micro_tr);
      u8g2.setCursor(64 - 1, 30);
      u8g2.print("0");
      u8g2.setCursor(64 - 19, 30);
      u8g2.print("-.1");
      u8g2.setCursor(64 - 45 + 12, 30);
      u8g2.print("-1");
      u8g2.setCursor(64 - 63 + 5, 30);
      u8g2.print("-10");
      u8g2.setCursor(64 + 19 - 9, 30);
      u8g2.print(".1");
      u8g2.setCursor(64 + 45 - 14, 30);
      u8g2.print("1");
      u8g2.setCursor(64 + 63 - 14, 30);
      u8g2.print("10");
      u8g2.drawLine(l_pas, 18, l_pas, 34);
      u8g2.drawLine(l_pas - 1, 18, l_pas - 1, 20);
      u8g2.drawLine(l_pas + 1, 18, l_pas + 1, 20);
    } while ( u8g2.nextPage() );
    delay(20);
    lecture_bp();
    //   Serial.println(triggerPB_INIT);
  } while (triggerPB_INIT != 1);
  action = 0;
}
//----------------------------------------------------------------------------------------------------------------------------------
void affiche_init() {
  //----------------------------------------------------------------------------------------------------------------------------------
  u8g2.firstPage();                                                 // Display values
  do {
    u8g2.setFontDirection(0);
    u8g2.setFont(u8g2_font_t0_14_tf);
    u8g2.setCursor(18, 24);
    u8g2.print(Txt3);
  } while ( u8g2.nextPage() );
}
//----------------------------------------------------------------------------------------------------------------------------------
void affiche(String l_angle, String l_corde, String l_debat) {
  //----------------------------------------------------------------------------------------------------------------------------------
  u8g2.firstPage();                                                 // Display values
  do {
    u8g2.setFontDirection(0);
    u8g2.setFont(u8g2_font_t0_11_tf);
    u8g2.setCursor(1, 10);
    u8g2.print(Txt4);
    u8g2.setFont(u8g2_font_crox4tb_tn);
    u8g2.setCursor(78, 15);
    u8g2.print(l_angle);
    u8g2.setFont(u8g2_font_t0_11_tf);
    u8g2.setCursor(1, 21);
    u8g2.print(Txt5 + l_corde + "mm");
    u8g2.setCursor(1, 31);
    u8g2.print(Txt6);
    u8g2.setFont(u8g2_font_crox4tb_tn);
    u8g2.setCursor(78, 32);
    u8g2.print(l_debat);
  } while ( u8g2.nextPage() );
}
//----------------------------------------------------------------------------------------------------------------------------------
void loop() {                                     // Main loop
  //----------------------------------------------------------------------------------------------------------------------------------
  float x_rot = 0, aff_angle = 0, angle = 0, debat = 0;
  int act = 0;
  lecture_bp();
  switch (action) {
    case (1):
      aff_menu();
      affiche_init();
      init_angle();
      action = 2;
      break;
    case (2):
      affiche_init();
      init_angle();
      action = 0;
      break;
    default:
      break;
  }
  x_rot = read_angle();                                                 // read current angle
  x_rot = fmod((ref_angle - x_rot) + 180, 360);                         // compute angle variation vs. reference angle
  if (x_rot < 0) {
    x_rot += 180;
  } else {
    x_rot -= 180;
  }
  angle = (x_rot / 180) * pi;                                         // angle value converted into radian
  // Serial.println("angle :" + String(angle));
  debat = corde * sin(angle);                                         // throw computation in same units as chord
  affiche(cnv_flt2str(x_rot, 6, 1), cnv_flt2str(corde, 4, 1), cnv_flt2str(debat, 6, 1));
}
