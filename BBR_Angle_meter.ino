#include <Filter.h>
#include <medianFilter.h>
#include <Arduino.h>                  // Arduino standard library
#include <U8g2lib.h>                  // Oled U8g2 library
#include <SparkFun_ADXL345.h>         // SparkFun ADXL345 Library

int action = 2;
double corde,  ref_angle;
const float pi = 3.1415926539;        // PI definition.... could have been a standard value....
const int buttonPin = 2;              // the input pin number of the pushbutton

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);   // pin remapping with ESP8266 HW I2C

medianFilter Filter;
//----------------------------------------------------------------------------------------------------------------------------------
ADXL345 adxl = ADXL345();                         // USE ADXL345 I2C COMMUNICATION

//----------------------------------------------------------------------------------------------------------------------------------
double read_angle() {                             // Function returning the current rotation value along X axis - in degrees
  //----------------------------------------------------------------------------------------------------------------------------------
  int x, y, z;
  double l_angle = 0;
  double ll_angle = 0;
  int mm = 800;

  for (int nn = 1;  nn <= mm; nn++) {               // Average value computed over roughly 100ms
    adxl.readAccel(&x, &y, &z);                     // Read the accelerometer values and store them in variables declared above x,y,z
    l_angle = atan2(y, z) * 57.3;                 // Compute rotation angle along X axis of ADXL345
    //  Serial.println("Angle " + String(l_angle));
    l_angle = Filter.run(l_angle * 1000);
    ll_angle = ll_angle + (l_angle / 1000);
  }
  //  Serial.println("Angle " + String(ll_angle / mm));
  return ll_angle / mm;
}
//----------------------------------------------------------------------------------------------------------------------------------
void init_angle() {
  //----------------------------------------------------------------------------------------------------------------------------------
  double nnnn = 0, ra = 0;
  do {
    nnnn = read_angle();
    ra = read_angle();                      // Initialize the actual angle as the reference angle
    //   Serial.print("init angle: "); Serial.print(abs(int(nnnn*100) - int(ref_angle*100))); Serial.print(" nn:"); Serial.print(int(nn*100)); Serial.print(" ref"); Serial.print(int(ref_angle*100));  Serial.println("");
  }
  while (abs(int(nnnn * 100) - int(ra * 100)) > 5); // Wait until reading is stable
  ref_angle = ra;
}
//----------------------------------------------------------------------------------------------------------------------------------
void setup() {
  //----------------------------------------------------------------------------------------------------------------------------------
  corde = 25;                                     // Chord width value
  Serial.begin(9600);
  // Serial.println("Init done");
  u8g2.begin();                                   // Start Oled
  affiche_init();
  Filter.begin();                                 // Start filtering
  adxl.powerOn();                                 // Power on the ADXL345
  adxl.setSpiBit(0);                              // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
  adxl.setRangeSetting(2);                        // Give the range settings
  adxl.setTapDetectionOnXYZ(0, 0, 1);             // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)
  // Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
  adxl.setTapThreshold(40);                       // 62.5 mg per increment was 50
  adxl.setTapDuration(15);                        // 625 μs per increment
  adxl.setDoubleTapLatency(40);                   // 1.25 ms per increment was 80
  adxl.setDoubleTapWindow(200);                   // 1.25 ms per increment was 200
  adxl.doubleTapINT(1);
  adxl.singleTapINT(1);
  pinMode(buttonPin, INPUT);                      // define the Push Button input
}

//----------------------------------------------------------------------------------------------------------------------------------
String cnv_flt2str(float num, int car, int digit) { // Convert a float variable into a string with a specific number of digits
  //----------------------------------------------------------------------------------------------------------------------------------
  float tmp_num, expo;
  String str = "";
  switch (digit) {
    case 0: expo = 1; break;
    case 1: expo = 10; break;
    case 2: expo = 100; break;
    case 3: expo = 1000; break;
    case 4: expo = 10000; break;
    default: expo = 0; break;
  }
  if (expo != 0) {
    str = String(int(num));
    if (expo > 1)  {
      tmp_num = abs(num) - int(abs(num));
      str = str + "." + String(int(tmp_num * expo));
    }
  }
  while (str.length() < car) {
    str = " " + str;
  }
  return str;
}
bool bp_pushed() {
  return (digitalRead(buttonPin) == HIGH);
}
//----------------------------------------------------------------------------------------------------------------------------------
void aff_menu() {
  //----------------------------------------------------------------------------------------------------------------------------------
  int action = 0, act = 0, pas = 1;

  while (bp_pushed()) {
    delay(1);
  }
  do {
    u8g2.firstPage();                                                 // Display values
    do {
      u8g2.setFontDirection(0);
      u8g2.setFont(u8g2_font_t0_14_tf);
      u8g2.setCursor(18, 24);
      u8g2.print("Corde : " + cnv_flt2str(corde, 4, 1) + "mm");
    } while ( u8g2.nextPage() );

    act = action_acc();                                             // read tap
    if (act != 0) {
      corde = corde + (act * pas);
    }
  } while (not(bp_pushed()));

}
//----------------------------------------------------------------------------------------------------------------------------------
void affiche_init() {
  //----------------------------------------------------------------------------------------------------------------------------------
  u8g2.firstPage();                                                 // Display values
  do {
    u8g2.setFontDirection(0);
    u8g2.setFont(u8g2_font_t0_14_tf);
    u8g2.setCursor(18, 24);
    u8g2.print("INIT EN COURS");
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
    u8g2.print("Angle deg -->");
    u8g2.setFont(u8g2_font_crox4tb_tn);
    u8g2.setCursor(78, 15);
    u8g2.print(l_angle);
    u8g2.setFont(u8g2_font_t0_11_tf);
    u8g2.setCursor(1, 21);
    u8g2.print("Corde " + l_corde + "mm");
    u8g2.setCursor(1, 31);
    u8g2.print("Debat mm --->");
    u8g2.setFont(u8g2_font_crox4tb_tn);
    u8g2.setCursor(78, 32);
    u8g2.print(l_debat);
  } while ( u8g2.nextPage() );
}
//----------------------------------------------------------------------------------------------------------------------------------
int action_acc() {                          // TAP reading on accelerometer
  //----------------------------------------------------------------------------------------------------------------------------------
  int act = 0;
  byte interrupts = adxl.getInterruptSource();

  if (adxl.triggered(interrupts, ADXL345_SINGLE_TAP)) {             // Tap Detection
    act = -1;                                                       // If single TAP action = -1
  }
  if (adxl.triggered(interrupts, ADXL345_DOUBLE_TAP)) {             // Double Tap Detection
    act = 1;                                                        // If double TAP action = 1
  }

  return act;
}
//----------------------------------------------------------------------------------------------------------------------------------
void loop() {                                     // Main loop
  //----------------------------------------------------------------------------------------------------------------------------------
  float x_rot = 0, aff_angle = 0, angle = 0, debat = 0;
  int act = 0;

  if (bp_pushed()) action = 1;
  switch (action) {
    case (1):
      aff_menu();
      action = 2;
      break;
    case (2):
      init_angle();
      action = 0;
      break;
    default:
      break;
  }

  x_rot = read_angle();                                               // read current angle
  x_rot = ref_angle - x_rot;                                          // compute angle variation vs. reference angle
  angle = (x_rot / 180) * pi;                                         // angle value converted into radian
  debat = sqrt(2 * sq(corde) - (2 * sq(corde) * cos(angle)));         // throw computation in same units as chord
  affiche(cnv_flt2str(x_rot, 6, 1), cnv_flt2str(corde, 4, 1), cnv_flt2str(debat, 6, 1));



}
