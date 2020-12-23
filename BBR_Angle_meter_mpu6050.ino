/*
   Copyright J'm f5mmx, France, 2020 (jmb91650@gmail.com)
*/
//   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//   This is a beerware; if you like it and if we meet some day, you can pay me a beer in return!
//   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
   --------------------------------------------------
   the MPU6050 version ++ HAVE FUN ++
   --------------------------------------------------
   V1.0 Dec. 2020: initial release, wasn't great....
   V2.0 Dec. 2020: complete redesign using new libraries for the MPU6050 and a new large colour display
                   highly inspired from https://github.com/adesandr/MAD_Single project
   --------------------------------------------------
   ARDUINO : Nano
   DISPLAY : ST7789 240X240 IPS colour display with SPI wired with 
                 GND > GND
                 VCC > 3.3V
                 SCL > D13
                 SDA > D11
                 RES > D8
                 CS  > D7
                 BLK > left unconnected
   ACCEL   : MPU6050 6 axis Gyro and Accelerometer GY-521 wire on I2C with
                 VCC > 5V
                 GND > GND
                 SLC >
                 SDA >
   PUSH But: Momentary switch connected between D2 and GND the button is used as follows
        BUTTON PRESSED during boot -> start sensor calibration process. The sensor need to be placed flat 
                                      on a horizontal table and not moved during all the calibration 
                                      process that can take a few minutes to finalize. Calibration data are
                                      stored into Arduino nano EEPROM so a new calibration is not needed each
                                      time the ThrowMeter is powered OFF and ON again.
        Normal operation after full boot or after calibration:
        Button simple click        -> Chord value decreassed
        Button double click        -> Chord value increased
        Button long time pressed   -> Set current sensor position as reference angle
 */
#include "Wire.h"                     // I2C library
#include <EEPROM.h>                   // EEPROM access library
#include "I2Cdev.h"                   // I2Cdev library                from https://github.com/jrowberg 
#include "MPU6050.h"                  // MPU6050 library               from https://github.com/jrowberg
#include <Adafruit_GFX.h>             // Adafruit graphical library    from https://github.com/adafruit/Adafruit-GFX-Library
#include <Arduino_ST7789_Fast.h>      // ST7789 colour display library from https://github.com/cbm80amiga/Arduino_ST7789_Fast
#include "RREFont.h"                  // RRE Font library              from https://github.com/cbm80amiga/RREFont
#include "rre_6x8.h"                  // RRE Font used
#include <ButtonEvents.h>             // Button Event library          from https://github.com/fasteddy516/ButtonEvents 
// ==> make sure to also install the Bounce2 library from https://github.com/thomasfredericks/Bounce2 as ButtonEvents is using it
//----------------------------------------------------------------------------------------------------------------------------------
#define DELAY_MEASURE 10              // Measure each 10ms
#define DELAY_DISPLAY 250             // Display display each 250ms
#define DELAY_START_INIT 2000         // delay to start calibration 
byte varCompteur = 0;
long t1 = 0;                          // Timer for Display management
long t2 = 0;                          // Timer for measure
int16_t ax, ay, az;                   // raw measure
int16_t gx, gy, gz;
uint8_t Accel_range;
uint8_t Gyro_range;
float travel = 0.0;
float angle = 0.0, af_angle = 0.0, ref_angle = 0.0;
//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize = 1000;   //Amount of readings used to average, make it higher to get more precision but calibration will be slower  (default:1000)
int acel_deadzone = 8;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
uint16_t ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
//----------------------------------------------------------------------------------------------------------------------------------
float debat = 0, debat_min = 1000, debat_max = -2000;
float Memdebat = 100000, Memdebat_min = 3000, Memdebat_max = -2000;
float angle_min = 200, angle_max = -200;
float Memangle = 1, Memangle_min = 3, Memangle_max = 4;
float corde = 68, Memcorde = 2;
bool update_frame = true;

//----------------------------------------------------------------------------------------------------------------------------------
// Push button definition
//----------------------------------------------------------------------------------------------------------------------------------
#define buttonPin 2                   // Push button wired on D2
ButtonEvents myButton;                // create an instance of the ButtonEvents class to attach to our button
//----------------------------------------------------------------------------------------------------------------------------------
// MPU6050 definition
//----------------------------------------------------------------------------------------------------------------------------------
MPU6050 accelgyro;
//----------------------------------------------------------------------------------------------------------------------------------
// ST7789 colour TFT display definition -- using SPI bus
//----------------------------------------------------------------------------------------------------------------------------------
#define TFT_DC    7                   // ST7789 display DC wired on D7
#define TFT_RST   8                   // ST7789 display RST wired on D8
#define SCR_WD   240                  // ST7789 width 240 pixels
#define SCR_HT   240                  // ST7789 height 240 pixels
Arduino_ST7789 tft = Arduino_ST7789(TFT_DC, TFT_RST); // Define TFT colour display
//----------------------------------------------------------------------------------------------------------------------------------
// RRE font init
//----------------------------------------------------------------------------------------------------------------------------------
RREFont font;
void customRect(int x, int y, int w, int h, int c) {
  return tft.fillRect(x, y, w, h, c);
}
/*******************************************************
  //   MPU6050 measurement
 *******************************************************/
void manageMeasure() {
  if (millis() - t2 > DELAY_MEASURE)
  {
    t2 = millis();
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    angle = 0.98 * (angle + float(gy) * 0.01 / 131) + 0.02 * atan2((double)ax, (double)az) * 180 / PI;
    af_angle = fmod((ref_angle - angle) + 180, 360);                         // compute angle variation vs. reference angle
    if (af_angle < 0) {
      af_angle += 180;
    } else {
      af_angle -= 180;
    }
    //    af_angle = angle - ref_angle;
    debat = corde * sin(af_angle * PI / 180.0);
    debat_min = min(debat, debat_min);
    debat_max = max(debat, debat_max);
    angle_min = min(af_angle, angle_min);
    angle_max = max(af_angle, angle_max);
  }
} /* End manageMeasure */

//----------------------------------------------------------------------------------------------------------------------------------
void init_angle() {                                // Initialize the actual angle as the reference angle
  //----------------------------------------------------------------------------------------------------------------------------------
  font.setBold(1); font.setScale(2);                             // Large font
  font.setColor(RED);
  font.printStr(ALIGN_CENTER, 100, "Zeroing sensor");
  font.printStr(ALIGN_CENTER, 120, "don't move it");
  font.setScale(1); font.setBold(0);
  delay(500);
  manageMeasure();
  ref_angle = angle;              // set reference angle
  delay(500);
  manageMeasure();
  debat_min = 200;
  debat_max = -200;
  angle_min = 200;
  angle_max = -200;
  Memcorde = 2;
  Memangle = 200;
}
/*******************************************************
     MPU6050 Initialization
 *******************************************************/
void Init()
{
  /*--- reset offsets   ---*/
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);

  meansensors();
  delay(1000);

  calibration();
  delay(1000);

  /*--- New reads and offsets display ---*/
  meansensors();

  /*--- Set offsets with the compute values ---*/
  accelgyro.setXAccelOffset(ax_offset);
  accelgyro.setYAccelOffset(ay_offset);
  accelgyro.setZAccelOffset(az_offset);
  accelgyro.setXGyroOffset(gx_offset);
  accelgyro.setYGyroOffset(gy_offset);
  accelgyro.setZGyroOffset(gz_offset);

} /* End Init() */

/*******************************************************
   average sensors reading
 *******************************************************/
void meansensors() {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
  float zz;
  while (i < (buffersize + 101)) {
    // read raw accel/gyro measurements from device
    zz = i;
    zz =  disp_value(zz, zz + 1, 10, 0, 30, 150, true, RED, GREEN);

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
} /* End meansensors() */
/*******************************************************
   MPU6050 calibration
 *******************************************************/
void calibration() {
  float zz;
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;

  while (1) {
    int ready = 0;
    zz = ready;
    zz =  disp_value(zz, zz + 1, 2, 0, 100, 170, true, BLUE, GREEN);
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();

    if (abs(mean_ax) <= acel_deadzone) ready++;
    else ax_offset = ax_offset - mean_ax / acel_deadzone;

    if (abs(mean_ay) <= acel_deadzone) ready++;
    else ay_offset = ay_offset - mean_ay / acel_deadzone;

    if (abs(16384 - mean_az) <= acel_deadzone) ready++;
    else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= giro_deadzone) ready++;
    else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

    if (abs(mean_gy) <= giro_deadzone) ready++;
    else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

    if (abs(mean_gz) <= giro_deadzone) ready++;
    else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

    if (ready == 6) break;
  }
} /* End calibration */

//----------------------------------------------------------------------------------------------------------------------------------
void calibrate() {                                // Calibrate MPU6050
  //----------------------------------------------------------------------------------------------------------------------------------
  font.setBold(1); font.setScale(2);                             // Large font
  font.setColor(RED);
  font.printStr(ALIGN_CENTER, 100, "CALIBRATING");
  font.printStr(ALIGN_CENTER, 120, "MPU SENSOR");
  Init();
  tft.fillRect(0, 100, 240, 140, GREEN);
  eeprom_save();
}
//----------------------------------------------------------------------------------------------------------------------------------
bool eeprom_read() {                              // Retrieve MPU6050 calibration data at power-up
  //----------------------------------------------------------------------------------------------------------------------------------
  uint16_t AX, AY, AZ;
  uint16_t GX, GY, GZ;
  boolean res = false;
  font.setBold(1); font.setScale(2);                             // Large font
  font.setColor(MAGENTA);
  font.printStr(ALIGN_CENTER, 100, "READING");
  font.printStr(ALIGN_CENTER, 120, "SAVED");
  font.printStr(ALIGN_CENTER, 140, "VALUES");
  EEPROM.get(0, AX);
  EEPROM.get(4, AY);
  EEPROM.get(8, AZ);
  EEPROM.get(12, GX);
  EEPROM.get(16, GY);
  EEPROM.get(20, GZ);
  res = isnan(AX) or isnan(AY) or isnan(AZ);         // check if valid data
  res = res or isnan(GX) or isnan(GY) or isnan(GZ);  // check if valid data
  if (res) {                                            // IF no valid data in EEPROM
    font.setBold(2); font.setScale(3);
    font.setColor(RED);
    font.printStr(ALIGN_CENTER, 180, "ERROR");
    delay(2000);
  }
  else {
    ax_offset = AX;
    ay_offset = AY;
    az_offset = AZ;
    gx_offset = GX;
    gy_offset = GY;
    gz_offset = GZ;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);
    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);
  }
  tft.fillRect(0, 100, 240, 140, GREEN);
  return (!res);                                        // IF no valid data in EEPROM return read failed
}
//----------------------------------------------------------------------------------------------------------------------------------
void eeprom_save() {                              // Save MPU6050 calibration data at power-up
  //----------------------------------------------------------------------------------------------------------------------------------
  font.setBold(1); font.setScale(2);                             // Large font
  font.setColor(RED);
  font.printStr(ALIGN_CENTER, 100, "SAVING");
  font.printStr(ALIGN_CENTER, 120, "CALIBRATION");
  font.printStr(ALIGN_CENTER, 140, "VALUES");
  EEPROM.put(0, ax_offset);
  EEPROM.put(4, ay_offset);
  EEPROM.put(8, az_offset);
  EEPROM.put(12, gx_offset);
  EEPROM.put(16, gy_offset);
  EEPROM.put(20, gz_offset);
  tft.fillRect(0, 100, 240, 140, GREEN);
}
//----------------------------------------------------------------------------------------------------------------------------------
// Display initialization
//----------------------------------------------------------------------------------------------------------------------------------
void setup_display() {
  Wire.begin();                                  // Start SPI
  tft.begin();                                   // TFT display Start
  font.init(customRect, SCR_WD, SCR_HT);         // custom fillRect function and screen width and height values
  font.setFont(&rre_6x8); font.setSpacing(1);   // Define font used
}
//----------------------------------------------------------------------------------------------------------------------------------
// MPU initialization
//----------------------------------------------------------------------------------------------------------------------------------
void setup_MPU() {
  accelgyro.initialize();                        // initialize MPU device
  if (!accelgyro.testConnection()) {             // if MPU fails to connect then display error message
    font.setSpacing(1);
    font.setScale(2);
    font.setBold(1);
    font.setColor(RED);
    font.printStr(ALIGN_CENTER, 75 + font.getHeight() * 4, "MPU Error");
    font.printStr(ALIGN_CENTER, 60 + font.getHeight() * 8, "Programm stopped");
    while (1);                                   // Freeze program here
  }
}
//----------------------------------------------------------------------------------------------------------------------------------
// Button initialization
//----------------------------------------------------------------------------------------------------------------------------------
void setup_Button() {
  myButton.attach(buttonPin);                    // attach ButtonEvents instance to the button pin
  myButton.activeLow();                          // button is active when grounded
  myButton.debounceTime(15);                     // apply 15ms debounce time on button                                   --> decrease chord value
  myButton.doubleTapTime(300);                   // set double-tap detection window to 250ms                             --> increase chord value
  myButton.holdTime(2000);                       // require button to be held for 2000ms before triggering a hold event  --> set actual angle as reference angle
}
//----------------------------------------------------------------------------------------------------------------------------------
// Application and components setup
//----------------------------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);                            // Start serial for debug message display
  setup_display();                               // Initialize display
  affiche_init();                                // Display initialization information
  setup_MPU();                                   // Initialize MPU

  pinMode(buttonPin, INPUT_PULLUP);              // define the Push Button input

  if (!digitalRead(buttonPin)) {                 // If button pressed during power-up
    calibrate();
  }                              // Calibrate sensor
  else {                                         // if button not pressed
    if (!eeprom_read()) calibrate();             // read data from eeprom, if no valide data calibrate
  }
  init_angle();                                  // Set actual angle as reference angle
  setup_Button();                                // Initialize push button behavior
  update_frame = true;                           // Authorize full frame draw
}

/*----------------------------------------------------------------------------------------------------------------------------------
  // Convert a float variable into a string with a specific number of digits
  //----------------------------------------------------------------------------------------------------------------------------------*/
String cnv_flt2str(float num, int car, int digit) {
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
/*----------------------------------------------------------------------------------------------------------------------------------
   Display initialization message
    ---------------------------------------------------------------------------------------------------------------------------------*/
void affiche_init() {
  tft.fillScreen(GREEN);                          // TFT clear display with GREY backgound
  font.setBold(0);
  font.setScale(3);                              // Large font
  font.setColor(BLACK, GREEN);
  font.printStr(ALIGN_CENTER, 60, "Initializing");
}
/*----------------------------------------------------------------------------------------------------------------------------------
   Function displaying values on main menu if the new value is different of the memorized value
   this was needed as the value would have been lightly blinking when unchanged overwritten
   return the value displayed
  ----------------------------------------------------------------------------------------------------------------------------------*/
float disp_value(float val, float memval, int nbChar, int nbDec, int col, int line, bool update_fr, uint16_t fwdColor, uint16_t bckColor) {
  char buf[100];
  String txt;
  if ((cnv_flt2str(val, nbChar, nbDec) != cnv_flt2str(memval, nbChar, nbDec)) or update_fr) {
    memval = val;
    txt = cnv_flt2str(val, nbChar, nbDec) + "  ";
    txt.toCharArray(buf, txt.length() + 1);
    font.setColor(fwdColor, bckColor);
    font.printStr(col, line, buf);
  }
  return memval;
}
//----------------------------------------------------------------------------------------------------------------------------------
// Main page display
//----------------------------------------------------------------------------------------------------------------------------------
void affiche() {
  int h = 20, StCol = 5, DspCol = 120;
  if ((millis() - t1) > DELAY_DISPLAY) {                     // If update display authorized
    t1 = millis();
    if (update_frame) {                                      // frame text display authorized
      tft.fillScreen(BLACK);                                 // TFT clear display with BLACK backgound
      tft.drawRect(0, 28, 240, 62, RED);
      tft.drawRect(0, 99+26, 240, 62, RED);
      tft.drawRect(0, 199, 240, 26, RED);
      font.setBold(1);
      font.setScale(3);
      font.setColor(GREEN);
      font.printStr(ALIGN_CENTER, 3, "ANGLE");
      font.setBold(0);
      font.setScale(2);
      font.setColor(CYAN);
      font.printStr(StCol, 2+1.5 * h, "Min");
      font.printStr(StCol, 2+3.5 * h, "Max");
      font.setColor(YELLOW);
      //font.setBold(1);
      font.printStr(StCol, 2+2.5 * h , "Current");
      font.setBold(1);
      font.setScale(3);
      font.setColor(GREEN);
      font.printStr(ALIGN_CENTER, 2 + 5 * h-2, "THROW");
      font.setBold(0);
      font.setScale(2);
      font.setColor(CYAN);
      font.printStr(StCol, 5 * h + 1.5 * h, "Min");
      font.printStr(StCol, 5 * h + 3.5 * h, "Max");
      font.setColor(YELLOW);
      //font.setBold(1);
      font.printStr(StCol, 5 * h + 2.5 * h , "Current");

      font.setColor(YELLOW, BLACK);
      //font.setBold(2);
      font.printStr(StCol, 5 * h + 5.2 * h, "Chord");
      update_frame = false;                                 // Frame updated no need to update it again
    }
    /* Display all values if different from last display */
    /* <<<<<<<<<<<<< A N G L E   values >>>>>>>>>>>>>>>>>>>>>>>    */
    font.setBold(0);
    font.setScale(2);
    Memangle_min =  disp_value(angle_min, Memangle_min, 7, 1, DspCol, 2+1.5 * h, update_frame, CYAN, BLACK);
    Memangle_max =  disp_value(angle_max, Memangle_max, 7, 1, DspCol, 2+3.5 * h, update_frame, CYAN, BLACK);
    Memangle = disp_value(af_angle, Memangle, 7, 1, DspCol, 2+2.5 * h, update_frame, YELLOW, BLACK);
    /* <<<<<<<<<<<<< T H R O W   values >>>>>>>>>>>>>>>>>>>>>>>    */
    Memdebat_min =  disp_value(debat_min, Memdebat_min, 7, 1, DspCol, 5 * h + 1.5 * h, update_frame, CYAN, BLACK);
    Memdebat_max =  disp_value(debat_max, Memdebat_max, 7, 1, DspCol, 5 * h + 3.5 * h, update_frame, CYAN, BLACK);
    Memdebat = disp_value(debat, Memdebat, 7, 1, DspCol, 5 * h + 2.5 * h, update_frame, YELLOW, BLACK);
    /* <<<<<<<<<<<<< C H O R D   values >>>>>>>>>>>>>>>>>>>>>>>    */
    Memcorde = disp_value(corde, Memcorde, 6, 0, DspCol, 5 * h + 5.2 * h, update_frame, YELLOW, BLACK);

  }
}
/*---------------------------------------------------------------------------------------------------------------------------------
    Button read and actions state:
    0 --> nothing happened
    1 --> Single click
    2 --> Double click
    3 --> Long time pressed
  //---------------------------------------------------------------------------------------------------------------------------------*/
uint8_t Read_button() {
  uint8_t btn = 0;
  myButton.update();
  // things to do if the button was tapped (single tap)
  if (myButton.tapped() == true) {
    btn = 1;
  }
  // things to do if the button was double-tapped
  if (myButton.doubleTapped() == true) {
    btn = 2;
  }
  // things to do if the button was holded
  if (myButton.held() == true) {
    btn = 3;
  }
  return btn;
}
/* ---------------------------------------------------------------------------------------------------------------------------------
   Manage the button pressed events
     Single Click -> decrease Chord value
     Double Click -> increase Chord value
     Long press   -> Set reference angle
*/
void manageButton() {
  switch (Read_button()) {
    case (0): {                       // <<<<<<<<<<<<<<< button not pressed
        break;
      }
    case (1): {                       // <<<<<<<<<<<<<<< button single click -> decrease Chord value
        if (corde > 11)  {
          corde--;
        }
        break;
      }
    case (2): {                       // <<<<<<<<<<<<<<< button bouble click -> increase Chord value
        if (corde < 149) {
          corde++;
        }
        break;
      }
    case (3): {                       // <<<<<<<<<<<<<<< long button press -> Set reference angle
        affiche_init();               // display initialisation page
        init_angle();                 // reference angle = current angle
        update_frame = true;
        break;
      }
  }
}
/*----------------------------------------------------------------------------------------------------------------------------------
   Main Program loop
  ----------------------------------------------------------------------------------------------------------------------------------*/
void loop() {
  manageButton();           // Read button action and react as needed
  manageMeasure();          // Measure current angle values and do all math
  affiche();                // Display values and if needed display frame
}
