/*For use with an Arduino Pro Micro & PMW3389 Motion Sensor.
  Complete project build files are available:

  Arduino code modified from the Ben Makes Everything Mouse, Daniel Kao Kensington Expert Mouse & Sunjun Kim PMW3360 Arduino projects: 
    https://youtu.be/qmX8vL-GbxU
    https://github.com/BenMakesEverything/PMW3389_Mouse
    https://github.com/dkao/Kensington_Expert_Mouse_PMW3389_Arduino
    https://github.com/SunjunKim/PMW3360_Arduino

  Additional PMW3389 sensor informaiton:
    https://github.com/mrjohnk/PMW3389DM
*/

#include <SPI.h>
#include <Keyboard.h>
#include <Adafruit_NeoPixel.h>
#include <RotaryEncoder.h>

//Sets mouse to use advance mode, nescessary for back/middle mouse buttons to work
#define ADVANCE_MODE

#ifdef ADVANCE_MODE
#include <AdvMouse.h>
#define MOUSE_BEGIN AdvMouse.begin()
#define MOUSE_PRESS(x) AdvMouse.press_(x)
#define MOUSE_RELEASE(x) AdvMouse.release_(x)
#else
#include <Mouse.h>
#define MOUSE_BEGIN Mouse.begin()
#define MOUSE_PRESS(x) Mouse.press(x)
#define MOUSE_RELEASE(x) Mouse.release(x)
#endif

#if defined(AVR)
#include <avr/pgmspace.h>
#else  //defined(AVR)
#include <pgmspace.h>
#endif  //defined(AVR)

#define LED_PIN 10
#define LED_COUNT 1
#define BRIGHTNESS 255  // Sets Neopixel brightness, 0-255

//Initialize Neopixel
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);

// Configurations
// The default CPI value should be in between 100 -- 12000
#define CPI 2300    //Default setting when mouse is first plugged in
#define DEBOUNCE 5  //unit = ms
#define NUMCPI 7    //Number of DPI presets

#define NUMBTN 5   //Number of mouse buttons
#define NUMKEYS 1  //This is the CPI switch button

#define Btn1_Pin 1   // Left Mouse
#define Btn2_Pin 0   // Right Mouse
#define Btn3_Pin 2   // Middle Mouse
#define Btn4_Pin 19  // Forward Mouse
#define Btn5_Pin 18  // Backward Mouse
#define Btn6_Pin 7   // Cycle CPI presets

//encoder pins
#define PIN_IN1 21
#define PIN_IN2 20

//Registers
#define Product_ID 0x00
#define Revision_ID 0x01
#define Motion 0x02
#define Delta_X_L 0x03
#define Delta_X_H 0x04
#define Delta_Y_L 0x05
#define Delta_Y_H 0x06
#define SQUAL 0x07
#define Raw_Data_Sum 0x08
#define Maximum_Raw_data 0x09
#define Minimum_Raw_data 0x0A
#define Shutter_Lower 0x0B
#define Shutter_Upper 0x0C
#define Ripple_Control 0x0D
#define Resolution_L 0x0E
#define Resolution_H 0x0F
#define Config2 0x10
#define Angle_Tune 0x11
#define Frame_Capture 0x12
#define SROM_Enable 0x13
#define Run_Downshift 0x14
#define Rest1_Rate_Lower 0x15
#define Rest1_Rate_Upper 0x16
#define Rest1_Downshift 0x17
#define Rest2_Rate_Lower 0x18
#define Rest2_Rate_Upper 0x19
#define Rest2_Downshift 0x1A
#define Rest3_Rate_Lower 0x1B
#define Rest3_Rate_Upper 0x1C
#define Observation 0x24
#define Data_Out_Lower 0x25
#define Data_Out_Upper 0x26
#define SROM_ID 0x2A
#define Min_SQ_Run 0x2B
#define Raw_Data_Threshold 0x2C
#define Control2 0x2D
#define Config5_L 0x2E
#define Config5_H 0x2F
#define Power_Up_Reset 0x3A
#define Shutdown 0x3B
#define Inverse_Product_ID 0x3F
#define LiftCutoff_Cal3 0x41
#define Angle_Snap 0x42
#define LiftCutoff_Cal1 0x4A
#define Motion_Burst 0x50
#define SROM_Load_Burst 0x62
#define Lift_Config 0x63
#define Raw_Data_Burst 0x64
#define LiftCutoff_Cal2 0x65
#define LiftCutoff_Cal_Timeout 0x71
#define LiftCutoff_Cal_Min_Length 0x72
#define PWM_Period_Cnt 0x73
#define PWM_Width_Cnt 0x74

RotaryEncoder* encoder = nullptr;

const int ncs = 4;    //PMW3389 "SS" pin
const int reset = 3;  //PMW3389 "MT" pin

int Btn_pins[NUMBTN] = {Btn1_Pin, Btn2_Pin, Btn3_Pin, Btn4_Pin, Btn5_Pin};
bool Btns[NUMBTN] = {false, false, false, false, false};       // button state indicator
uint8_t Btn_buffers[NUMBTN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // button debounce buffer
char Btn_keys[NUMBTN] = { MOUSE_LEFT, MOUSE_RIGHT, MOUSE_MIDDLE, MOUSE_FORWARD, MOUSE_BACK };

int Key_pins[NUMBTN] = { Btn6_Pin };  // unassigned dpi button
bool Keys[NUMBTN] = { false };
uint8_t Key_buffers[NUMBTN] = { 0xFF };
char Key_keys[NUMBTN];

//Colors that represent the CPI/DPI setting: Yellow=0, Green=1, Teal=2, Blue=3, Purple=4, Red=5, Orange=6,  
int DpiColor[NUMCPI][6] = {  { 140, 115, 0, 0 }, { 0, 255, 0, 0 }, { 0, 155, 100, 0 }, { 0, 55, 200, 0 }, { 85, 0, 170, 0 }, { 255, 0, 0, 0 }, { 200, 55, 0, 0 } };  

unsigned long Cpis[NUMCPI] = { 400, 800, 1600, 2000, 2300, 2600, 3600 };
struct CpiUpdater {
  bool target_set;
  bool updated;
  uint8_t target_cpi_index;
};

CpiUpdater CpiUpdate = { false, false, 4 };  // Default Dpi = Cpis[2]

byte initComplete = 0;
bool inBurst = false;   // in busrt mode
bool reportSQ = false;  // report surface quality
int16_t dx, dy;

unsigned long lastTS;
unsigned long lastButtonCheck = 0;
unsigned long curTime;

//Be sure to add the SROM file into this sketch via "Sketch->Add File"
extern const unsigned short firmware_length;
extern const unsigned char firmware_data[];

void setup() {
  pinMode(ncs, OUTPUT);
  pinMode(reset, INPUT_PULLUP);
  pinMode(Btn1_Pin, INPUT_PULLUP);
  pinMode(Btn2_Pin, INPUT_PULLUP);
  pinMode(Btn3_Pin, INPUT_PULLUP);
  pinMode(Btn4_Pin, INPUT_PULLUP);
  pinMode(Btn5_Pin, INPUT_PULLUP);
  pinMode(Btn6_Pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, RISING);  //c had off and worked
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, RISING);  //c had off and worked

  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);

  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

  //Neopixel code
  strip.begin();                    // INITIALIZE NeoPixel strip object (REQUIRED)
  setColor(4);                      // make sure this is set to the same # as setCPI
  strip.show();                     // Turn OFF all pixels ASAP
  strip.setBrightness(BRIGHTNESS);  // Set brightness to default val

  performStartup();

  dx = dy = 0;

  delay(1000);
  initComplete = 9;

  lastTS = micros();

  MOUSE_BEGIN;
  Keyboard.begin();
}

void adns_com_begin() {
  digitalWrite(ncs, LOW);
}

void adns_com_end() {
  digitalWrite(ncs, HIGH);
}

byte adns_read_reg(byte reg_addr) {
  adns_com_begin();

  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f);
  delayMicroseconds(35);  // tSRAD
  // read data
  byte data = SPI.transfer(0);

  delayMicroseconds(1);  //tSCLK-NCS for read operation is 120ns
  adns_com_end();
  delayMicroseconds(19);  //tSRW/tSRR (=20us) minus tSCLK-NCS

  return data;
}

void adns_write_reg(byte reg_addr, byte data) {
  adns_com_begin();

  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80);
  //sent data
  SPI.transfer(data);

  delayMicroseconds(20);  //tSCLK-NCS for write operation
  adns_com_end();
  delayMicroseconds(100);  //tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound
}

void adns_upload_firmware() {
  // send the firmware to the chip, cf p.18 of the datasheet
  //Write 0 to Rest_En bit of Config2 register to disable Rest mode.
  adns_write_reg(Config2, 0x00);

  // write 0x1d in SROM_enable reg for initializing
  adns_write_reg(SROM_Enable, 0x1d);

  // wait for more than one frame period
  delay(10);  // assume that the frame rate is as low as 100fps... even if it should never be that low

  // write 0x18 to SROM_enable to start SROM download
  adns_write_reg(SROM_Enable, 0x18);

  // write the SROM file (=firmware data)
  adns_com_begin();
  SPI.transfer(SROM_Load_Burst | 0x80);  // write burst destination adress
  delayMicroseconds(15);

  // send all bytes of the firmware
  unsigned char c;
  for (int i = 0; i < firmware_length; i++) {
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }

  //Read the SROM_ID register to verify the ID before any other register reads or writes.
  adns_read_reg(SROM_ID);

  //Write 0x00 (rest disable) to Config2 register for wired mouse or 0x20 for wireless mouse design.
  adns_write_reg(Config2, 0x00);

  adns_com_end();
}

void setCPI(int cpi) {
  unsigned cpival = cpi / 50;
  adns_com_begin();
  adns_write_reg(Resolution_L, (cpival & 0xFF));
  adns_write_reg(Resolution_H, ((cpival >> 8) & 0xFF));
  adns_com_end();
}

void performStartup(void) {
  // hard reset
  adns_com_end();    // ensure that the serial port is reset
  adns_com_begin();  // ensure that the serial port is reset
  adns_com_end();    // ensure that the serial port is reset

  adns_write_reg(Shutdown, 0xb6);  // Shutdown first
  delay(300);

  adns_com_begin();  // drop and raise ncs to reset spi port
  delayMicroseconds(40);
  adns_com_end();
  delayMicroseconds(40);

  adns_write_reg(Power_Up_Reset, 0x5a);  // force reset
  delay(50);                             // wait for it to reboot

  // read registers 0x02 to 0x06 (and discard the data)
  adns_read_reg(Motion);
  adns_read_reg(Delta_X_L);
  adns_read_reg(Delta_X_H);
  adns_read_reg(Delta_Y_L);
  adns_read_reg(Delta_Y_H);
  // upload the firmware
  adns_upload_firmware();
  delay(10);

  setCPI(Cpis[CpiUpdate.target_cpi_index]);
}

// Button state checkup routine
void check_button_state() {
  // runs only after initialization
  if (initComplete != 9)
    return;

  unsigned long elapsed = curTime - lastButtonCheck;

  // Update at a period of 1/8 of the DEBOUNCE time
  if (elapsed < (DEBOUNCE * 1000UL / 8))
    return;

  lastButtonCheck = curTime;
  // Debounce mouse buttons
  for (int i = 0; i < NUMBTN; i++) {
    int btn_state = digitalRead(Btn_pins[i]);
    Btn_buffers[i] = Btn_buffers[i] << 1 | btn_state;

    if (!Btns[i] && Btn_buffers[i] == 0x00)  // button press stabilized
    {
      MOUSE_PRESS(Btn_keys[i]);
      Btns[i] = true;
    } else if (Btns[i] && Btn_buffers[i] == 0xFF)  // button release stabilized
    {
      MOUSE_RELEASE(Btn_keys[i]);
      Btns[i] = false;
    }
  }

  //Debounce for KB keys
for (int k = 0; k < NUMKEYS; k++) {
    int Key_state = digitalRead(Key_pins[k]);
    Key_buffers[k] = Key_buffers[k] << 1 | Key_state;

    if (!Keys[k] && Key_buffers[k] == 0x00)  // button press stabilized
    {
      Keyboard.press(Key_keys[k]);
      Keys[k] = true;
    } else if (Keys[k] && Key_buffers[k] == 0xFF)  // button release stabilized
    {
      Keyboard.release(Key_keys[k]);
      Keys[k] = false;
    }
  }

  // CPI switcher
  if (Keys[0] == true /*&& Keys[0] == true*/) {
    if (CpiUpdate.target_set != true) {
      CpiUpdate.target_cpi_index = (CpiUpdate.target_cpi_index + 1) % NUMCPI;
      CpiUpdate.target_set = true;
      CpiUpdate.updated = false;
      setColor(CpiUpdate.target_cpi_index);
    }
  } else {
    CpiUpdate.target_set = false;
    CpiUpdate.updated = false;
  }
}

// device signature
void dispRegisters(void) {
  int oreg[7] = {
    0x00, 0x3F, 0x2A, 0x02
  };
  char* oregname[] = {
    "Product_ID", "Inverse_Product_ID", "SROM_Version", "Motion"
  };
  byte regres;

  digitalWrite(ncs, LOW);

  int rctr = 0;
  for (rctr = 0; rctr < 4; rctr++) {
    SPI.transfer(oreg[rctr]);
    delay(1);
    regres = SPI.transfer(0);
    delay(1);
  }
  digitalWrite(ncs, HIGH);
}

void loop() {
  byte burstBuffer[12];
  curTime = micros();
  unsigned long elapsed = curTime - lastTS;

  static int pos = 0;
  encoder->tick();  // just call tick() to check the state.

  int newPos = encoder->getPosition();
  if (pos != newPos) {
    int dir = (int)encoder->getDirection();
    AdvMouse.move(0, 0, dir);
    pos = newPos;
  }

  check_button_state();

  if (!inBurst) {
    adns_write_reg(Motion_Burst, 0x00);  // start burst mode
    lastTS = curTime;
    inBurst = true;
  }

  if (elapsed >= 1000)  // polling interval : more than > 0.5 ms.
  {
    adns_com_begin();
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));

    SPI.transfer(Motion_Burst);
    delayMicroseconds(35);  // waits for tSRAD

    SPI.transfer(burstBuffer, 12);  // read burst buffer
    delayMicroseconds(1);           // tSCLK-NCS for read operation is 120ns

    SPI.endTransaction();

    int motion = (burstBuffer[0] & 0x80) > 0;
    int surface = (burstBuffer[0] & 0x08) > 0;  // 0 if on surface / 1 if off surface

    int xl = burstBuffer[2];
    int xh = burstBuffer[3];
    int yl = burstBuffer[4];
    int yh = burstBuffer[5];

    int squal = burstBuffer[6];

    int x = xh << 8 | xl;
    int y = yh << 8 | yl;

    dx = x;
    dy = y;

    adns_com_end();

    // update only if a movement is detected.
#ifdef ADVANCE_MODE
    if (AdvMouse.needSendReport() || motion) {
      AdvMouse.move(dx, dy, 0);  //these are ngative because the sensor is rotated 180 degrees from default orientation

      dx = 0;
      dy = 0;
    }
#else
    if (motion) {
      signed char mdx = constrain(dx, -127, 127);
      signed char mdy = constrain(dy, -127, 127);

      Mouse.move(-mdx, -mdy, 0);

      dx = 0;
      dy = 0;
    }
#endif
    lastTS = curTime;
  }

  // update CPI cycled from button combo
  if (CpiUpdate.target_set == true && CpiUpdate.updated == false) {
    setCPI(Cpis[CpiUpdate.target_cpi_index]);
    CpiUpdate.updated = true;
  }
}

void checkPosition() {
  encoder->tick();
}

//R,G,B,W collor set function
void setColor(int DpiSetting) {
  int Rval = DpiColor[DpiSetting][0];
  int Gval = DpiColor[DpiSetting][1];
  int Bval = DpiColor[DpiSetting][2];
  int Wval = DpiColor[DpiSetting][3];
  strip.fill(strip.Color(Rval, Gval, Bval, Wval));
  strip.setBrightness(BRIGHTNESS);
  strip.show();
  delay(10);
}
