
#include <Arduino.h>
#include "bitmaps.h"

#include <string.h>

#include "i2c_api.h"
#include "quat2euler.h"
#include "report.h"
#include "USFSMAX.h"

#include "hardware/pio.h"
#include "quadrature.pio.h"

#include <EEPROM.h>

#include "MovingAverage.h"

#define QUADRATURE_A_PIN 21
#define QUADRATURE_B_PIN 20

MovingAverage<float, 32> filter;  //powers of 2 only   this sets how fast altitude responds


PIO pio = pio0;
uint offset, sm;

#define ENC_A 21      //ky-040 clk pin, add 100nF/0.1uF capacitors between pin & ground!!!
#define ENC_B 20      //ky-040 dt  pin, add 100nF/0.1uF capacitors between pin & ground!!!
#define BUTTONENC 22  //ky-040 sw  pin, add 100nF/0.1uF capacitors between pin & ground!!!

#define blank 2

#define scan3 15
#define scan2 14
#define scan1 13


#define row0 6
#define row1 7
#define row2 8
#define row3 9
#define row4 10
#define row5 11
#define row6 12


int scroll = 0;   //position of whatever is on the display, so this is the compass heading or text scroll
int scroll2 = 0;  // used to increment scroll for text scrolling (this value here is the start point)

int displayedMenu = 3;  //mode selection - 0 for compass  1 = Menu   2 = scrolling text   3 = splSH

//general display stuff
char displayText[500] = { "Calibration error! Setting to 3D DHI. Remove compass module and tumble until calibration is finished, then reset and run 2D calibration.     \0 " };  //this is where you put text to display (from wifi or serial or whatever) terminated with \0
int invertedChars[500];                                                                                                                                                          // put 1 in locations where you want text to be knockout
unsigned char displayBuffer[3000];                                                                                                                                               //this is where the actual bitmap is loaded to be shown on the display


//compass specific stuff
int selectedStart = 0;  //this lets the display follow the selected menu items
unsigned char menuArray[6][20] = {
  { "Heading" }, { "Declination" }, { "Brightness" }, { "Clicks" }, { "Marker" }, { "              " }
};
int menuArrayLengths[6] = { 7, 11, 10, 6, 6, 19 };
int hoveredMenuItem = 0;  // 1 = Heading   2 = Declination    3 = Brightness     4 = Calibrate
int selectedMenuItem = 0;
int menuItem = 0;  // 1 = Heading   2 = Declination    3 = Brightness     4 = Calibrate   5 = Marker
int marker = 180;
int markerFlag = 0;
int markerSetFlag = 0;  //1 for marker selection menu
int calibratingFlag = 0;
int calConfirm = 0;
int showHeading = 1;   //whether to show heading numbers on the left in the compass
int showAltitude = 0;  //this shows pressure altitude so it's not very accurate, but if you know the altitude of where you start it can be adjusted
int declinationSet = 0;
int Declination = 11;  // substitute your magnetic declination but really this wont matter because this is used to adjust the heading
float pressureOffset = 0.0;
int clicks = 1;
bool lastState = 0;
int position = 0;

int tilt = 0;

int calibrationError = 0;

uint8_t HIoffsetStatus = 0;

//scrolling text stuff
int charsInDisplay = 0;
const int numChars = 600;
char receivedChars[numChars];  // an array to store the received data
int scrollSpeed = 30;          //number of millisecods to wait before moving 1 pixel
unsigned long timeLast = 0;
int scrollCharPosition;

unsigned long previousMillis = 0;
const long interval = 100;


unsigned long previousMillisScroll;
int scanLocation = -1;  //keeps track of where we are in the cathode scanning sequence (-1 keeps it from scanning the first cycle)
int brightness = 120;   //number of microseconds to hold on each column of the display (works from like 130 - 270, above that it gets kinda flashy)
bool toggleButton = 1;
volatile bool pressed = 0;
int lastPosition = 0;
int oldPos = 0;

boolean newData = true;
volatile int encoder_value = 0;
int encoderOffset = 0;
int encoderRaw = 0;

int scrollFlag = 1;

static const uint8_t INT_PIN = 3;

// Geomagnetic field data for Lexington, Virginia on 8 June 2022.
// https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
// Units are uT, angles are in decimal degrees.
static const float M_V = 42.0589f;
static const float M_H = 23.1087f;
static const float MAG_DECLINATION = 11.1477f;

static const uint32_t SERIAL_UPDATE_PERIOD_MSEC = 500;

// Dynamic Hard Iron corrector
static bool ENABLE_DHI_CORRECTOR = true;

// Dynamic Hard Iron Corrector allgorithm (otherwise 3D)
static bool USE_2D_DHI_CORRECTOR = true;

// I2C Clock Speed (Uncomment one only)
static const uint32_t I2C_CLOCK = 400000; /* 100000 */ /* 400000 */

// Instantiate class object
USFSMAX usfsmax;

static bool gyroCalActive = false;

// Data-ready interrupt handler
static volatile bool dataReady = true;

    uint8_t calRegister;
    uint8_t dhiRSQlsb;
    uint8_t dhiRSQmsb;
    int dhiRSQ;


void setup() {



  // Open serial port
  Serial.begin(115200);
  delay(200);

  // Initialize usfsmax I2C bus

  i2cBegin();
  delay(100);


  i2cSetClock(100000);
  delay(200);


  HIoffsetStatus = i2cReadByte(0x57, 0x01);  // High bit will be 1 if there's a valid HI offset in EEPROM
  HIoffsetStatus = HIoffsetStatus & 0b10000000;
  HIoffsetStatus = HIoffsetStatus >> 7;

  if (USE_2D_DHI_CORRECTOR == true && HIoffsetStatus == 0)  //The compass module hangs if there's no HI offset and it tries to use the 2D HI correction
  {
    calibrationError = 1;
    USE_2D_DHI_CORRECTOR = false;

    displayedMenu = 2;
    calibratingFlag = 1;

    for (int i = 0; i < 865; i++) {
      delay(11);
      scroll2 = i;
    }
    scroll2 = 0;

    Serial.println("Calibration Error! Setting to 3D DHI");  //so this sets it to 3D DHI, then you remove the compass and tumble it to get any HI offset
    delay(2000);                                             //then reset the compass and then do a 2D Calibration (this is a bug in the compass module code)
  }


  usfsmax.setGeoMag(M_V, M_H, MAG_DECLINATION);
  delay(100);
  Serial.println("Initializing usfsmax...\n");
  usfsmax.begin();



  delay(300);
  if (ENABLE_DHI_CORRECTOR) {  //this was originally in the USFSMAX.begin but I moved it here so I can have some control over it
    if (USE_2D_DHI_CORRECTOR) {

      Serial.println("Using 2D DHI");
      // Enable DHI corrector, 2D (0x10|0x50)


      Serial.print("HIoffsetStatus\t");
      Serial.println(HIoffsetStatus);

      if (HIoffsetStatus == 0) {

        calibratingFlag = 1;
      }
      i2cWriteByte(0x57, 0x61, 0x50);
      //calibratingFlag = 1;
    } else {

      Serial.println("Using 3D DHI");


      Serial.print("HIoffsetStatus\t");
      Serial.println(HIoffsetStatus);

      if (HIoffsetStatus == 0) {

        calibratingFlag = 1;
      }
      // Enable DHI corrector, 3D (0x10)
      i2cWriteByte(0x57, 0x61, 0x10);
    }
  }


  delay(100);
  usfsmax.retrieveCalibration();

  delay(100);
  i2cSetClock(I2C_CLOCK);
  
  pinMode(17, OUTPUT_12MA);
  digitalWrite(17, HIGH);
  delay(1);
  digitalWrite(17, LOW);

  serialPrintMessage("USFXMAX_0 successfully initialized!\n");
  Serial.begin(115200);


  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  pinMode(BUTTONENC, INPUT_PULLUP);
  displayedMenu = 3;
  //loadDisplayBuffer();
  //writeNeon();

  offset = pio_add_program(pio, &quadrature_program);
  sm = pio_claim_unused_sm(pio, true);
  quadrature_program_init(pio, sm, offset, QUADRATURE_A_PIN, QUADRATURE_B_PIN);

  int eepromCommitFlag = 0;
  EEPROM.begin(256);

  EEPROM.get(0, Declination);


  EEPROM.get(10, brightness);
  if (brightness > 500 || brightness < 50) {
    brightness = 160;
    EEPROM.put(10, brightness);
    eepromCommitFlag = 1;
  }

  EEPROM.get(20, marker);
  if (marker > 360 || marker < 0) {
    marker = 180;
    EEPROM.put(20, marker);
    eepromCommitFlag = 1;
  }

  EEPROM.get(30, showHeading);
  if (showHeading != 1 || showHeading != 0) {
    showHeading = 1;
    EEPROM.put(30, showHeading);
    eepromCommitFlag = 1;
  }

  EEPROM.get(40, clicks);
  if (clicks > 2 || clicks < 0) {
    clicks = 2;
    EEPROM.put(40, clicks);
    eepromCommitFlag = 1;
  }


  if (eepromCommitFlag == 1) //only commit to the fake EEPROM if a number was out of bounds and needs it
  {
     EEPROM.commit(); 
  }

  delay(100);
  // Attach interrupt
  pinMode(INT_PIN, INPUT);
  attachInterrupt(INT_PIN, handleInterrupt, RISING);

  if (calibrationError == 1) {
    usfsmax.resetDhi();
  }
}

int showSplash = 1;

unsigned long holdButton = 0;
unsigned long holdButtonFirstPressed = 0;
unsigned long lastPress = 0;
int pressFlag = 0;
int showDHI = 0;

int newPositionEncoder = 0;

void loop() {


  pio_sm_exec_wait_blocking(pio, sm, pio_encode_in(pio_x, 32)); //PIO rotary encoder handler

  encoderRaw = (pio_sm_get_blocking(pio, sm));

  encoder_value = encoderRaw - encoderOffset;


  if (encoder_value != newPositionEncoder) {
    position = newPositionEncoder;
    encoderISR();
  }


if (pressed == 1) encoderButtonISR();


  int buttonState = digitalRead(BUTTONENC);

  if (buttonState == 0) {
    //click();
    //click();
    //click();
   // pressed = 1;
    // Serial.println("Pressed!");
  }



//



  static float angles[3];
  static uint32_t count;
  static uint32_t last_refresh;

  if (dataReady) {

    count++;

    dataReady = false;

    usfsmax.update();

    if (usfsmax.gotQuat()) {
      float quat[4] = {};
      usfsmax.readQuat(quat);
      quat2euler(quat, angles);
    }
  }




  if (showSplash == 1) {

    if (millis() < 2000 || (usfsmax.gotQuat() == 0)) {
      scroll2 = 0;
      displayedMenu = 3;
    } else {
      showSplash = 0;
      displayedMenu = 0;
      scroll2 = (int)angles[2];
    }
  } else {
    scroll2 = (int)angles[2];
  }


  if (millis() - last_refresh > SERIAL_UPDATE_PERIOD_MSEC) {  //runs twice per second (or whatever you set the serial update period to)

    //Serial.println(gyroCalActive);
//click(2);
    if (USE_2D_DHI_CORRECTOR == true && ((angles[0] > 10 || angles[0] < -10) || (angles[1] > 10 || angles[1] < -10))) {  //2D calibration values are discarded if the tilt is over 10 degrees, so this tells you
      Serial.println("Tilt!");
      tilt = 1;
    } else {
      tilt = 0;
    }

if (calibratingFlag == 1 || showDHI == 1)
  {
    calRegister = i2cReadByte(0x57, 0x01);
    Serial.println(calRegister, HEX);


    dhiRSQlsb = i2cReadByte(0x57, 0x5E);
    dhiRSQmsb = i2cReadByte(0x57, 0x5F);
    dhiRSQ = dhiRSQmsb << 8;
    dhiRSQ |= dhiRSQlsb;
    Serial.println(dhiRSQ);
  }

    if (showDHI == 1) {

      strcpy(displayText, " DHI Rsq = 0.                                                                                       \0");
      char dhiASCII[4];

      itoa(dhiRSQ, dhiASCII, 10);

      for (int i = 0; i < 4; i++) {
        displayText[13 + i] = dhiASCII[i];

        Serial.println(dhiASCII[i]);
      }

      displayedMenu = 2;
      scroll2 = 0;
      delay(10000);
      displayedMenu = 0;
      showDHI = 0;
    }

    if (calibratingFlag == 1) {
      //Serial.println(i2cReadByte(0x57, 0x01),BIN);


      if ((calRegister & 0x80) > 0)  //checks if there's a valid HI offset
      {

        Serial.println(calRegister, HEX);
        calibratingFlag = 0;
        dhiRSQlsb = i2cReadByte(0x57, 0x5E);
        dhiRSQmsb = i2cReadByte(0x57, 0x5F);
        dhiRSQ = dhiRSQmsb << 8;
        dhiRSQ |= dhiRSQlsb;
        Serial.println(dhiRSQ);
        showDHI = 1;



      } else {

        Serial.println(calRegister, HEX);

        dhiRSQlsb = 85;
        dhiRSQmsb = 164;
        dhiRSQ = dhiRSQmsb << 8;
        dhiRSQ |= dhiRSQlsb;
        Serial.println(dhiRSQ);
      }
    }
    //last_refresh = millis();
    Serial.println(calibratingFlag);
    last_refresh = millis();

    serialInterfaceHandler();

    serialReportAngles(count, angles);  //this prints roll pitch and yaw to the serial monitor
  }




  if (buttonState == 0) {  //hold the encoder button for 10 seconds to reset the DHI and start a calibration

    holdButton = millis();
    if (holdButton - holdButtonFirstPressed > 7000) {
      calibratingFlag = 1;
      usfsmax.resetDhi();

    } else {
    }


  } else {
    holdButtonFirstPressed = millis();
  }

  if (buttonState == 0 && (millis() - lastPress > 200) && pressFlag == 0) {
    pressed = 1;
    lastPress = millis();
    pressFlag = 1;

  } else if (buttonState == 1) {
    pressFlag = 0;
    pressed = 0;
  }



}


static void handleInterrupt() {

  dataReady = true;
  //Serial.println("!");
}

static void sendOneToProceed(void) {
  serialPrintMessage("Send '1' to continue...");

  while (true) {
    if (serialGetCharacter() == '1') {
      break;
    }
    delay(10);
  }
}

// Serial interface handler
static void serialInterfaceHandler() {
  uint8_t serial_input = 0;
  if (serialAvailable()) serial_input = serialGetCharacter();
  if (serial_input == '1') {  // Type '1' to initiate usfsmax Gyro Cal
    usfsmax.gyroCal();
    gyroCalActive = true;
  }

  // Type '2' to list current sensor calibration data
  if (serial_input == '2') {

    // Set I2C clock to 100kHz to read the calibration data from the MAX32660
    i2cSetClock(100000);
    delay(100);

    usfsmax.retrieveCalibration();

    // Resume high-speed I2C operation
    i2cSetClock(I2C_CLOCK);
    delay(100);

    serialReportCalibrationResults(&usfsmax);

    // Halt the serial monitor to let the user read the calibration data
    //sendOneToProceed();
  }

  // Type '3' to reset the DHI corrector
  if (serial_input == '3') {

    usfsmax.resetDhi();

    calibratingFlag = 1;
  }
  if (serial_input == '4') {

    showDHI = 1;
  }
  serial_input = 0;
}


void pressedEncoder() {
  //holdButtonFirstPressed = millis();
  //Serial.println("Pressed!");
  pressed = 1;
  //click();
}

void encoderISR() {
  //encoder.readAB();


  newPositionEncoder = encoder_value;  //encoder.getPosition();


  Serial.println(newPositionEncoder);
  if (declinationSet == 1) {

    Declination = (position % 360);
    EEPROM.put(0, Declination);
  }

  if (selectedMenuItem == 3) {

    int brightnessIncrement = 5;

    switch (brightness) {
      case 0 ... 99:
        brightnessIncrement = 1;
        break;

      case 100 ... 179:
        brightnessIncrement = 1;
        break;

      case 180 ... 270:
        brightnessIncrement = 1;
        break;

      case 271 ... 350:
        brightnessIncrement = 2;
        break;

      case 351 ... 500:
        brightnessIncrement = 15;
        break;

      case 501 ... 2000:
        brightness /= 10;
        brightness *= 10;
        brightnessIncrement = 50;
        break;

      case 2001 ... 2000000:
        brightness /= 10;
        brightness *= 10;
        brightnessIncrement = 200;
        break;
    }


    if (oldPos < position) {
      brightness += brightnessIncrement;
      EEPROM.put(10, brightness);

    } else {
      brightness -= brightnessIncrement;
      EEPROM.put(10, brightness);
    }
    oldPos = position;
  }

  if (selectedMenuItem == 5) {

    if (oldPos < position) {
      marker += 1;
      if (marker >= 360) marker = 0;
    } else {
      marker -= 1;
      if (marker < 0) marker = 359;
    }
    oldPos = position;

    marker = marker % 360;
    EEPROM.put(20, marker);
  }


  // if (showAltitude == 1) {

  //   if (oldPos < position) {
  //     pressureOffset += 0.05;

  //   } else {
  //     pressureOffset -= 0.05;
  //   }
  //   oldPos = position;
  //   EEPROM.put(40, pressureOffset);
  // }


  // letter = abs(position);
  
  hoveredMenuItem = abs(position % 7);

if (displayedMenu == 1) {
  if (hoveredMenuItem >= 1 && hoveredMenuItem <= 5) 
  {

    click(2);
  } }
  else {

    click(clicks);
  }


  //position = Declination;
}


void click(int type)
{
  

  
if (type == 1) {
  if (lastState == 1)
  { digitalWrite(17, LOW);
    lastState = 0;
   
  }
  else
  { digitalWrite(17, HIGH);
    lastState = 1;
   
  }

} else if (type == 2)
{

    digitalWrite(17, LOW);
    lastState = 0;
    delay(5);
    digitalWrite(17, HIGH);


} else if (type == 3)
{
digitalWrite(17, LOW);

}


}





void encoderButtonISR() {

  pressed = 0;


  if (selectedMenuItem > 5) selectedMenuItem = 0;
  //while(1);
  //noInterrupts();
  toggleButton = !toggleButton;




  if (selectedMenuItem == 5) {
    declinationSet = 0;
    showAltitude = 0;
    showHeading = 1;
    EEPROM.put(20, marker);
    EEPROM.commit();
    markerSetFlag = 0;

    hoveredMenuItem = 0;
    selectedMenuItem = hoveredMenuItem;
    displayedMenu = !displayedMenu;
    //interrupts();
    return;
  }
  selectedMenuItem = hoveredMenuItem;

  if (selectedMenuItem == 1) {
    //showHeading = !showHeading;

    declinationSet = 0;
    encoderOffset = position;
    //encoder.setPosition (0);

    if (showHeading == 1) {
      showAltitude = 0;
      showHeading = 0;
    } else {
      showAltitude = 0;
      showHeading = 1;
    }
    //selectedMenuItem = 0;
    EEPROM.put(30, showHeading);
    EEPROM.commit();
    hoveredMenuItem = 0;
    selectedMenuItem = 0;
    //displayedMenu = 0;
    displayedMenu = !displayedMenu;

    return;
  }

  if (selectedMenuItem == 2) {
    if (declinationSet == 0) {
      // encoder.setPosition(Declination);
      declinationSet = 1;
      showHeading = 1;
      hoveredMenuItem = 2;
      selectedMenuItem = 2;
      displayedMenu = 0;
      EEPROM.put(0, Declination);
      EEPROM.commit();
    } else {
      declinationSet = 0;
      EEPROM.put(0, Declination);
      EEPROM.get(30, showHeading);
      EEPROM.commit();
      hoveredMenuItem = 0;
      selectedMenuItem = 0;
      //selectedMenuItem = hoveredMenuItem;
      displayedMenu = 0;
      //interrupts();
    }
    return;
  } else {
    declinationSet = 0;
  }

  if (selectedMenuItem == 3) {
    EEPROM.put(10, brightness);
    declinationSet = 0;
    //hoveredMenuItem = 0;

    //displayedMenu = !displayedMenu;
  }


  if (selectedMenuItem == 5) {

    declinationSet = 0;
    if (markerFlag == 0) {
      markerFlag = 1;
      markerSetFlag = 1;


    } else if (markerFlag == 1) {
      markerFlag = 0;
      markerSetFlag = 0;
      hoveredMenuItem = 0;
      selectedMenuItem = hoveredMenuItem;
    }
  }


  if (selectedMenuItem == 4) {
    declinationSet = 0;
    encoderOffset = position;
    if (clicks == 0) {
      clicks = 2;
      //click(3);
      
    } else if (clicks == 2) {
      
      clicks = 1;
      //click(2);

    } else {
      clicks = 0;
    }
    EEPROM.put(40, clicks);
    hoveredMenuItem = 4;
    //selectedMenuItem = 4;
    //EEPROM.commit();
    //return;
  }
  click(2);
  EEPROM.commit();

  displayedMenu = !displayedMenu;

//digitalWrite(17, LOW);
  //hoveredMenuItem = 0;
  //interrupts();
  //click();
  
}






void setup1() {
  pinMode(scan1, OUTPUT_8MA);
  pinMode(scan2, OUTPUT_8MA);
  pinMode(scan3, OUTPUT_8MA);

  pinMode(row0, OUTPUT_8MA);
  pinMode(row1, OUTPUT_8MA);
  pinMode(row2, OUTPUT_8MA);
  pinMode(row3, OUTPUT_8MA);
  pinMode(row4, OUTPUT_8MA);
  pinMode(row5, OUTPUT_8MA);
  pinMode(row6, OUTPUT_8MA);

  pinMode(blank, OUTPUT_8MA);
}

void loop1() {



  loadDisplayBuffer();
  writeNeon();
}




void showHeadingNumbers(int headingToShow) {

  // if (displayedMenu == 1) return;

  int numberCount = 0;
  int rowCount = 0;
  int scrollValue = scroll;

  int lastDigit = 1;
  int menuReminderAscii = 78;
  int asciiInt[12] = { -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16 };

  if ((showHeading == 1 || declinationSet == 1 || markerSetFlag == 1 && showAltitude != 1) && selectedMenuItem != 3) {
    headingToShow = (headingToShow + 280) % 360;
  }

  if (headingToShow < 10) {

    asciiInt[1] = headingToShow;


  } else if (headingToShow >= 10 && headingToShow < 100) {

    asciiInt[2] = headingToShow % 10;
    asciiInt[1] = headingToShow / 10;

  } else if (headingToShow >= 100 && headingToShow < 1000) {

    asciiInt[3] = headingToShow % 10;
    asciiInt[2] = (headingToShow / 10) % 10;
    asciiInt[1] = (headingToShow / 100) % 10;

  } else if (headingToShow >= 1000 && headingToShow < 10000) {

    asciiInt[4] = headingToShow % 10;
    asciiInt[3] = (headingToShow / 10) % 10;
    asciiInt[2] = (headingToShow / 100) % 10;
    asciiInt[1] = (headingToShow / 1000) % 10;

  } else if (headingToShow >= 10000) {  //I know heading will never be more than 3 digits, but if you want to show different numbers you can up to 99999

    asciiInt[5] = headingToShow % 10;
    asciiInt[4] = (headingToShow / 10) % 10;
    asciiInt[3] = (headingToShow / 100) % 10;
    asciiInt[2] = (headingToShow / 1000) % 10;
    asciiInt[1] = (headingToShow / 10000) % 10;
  }


  for (int i = 1; i < 9; i++) {  //finds the last digit to replace with menuReminder bitmap
    if (asciiInt[i] == -16) {
      lastDigit = i;
      break;
    }
  }
  switch (selectedMenuItem) {  //these change the units to thing to remind you what you're doing
    case 0:
      menuReminderAscii = 78;
      break;

    case 1:
      break;

    case 2:
      menuReminderAscii = 79;
      break;

    case 3:
      menuReminderAscii = 81;
      break;

    case 5:
      menuReminderAscii = 80;
      break;
  }

  if (calibratingFlag == 1) {

    if (tilt == 0) {
      menuReminderAscii = 82;
    } else {
      menuReminderAscii = 84;
    }
  }

  if (showAltitude == 1 && showHeading == 0 && selectedMenuItem != 2 && selectedMenuItem != 3) {
    menuReminderAscii = 83;
  }

  if (declinationSet == 1) {
    menuReminderAscii = 79;
  }

  asciiInt[lastDigit] = menuReminderAscii;

  if (asciiInt[1] == 1) rowCount -= 1;

  for (int j = 1; j <= 11; j++) {  //this actually adds the heading stuff to the display buffer

    for (int k = 0; k < 5; k++) {

      if (k >= 5) {
        displayBuffer[(scrollValue + rowCount) % 360] = 0b11111111;  //this adds a blank column between letters
      } else {
        displayBuffer[(scrollValue + rowCount) % 360] = ~charBitmaps[asciiInt[j] + 16][k];
      }
      rowCount++;
    }


    displayBuffer[(scrollValue + rowCount) % 360] = 0b11111111;
    rowCount++;

    if (j + 1 == lastDigit && menuReminderAscii != 78) {
      displayBuffer[(scrollValue + rowCount) % 360] = 0b11111111;
      rowCount++;
    }

    if (asciiInt[j] == 1) rowCount -= 1;  //this is because the digit 1 is narrower than the rest and the spacing was bugging me //#keming

    if (asciiInt[j + 1] == -16) {                                //this adds space between the heading and the compass
      if (menuReminderAscii != 78 && menuReminderAscii != 82) {  //different spacings looked right depending on the menuReminder
        displayBuffer[(scrollValue + rowCount) % 360] = 0b11111111;
        displayBuffer[(scrollValue + rowCount + 1) % 360] = 0b11111111;
        displayBuffer[(scrollValue + rowCount + 2) % 360] = 0b11111111;
      }
      return;
    }
  }
}

void loadDisplayBuffer(void) {  //this fills displayBuffer[] with data depending on the menu
  static int halftoneStep = 0;

  switch (displayedMenu) {
    case 3:
      {
        //hoveredMenuItem = 0;
        //showHeading = 0;
        for (int i = 0; i < 112; i++) {

          displayBuffer[i] = splash[halftoneStep][i];
        }
        //halftoneStep++;
        //if (halftoneStep >= 3) halftoneStep = 1;
        break;
      }
    case 0:  //compass
      {

        hoveredMenuItem = 0;
        //scroll = scroll2;
        int decLoc = (scroll + 55) % 360;

        for (int i = 0; i < 360; i++) {

          displayBuffer[i] = compass[i];
          if (marker == i && markerFlag == 1) {

            displayBuffer[i] = 0b0000000;
          }
          if (declinationSet == 1 && i == decLoc) {
            displayBuffer[i] = 0b0000000;
          }
        }
      }
      break;

    case 1:  //compass menu
      {
        int letterCount = 0;
        int menuCount = 0;

        int displayTextLocation = 0;


        for (int k = 0; k < 5; k++) {

          for (int i = 0; i < menuArrayLengths[k]; i++) {
            displayText[displayTextLocation] = menuArray[k][i];
            if (hoveredMenuItem == k + 1) {
              invertedChars[displayTextLocation] = 1;

            } else {

              invertedChars[displayTextLocation] = 0;
            }

            displayTextLocation++;
          }
          displayText[displayTextLocation] = ' ';
          displayTextLocation++;
        }
        for (int i = 0; i < 30; i++)
        {
          displayText[displayTextLocation] = ' ';
          displayTextLocation++;
        }




        for (int i = 0; i < 40; i++) {
          displayBuffer[i] = 0b01111111;
        }

        for (int i = 0; i < 360; i += 6) {

          for (int j = 0; j < 6; j++) {

            int letterInt = displayText[i / 6] - 32;

            if (letterInt > 92 || letterInt < 0) letterInt = 0;


            if (invertedChars[i / 6] == 1) {
              if (j == 0) {
                displayBuffer[i + j - 1 + 40] = 0b00000000;
              }
              if (j < 5) {

                displayBuffer[i + j + 40] = charBitmaps[letterInt][j];

              } else {
                displayBuffer[i + j + 40] = 0b00000000;
              }
            } else {
              if (j < 5) {

                displayBuffer[i + j + 40] = ~charBitmaps[letterInt][j];

              } else {
                displayBuffer[i + j + 40] = 0b11111111;
              }
            }
          }
          letterCount++;
        }

        break;
      }
    case 2:  //scrolling text
      {


        int displayTextLocation2 = 0;
        int endFlag = 0;
        int bufferPosition = 0;
        int foundEndingFlag = 0;

        for (int i = 0; i < sizeof(displayBuffer); i += 6) {


          if (displayText[(i / 6)] == '\0' && foundEndingFlag == 0) {
            foundEndingFlag = 1;
            charsInDisplay = (i / 6) + 1;
            //bufferPosition = 0;
          }

          if ((bufferPosition / 6) > charsInDisplay)  //this makes it fill the entire buffer with repeated copies of displayText[]
          {                                           //if speed is an issue you can change this loop to not waste so many cycles
            endFlag = 1;
            bufferPosition = -6;
          } else {
            endFlag = 0;
          }

          for (int j = 0; j < 6; j++) {

            if (endFlag == 0) {
              int letterInt = displayText[bufferPosition / 6] - 32;

              if (letterInt > 92 || letterInt < 0) letterInt = 0;


              if (invertedChars[i / 6] == 1) {  //for inverted charachters
                if (j == 0) {
                  displayBuffer[i + j - 1] = 0b00000000;
                }
                if (j < 5) {

                  displayBuffer[i + j] = charBitmaps[letterInt][j];

                } else {
                  displayBuffer[bufferPosition + j] = 0b00000000;
                }
              } else {  //non inverted charachters
                if (j < 5) {

                  displayBuffer[i + j] = ~charBitmaps[letterInt][j];

                } else {
                  displayBuffer[i + j] = 0b11111111;
                }
              }
            } else {

              displayBuffer[i + j] = 0b11111111;
              //endFlag++;
            }
          }

          bufferPosition += 6;
        }
        break;
      }
  }
}



void writeNeon(void) {  //this function writes data to the display



  if (displayedMenu == 0) {

    scroll = scroll2;  //change this to scroll = heading; when I get the compass on

    scroll = (scroll + Declination);
    if (scroll < 0) scroll += 360;  // Allow for under|overflow
    if (scroll >= 360) scroll -= 360;


    if (markerSetFlag == 1) {
      scroll = marker + 305;
      scroll = scroll % 360;
    }

    if (selectedMenuItem == 3) {
      showHeadingNumbers(brightness);


    } else if (showHeading == 1) {

      showHeadingNumbers(scroll);

    } else if (showAltitude == 1 && declinationSet == 0) {

      float pressure = (float)((usfsmax.baroADC / 4096.0f));
      pressure += 142.0f;
      Serial.println(pressure);
      //Serial.println(usfsmax.baroADC);

      float pressureExp = pow((pressure / (1013.25f)), 0.190284);

      float elevation = 145366.45f * (1 - pressureExp);

      //int elevationInt = (int)filter.add(elevation); //use this with the movingAvg library
      int elevationInt = (int)elevation;

      //Serial.println(pressureOffset);

      showHeadingNumbers(elevationInt);
    }
  } else if (displayedMenu == 1) {  //compass menu offsets

    for (int i = 0; i < 60; i++) {
      if (invertedChars[i] == 1) {
        selectedStart = i;
        break;
      }
    }
    scroll = (selectedStart * 6) + 20;

  } else {  //scrolling text

    scroll = scroll2;
  }


  digitalWrite(blank, LOW);
  delayMicroseconds(10);

  for (int i = scroll; i < (111 + scroll); i++) {

    digitalWrite(row0, HIGH);  //pull display anodes low (~100V) while dealing with the scan cathodes
    digitalWrite(row1, HIGH);
    digitalWrite(row2, HIGH);
    digitalWrite(row3, HIGH);
    digitalWrite(row4, HIGH);
    digitalWrite(row5, HIGH);
    digitalWrite(row6, HIGH);

    scanLocation++;
    if (scanLocation >= 3) scanLocation = 0;

    if (scanLocation == 0) {

      digitalWrite(scan1, HIGH);
      digitalWrite(scan2, LOW);
      digitalWrite(scan3, LOW);

    } else if (scanLocation == 1) {

      digitalWrite(scan2, HIGH);  //pull the scan high first to give an infinitesimally small overlap
      digitalWrite(scan1, LOW);
      digitalWrite(scan3, LOW);

    } else if (scanLocation == 2) {

      digitalWrite(scan3, HIGH);
      digitalWrite(scan1, LOW);
      digitalWrite(scan2, LOW);
    }

    unsigned char row;

    if (displayedMenu == 2) {
      row = displayBuffer[i];  //for scrolling text we can show more than 360 pixels in the display buffer
    } else {
      row = displayBuffer[i % 360];  //for the compass we're only interested in the first 360 of the displayBuffer
    }

    digitalWrite(row0, row & 0b01000000);  //displays one column of data on the scan anodes
    digitalWrite(row1, row & 0b00100000);
    digitalWrite(row2, row & 0b00010000);
    digitalWrite(row3, row & 0b00001000);
    digitalWrite(row4, row & 0b00000100);
    digitalWrite(row5, row & 0b00000010);
    digitalWrite(row6, row & 0b00000001);

    delayMicroseconds(brightness);  //give the plasma time to cook

    digitalWrite(row0, HIGH);  //pull all anodes low for the next cylcle
    digitalWrite(row1, HIGH);  //I know this is basically running twice in a row but
    digitalWrite(row2, HIGH);  //for some reason the display is more stable
    digitalWrite(row3, HIGH);  //when I add this
    digitalWrite(row4, HIGH);
    digitalWrite(row5, HIGH);
    digitalWrite(row6, HIGH);
  }
  //interrupts();
  scanLocation = -1;  //-1 so it doesnt scan on the first time though the loop

  digitalWrite(row0, HIGH);
  digitalWrite(row1, HIGH);
  digitalWrite(row2, HIGH);
  digitalWrite(row3, HIGH);
  digitalWrite(row4, HIGH);
  digitalWrite(row5, HIGH);
  digitalWrite(row6, HIGH);


  digitalWrite(scan1, LOW);
  digitalWrite(scan2, LOW);
  digitalWrite(scan3, LOW);

  digitalWrite(blank, HIGH);  //blanking pulse (left on until the next time trough this function)


  //scanLocation = 0;
  delayMicroseconds(120);


  //digitalWrite(blank, LOW);
}
