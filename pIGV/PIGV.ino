#include "bitmaps.h"

#define ENCODER_DO_NOT_USE_INTERRUPTS
//#include "Encoder.h"
#include "hardware/pio.h"
#include "quadrature.pio.h"

#define QUADRATURE_A_PIN 21
#define QUADRATURE_B_PIN 20

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


int scroll = 0;     //position of whatever is on the display, so this is the compass heading or text scroll
int scroll2 = 112;  // used to increment scroll for text scrolling (this value here is the start point)

int displayedMenu = 2;  //mode selection - 0 for compass  1 = Menu   2 = scrolling text

//general display stuff
char displayText[330] = { "Holy fuck guys! It's a pIGV1-16\0 " };  //this is where you put text to display (from wifi or serial or whatever) terminated with \0
int invertedChars[330];                                            // put 1 in locations where you want text to be knockout
unsigned char displayBuffer[2000];                                 //this is where the actual bitmap is loaded to be shown on the display


//compass specific stuff
int selectedStart = 0;  //this lets the display follow the selected menu items
unsigned char menuArray[6][20] = {
  { "Heading" }, { "Declination" }, { "Brightness" }, { "Altitude" }, { "Marker" }, { "Drive in circles!!" }
};
int menuArrayLengths[6] = { 7, 11, 10, 8, 6, 19 };
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
int brightness = 120;  //number of microseconds to hold on each column of the display (works from like 130 - 270, above that it gets kinda flashy)
bool toggleButton = 1;
volatile bool pressed = 0;
int lastPosition = 0;
int oldPos = 0;

boolean newData = false;
volatile int encoder_value = 0;
int encoderOffset = 0;
int encoderRaw = 0;

int scrollFlag = 1;

void setup() {
  Serial.begin(115200);
  Serial.println("ready");

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  pinMode(BUTTONENC, INPUT_PULLUP);


    offset = pio_add_program(pio, &quadrature_program);
  sm = pio_claim_unused_sm(pio, true);
  quadrature_program_init(pio, sm, offset, QUADRATURE_A_PIN, QUADRATURE_B_PIN);

}

void loop() {
  recvWithEndMarker();
  showNewData();

 // Serial.println(scrollSpeed);
    pio_sm_exec_wait_blocking(pio, sm, pio_encode_in(pio_x, 32));

encoderRaw = pio_sm_get_blocking(pio, sm);

encoder_value = encoderRaw  - encoderOffset;

scrollSpeed = encoder_value;


if (digitalRead(BUTTONENC) == 0)
{
  scrollFlag = !scrollFlag;
  encoderOffset =  encoderRaw;
  encoder_value = 0;
  delay(300);
}

}

void recvWithEndMarker() {
  static int ndx = 10;
  char endMarker = '\n';
  char rc;


  while (Serial.available() > 0 && newData == false) {

    rc = Serial.read();

    if (rc == '%')
    {
      displayedMenu = 0;
      scrollFlag = 0;
    } 

    if (rc != endMarker) {

      displayText[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = 20;
      }
      charsInDisplay = ndx - 9;

    } else {

      displayText[ndx] = '\0';  // terminate the string
//Serial.println (ndx);
      for (int i = 0; i < 10; i++)
          {
            displayText[i] = ' ';
           // Serial.println (i);
          }
      for (int i = ndx+1; i < 330; i++)
          {
            displayText[i] = ' ';
           // Serial.println (i);
          }

      ndx = 10;
      newData = true;
      scroll2  = 0;
    }
  }
  //scroll2 = 50;
}

void showNewData() {
  if (newData == true) {
    Serial.print("This just in ... ");
    Serial.println(displayText);
    newData = false;
  }
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

  unsigned long timeNow = millis();

  
if (scrollFlag == 1)
{
  if (timeNow - timeLast > scrollSpeed) {
    scroll2++;
    scrollCharPosition = scroll2 / 6;

    if (scroll2 >= (charsInDisplay * 6) + 18) {
      scroll2 = 6;
    }
    timeLast = timeNow;
  }
} else {
  scroll2 = encoder_value;
}
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

  } else if (headingToShow >= 10000) { //I know heading will never be more than 3 digits, but if you want to show different numbers you can up to 99999

    asciiInt[5] = headingToShow % 10;
    asciiInt[4] = (headingToShow / 10) % 10;
    asciiInt[3] = (headingToShow / 100) % 10;
    asciiInt[2] = (headingToShow / 1000) % 10;
    asciiInt[1] = (headingToShow / 10000) % 10;
  }


  for (int i = 1; i < 9; i++) { //finds the last digit to replace with menuReminder bitmap
    if (asciiInt[i] == -16) {
      lastDigit = i;
      break;
    }
  }
  switch (selectedMenuItem) { //these change the units to thing to remind you what you're doing
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
    menuReminderAscii = 82;
  }

  if (showAltitude == 1 && showHeading == 0) {
    menuReminderAscii = 83;
  }

  if (declinationSet == 1) {
    menuReminderAscii = 79;
  }

  asciiInt[lastDigit] = menuReminderAscii;

  if (asciiInt[1] == 1) rowCount -= 1;

  for (int j = 1; j <= 11; j++) { //this actually adds the heading stuff to the display buffer

    for (int k = 0; k < 5; k++) {

      if (k >= 5) {
        displayBuffer[(scrollValue + rowCount) % 360] = 0b11111111; //this adds a blank column between letters
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

    if (asciiInt[j] == 1) rowCount -= 1;//this is because the digit 1 is narrower than the rest and the spacing was bugging me //#keming

    if (asciiInt[j + 1] == -16) {//this adds space between the heading and the compass 
      if (menuReminderAscii != 78 && menuReminderAscii != 82) { //different spacings looked right depending on the menuReminder
        displayBuffer[(scrollValue + rowCount) % 360] = 0b11111111;
        displayBuffer[(scrollValue + rowCount + 1) % 360] = 0b11111111;
        displayBuffer[(scrollValue + rowCount + 2) % 360] = 0b11111111;
      }
      return;
    }
  }
}

void loadDisplayBuffer(void) {  //this fills displayBuffer[] with data depending on the menu


  switch (displayedMenu) {
    case 0:  //compass
      {

        hoveredMenuItem = 0;

        for (int i = 0; i < 360; i++) {

          displayBuffer[i] = compass[i];
          if (marker == i && markerFlag == 1) {

            displayBuffer[i] = 0b0000000;
          }
          if (declinationSet == 1 && i == ((scroll + 55) % 360)) displayBuffer[i] = 0b0000000;
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


              if (invertedChars[i / 6] == 1) { //for inverted charachters
                if (j == 0) {
                  displayBuffer[i + j - 1] = 0b00000000;
                }
                if (j < 5) {

                  displayBuffer[i + j] = charBitmaps[letterInt][j];

                } else {
                  displayBuffer[bufferPosition + j] = 0b00000000;
                }
              } else {                        //non inverted charachters
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

  digitalWrite(blank, LOW);
  delayMicroseconds(10);

  if (displayedMenu == 0) {

    scroll = scroll2; //change this to scroll = heading; when I get the compass on

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
/*
      float pressure = (float)((baroADC[0] / 4096.0f));
      pressure -= 92.0f;

      float pressureExp = pow((pressure / (1013.25f + pressureOffset)), 0.190284);

      float elevation = 145366.45f * (1 - pressureExp);

      //int elevationInt = (int)filter.add(elevation); //use this with the movingAvg library
      int elevationInt = (int)elevation;

      showHeadingNumbers(elevationInt);
      */
    }
  } else if (displayedMenu == 1) { //compass menu offsets

    for (int i = 0; i < 60; i++) {
      if (invertedChars[i] == 1) {
        selectedStart = i;
        break;
      }
    }
    scroll = (selectedStart * 6) + 20;

  } else { //scrolling text

    scroll = scroll2;
  }

  for (int i = scroll; i < (111 + scroll); i++) {

    digitalWrite(row0, HIGH); //pull display anodes low (~100V) while dealing with the scan cathodes
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

      digitalWrite(scan2, HIGH); //pull the scan high first to give an infinitesimally small overlap 
      digitalWrite(scan1, LOW);
      digitalWrite(scan3, LOW);

    } else if (scanLocation == 2) {

      digitalWrite(scan3, HIGH);
      digitalWrite(scan1, LOW);
      digitalWrite(scan2, LOW);

    }

    unsigned char row;

    if (displayedMenu == 2) 
    {
      row = displayBuffer[i]; //for scrolling text we can show more than 360 pixels in the display buffer
    } else {
      row = displayBuffer[i % 360];//for the compass we're only interested in the first 360 of the displayBuffer
    }

    digitalWrite(row0, row & 0b01000000); //displays one column of data on the scan anodes
    digitalWrite(row1, row & 0b00100000);
    digitalWrite(row2, row & 0b00010000);
    digitalWrite(row3, row & 0b00001000);
    digitalWrite(row4, row & 0b00000100);
    digitalWrite(row5, row & 0b00000010);
    digitalWrite(row6, row & 0b00000001);

    delayMicroseconds(brightness); //give the plasma time to cook

    digitalWrite(row0, HIGH); //pull all anodes low for the next cylcle
    digitalWrite(row1, HIGH); //I know this is basically running twice in a row but
    digitalWrite(row2, HIGH); //for some reason the display is more stable 
    digitalWrite(row3, HIGH); //when I add this
    digitalWrite(row4, HIGH);
    digitalWrite(row5, HIGH);
    digitalWrite(row6, HIGH);
    
  }
  //interrupts();
  scanLocation = -1; //-1 so it doesnt scan on the first time though the loop

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

  digitalWrite(blank, HIGH); //blanking pulse (left on until the next time trough this function)


  //scanLocation = 0;
  delayMicroseconds(45);


  //digitalWrite(blank, LOW);
}
