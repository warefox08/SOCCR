#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/String.h>

#include <SPI.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

#define TRN 4 // Transistor pin
#define PBR 5 // Right pushbutton pin
#define PBC 6 // Central pushbutton pin
#define PBL 7 // Left pushbutton pin


#define TFT_CS 10 // SPI Chip Select Pin
#define TFT_RST 9 // SPI Reset Pin
#define TFT_DC 8 // SPI Data/Command Pin

#define LENGTH(X) (sizeof X/sizeof X[0])// Function for array length
#define LW 34 //Line width
#define YO 0 //Y offset
#define TO 35 //Text offset

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

char guipos = 0;
char* guitext[4] = {"Toggle", "Move", "Right", "Left"};
char commandtxt[5] = {'T','M','R', 'L','S'};
char guilen = LENGTH(guitext); 

bool buttonpress[3] = {false, false, false}; //Debounce flags
bool DF = 1; //Default Flag

unsigned long responseT; //REMOVE ON FINAL FOR TESTING

void guidown() {
  if (guipos > 0) {
    tft.setCursor(10,YO+guipos * LW);
    tft.setTextColor(ST77XX_BLACK);
    tft.print(">");
    guipos--;
    tft.setCursor(10,YO+guipos * LW);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(">");
  }
}

void guiup() {
  if (guipos < guilen) {
    tft.setCursor(10,YO+guipos * LW);
    tft.setTextColor(ST77XX_BLACK);
    tft.print(">");
    guipos++;
    tft.setCursor(10,YO+guipos * LW);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(">");
  }
}


void left() {
  //Serial.print("Left"); //REMOVE ON FINAL FOR TESTING
  static unsigned long OldIntTimeL;
  unsigned long IntTimeL = millis();
  if ((IntTimeL - OldIntTimeL) > 100) {
    guidown();
  }
  OldIntTimeL = IntTimeL;
}
void right() {
  //Serial.print("Right");//REMOVE ON FINAL FOR TESTING
  static unsigned long OldIntTimeR;
  unsigned long IntTimeR = millis();
  if ((IntTimeR - OldIntTimeR) > 100) {
    guiup();
  }
  OldIntTimeR = IntTimeR;
}
void centre() {
  static unsigned long OldIntTimeC;
  unsigned long IntTimeC = millis(); 
  if ((IntTimeC - OldIntTimeC) > 100) {
    Serial.print(guitext[guipos]);
    Serial2.write(commandtxt[guipos]);
    if (guipos == 0){
      digitalWrite(TRN, !digitalRead(TRN));
    }
    //responseT = millis(); //REMOVE ON FINAL FOR TESTING
  }
  OldIntTimeC = IntTimeC;
}
void setup() {
  Serial.begin(9600);//Serial for debugging
  Serial2.begin(115200);
  tftsetup();

  pinMode(TRN, OUTPUT);
  pinMode(PBL, INPUT);
  pinMode(PBC, INPUT);
  pinMode(PBR, INPUT);
  attachInterrupt(digitalPinToInterrupt(PBL), left, FALLING);
  attachInterrupt(digitalPinToInterrupt(PBR), right, FALLING);
  attachInterrupt(digitalPinToInterrupt(PBC), centre, FALLING);

  delay(1000);
}

void loop() {
  if (Serial2.available()) {
    char cmd = Serial2.read();
    switch (cmd){
      case 'C':
      screenstate(cmd);
      delay(100);
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(0,YO+128);
      tft.print("Movement Complete");
      delay(1500);
      tft.setTextColor(ST77XX_BLACK);
      tft.setCursor(0,YO+128);
      tft.print("Movement Complete");
      break;
      
      case 'N':
      screenstate(cmd);
      delay(100);
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(0,YO+128);
      tft.print("Laser Not Found");
      delay(1500);
      tft.setTextColor(ST77XX_BLACK);
      tft.setCursor(0,YO+128);
      tft.print("Laser Not Found");
      break;
      case 'F':
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(0,YO+128);
      tft.print("Laser Found");
      delay(1500);
      screenstate(cmd);
      break;
    }
    //float T = millis() - responseT; //REMOVE ON FINAL FOR TESTING
    //Serial.print("\n"); //REMOVE ON FINAL FOR TESTING
    //Serial.print(T);  //REMOVE ON FINAL FOR TESTING 
  }
}

void screenstate(char ScrSt) {
  switch (ScrSt) {
    case 'F':
      detachInterrupt(digitalPinToInterrupt(PBL));
      detachInterrupt(digitalPinToInterrupt(PBR));
      tft.setCursor(10, guipos * LW);
      tft.setTextColor(ST77XX_BLACK);
      tft.print(">");
      guipos = 4;
      tft.fillScreen(ST77XX_BLACK);
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(0,YO+0);
      tft.print("Movement in\nprogress, \npress fire to stop");
      DF = 0;
      break;
    default:
      if (DF == 0) {
        attachInterrupt(digitalPinToInterrupt(PBL), left, FALLING);
        attachInterrupt(digitalPinToInterrupt(PBR), right, FALLING);
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextColor(ST77XX_WHITE);
        guipos = 0;
        tft.setCursor(10, YO+0);
        tft.setTextColor(ST77XX_WHITE);
        tft.print(">");
        tft.setTextSize(3);
        for (int i = 0; i < guilen; i++) {
          tft.setCursor(TO, YO+i * LW);
          tft.print(guitext[i]);
        }
        tft.drawLine(0,guilen*LW,240,guilen*LW, ST77XX_WHITE);
      }
      DF = 1;
      break;
  }
}
void tftsetup() {
  tft.init(240, 240);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);
  for (int i = 0; i < guilen; i++) {
    tft.setCursor(TO,YO+i*LW);
    tft.print(guitext[i]);
  }
  tft.drawLine(0,guilen*LW,240, guilen*LW, ST77XX_WHITE);
  guipos = 0;
  tft.setCursor(10,YO+0);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(">");
}
