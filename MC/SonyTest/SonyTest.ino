

#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/String.h>

#include <SPI.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789


#define PBR 7
#define PBC 6
#define PBL 5

#define TFT_CS        10
#define TFT_RST        9 
#define TFT_DC         8

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
//Adafruit_ST7789 tft = Adafruit_ST7789(&SPI4, TFT_CS, TFT_DC, TFT_RST);
//Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
char guipos = 0;
char* guitext[3] = {"Move", "Stop", "Turn"};
char commandtxt[3] ={'M','I','S'};

bool transmit = false;
bool buttonpress[3] = {false, false, false};

void left() {
  //Serial.print("Left");
  static unsigned long OldIntTimeL;
  unsigned long IntTimeL = millis();
  if ((IntTimeL - OldIntTimeL) > 100) {
    if (guipos > 0) {
      tft.setCursor(200, guipos * 34);
      tft.setTextColor(ST77XX_BLACK);
      tft.print("<");
      guipos--;
      tft.setCursor(200, guipos * 34);
      tft.setTextColor(ST77XX_WHITE);
      tft.print("<");
    }
  }
  OldIntTimeL = IntTimeL;
}
void right() {
  //Serial.print("Right");
  static unsigned long OldIntTimeR;
  unsigned long IntTimeR = millis();
  if ((IntTimeR - OldIntTimeR) > 100) {
    if (guipos < (sizeof(guitext) / sizeof(*guitext)) - 1) {
      tft.setCursor(200, guipos * 34);
      tft.setTextColor(ST77XX_BLACK);
      tft.print("<");
      guipos++;
      tft.setCursor(200, guipos * 34);
      tft.setTextColor(ST77XX_WHITE);
      tft.print("<");
    }
  }
  OldIntTimeR = IntTimeR;
}
void centre() {
  static unsigned long OldIntTimeC;
  unsigned long IntTimeC = millis();
  if ((IntTimeC - OldIntTimeC) > 100) {
    Serial.print(guitext[guipos]);
    Serial2.write(commandtxt[guipos]);
  }
  OldIntTimeC = IntTimeC;
}
void setup() {
  Serial.begin(9600);//Serial for debugging
  //TRN.begin(115200); //Serial To ESP32
  Serial2.begin(115200);
  tftsetup();

  pinMode(PBL, INPUT);
  pinMode(PBC, INPUT);
  pinMode(PBR, INPUT);
  attachInterrupt(PBL, left, FALLING);
  attachInterrupt(PBC, centre, FALLING);
  attachInterrupt(PBR, right, FALLING);
  delay(1000);
}

void loop() {

}

void tftsetup() {
  tft.init(240, 240);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);
  for (int i = 0; (i < sizeof(guitext) / sizeof(*guitext)); i++) {
    tft.setCursor(0, i * 32);
    tft.print(guitext[i]);
  }
}
