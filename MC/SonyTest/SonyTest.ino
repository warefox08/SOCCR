
#include <SoftwareSerial.h>

#include <ros.h>
#include <std_msgs/String.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

#define REP 14
#include <SPI.h>

#define PBR 21
#define PBC 20
#define PBL 19

#define TFT_CS 24 // Chip Select SPI_CS
#define TFT_SCLK 23  // Clock out SPI_SCK
#define TFT_DC 17  //Data Clock SPI_MISO
#define TFT_MOSI 16  // Data out SPI_MOSI
#define TFT_RST 15  //Reset PWM

#define RX 0
#define TX 1
SoftwareSerial TRN(RX, TX);

//Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
char guipos =0;
char* guitext[3] ={"Move","Interact","Status"};
char* commandtxt[3] ={"M","I","S"};

bool transmit=false;

void left(){
  //Serial.print("Left");
  if (guipos > 0){
  tft.setCursor(200, guipos*34);
  tft.setTextColor(ST77XX_BLACK);
  tft.print("<");
  guipos--;
  tft.setCursor(200, guipos*34);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("<");
  }
}
void right(){
  //Serial.print("Right");  
  if (guipos < (sizeof(guitext) / sizeof(*guitext))-1){
  tft.setCursor(200, guipos*34);
  tft.setTextColor(ST77XX_BLACK);
  tft.print("<");
  guipos++;
  tft.setCursor(200, guipos*34);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("<");
  }
}
void centre(){
  //Serial.print("Centre \n");  
  Serial.print(guitext[guipos]);
  TRN.write(commandtxt[guipos]);
}
void setup() {
  Serial.begin(9600);//Serial for debugging
  TRN.begin(115200); //Serial To ESP32
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

void tftsetup(){
  tft.init(240, 240);  
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);
  for (int i=0;(i < sizeof(guitext) / sizeof(*guitext)); i++){
    tft.setCursor(0,i*32);
    tft.print(guitext[i]);
  }
}
