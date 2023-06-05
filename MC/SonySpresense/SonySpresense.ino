 #include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/String.h>

#include <SPI.h> //SPI library for the screen

#include <Adafruit_GFX.h>    // Core graphics functions library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

//IO pin definitions
#define TRN 2 // Transistor pin
#define PBR 5 // Right pushbutton pin
#define PBC 6 // Central pushbutton pin
#define PBL 7 // Left pushbutton pin

// Screen pin definitions
#define TFT_CS 10 // SPI Chip Select Pin
#define TFT_RST 9 // SPI Reset Pin
#define TFT_DC 8 // SPI Data/Command Pin

//Ease of use definitions
#define LENGTH(X) (sizeof X/sizeof X[0])// Function for array length
#define LW 34 //Line width
#define YO 24 //Y offset
#define TO 35 //Text offset

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

static int guilen = 3; //Length of Gui Page
static int guipages = 3; //Number of gui page
char guipos = 0; //Current GUI position
char guipg = 0; //Current page of GUI
char* guitext[3][3] = {{"Aim", "Turn", "Photo"},{"INT1", "INT2", "INT3"},{"Move","Back","Back & Off"}};
char commandtxt[3][3] = {{'A', 'T', 'P'}, {'X', 'Y', 'Z'},{'M','B','O'}};
char command = 'A';


bool buttonpress[3] = {false, false, false}; //Debounce flags
bool DF = 1; //Default Menu Flag, If a menu change resukts in 
bool LF = 0; //Laser Flag
bool LIF = 0; //Left Intterupt flag, 1 if in turning mode 
bool RIF = 0; //Right Interrupt Flag, 1 if in turning mode

int laseroff() {
  digitalWrite(TRN, 1); //Turn off Laser
  LF = 0; //Lower Laser Flag
  tft.fillCircle(230, 8, 4, ST77XX_BLACK); //Turn off top GUI Indicator
  return 0; //Return 0 to deconstruct original timer
}
void laseron() {
  digitalWrite(TRN, 0);
  LF = 1;
  tft.fillCircle(230, 8, 4, ST77XX_BLUE);
}
void fire(){ //Fire command that lets only the 'M','P' and 'S' command get transmitted
  switch(command){
    case 'M':
      Serial2.write(command);
      break;
    case 'P':
      Serial2.write(command);
      screenstate(command);
      break;
    case 'S':
      Serial2.write(command);
      screenstate(command);
      break;
    default:
      screenstate(command);
      break;
  }  
}
void guiup() {
  if (guipos > 0) {
    tft.setCursor(10, YO + guipos * LW);
    tft.setTextColor(ST77XX_BLACK);
    tft.print(">");  //Delete old cursor
    
    guipos--; //Decrement gui position
    command = commandtxt[guipg][guipos]; //Change active command
    
    tft.setCursor(10, YO+guipos*LW);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(">"); //Print new cursor
  } else if ((guipos==0) && (guipg!=0) && (guipg!=2)) {
    tft.setCursor(10, YO+guipos*LW);
    tft.setTextColor(ST77XX_BLACK);
    tft.print(">"); //Delete old cursor
    
    guipos = guilen - 1; //Increment gui position
    guipg--;
    command = commandtxt[guipg][guipos]; //Change active command
    printmenu(); //Change to new page
  } 
}

void guidown() {
  if (guipos < (guilen - 1)) {
    tft.setCursor(10, YO + guipos * LW);
    tft.setTextColor(ST77XX_BLACK);
    tft.print(">"); //Delete old cursor
    
    guipos++; //Increment gui position
    command = commandtxt[guipg][guipos]; //Change active command
    
    tft.setCursor(10, YO+guipos*LW);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(">"); //Print new cursor
  } else if ((guipos == (guilen-1)) && (guipg<(guipages-2))) {
    tft.setCursor(10, YO+guipos*LW);
    tft.setTextColor(ST77XX_BLACK);
    tft.print(">"); //Delete old cursor
    
    guipos = 0; //Increment gui position
    guipg++;
    command = commandtxt[guipg][guipos]; //Change active command
    printmenu(); //Change to new page
  }
}

void Tleft(){//ISR for left button in turn mode
  Serial2.write('L');
}
void Tright(){//ISR for right button in turn mode
  Serial2.write('R');
}
void left() {//ISR for left button in menu mode
  static unsigned long OldIntTimeL;
  unsigned long IntTimeL = millis();
  if ((IntTimeL - OldIntTimeL) > 100) {
    guiup();
  }
  OldIntTimeL = IntTimeL;
}
void right() {//ISR for right button in menu mode
  static unsigned long OldIntTimeR;
  unsigned long IntTimeR = millis();
  if ((IntTimeR - OldIntTimeR) > 100) {
    guidown();
  }
  OldIntTimeR = IntTimeR;
}
void centre() {//ISR for centre/fire/trigger button
  static unsigned long OldIntTimeC;
  unsigned long IntTimeC = millis();
  if ((IntTimeC - OldIntTimeC) > 100) {
      fire();
  }
  OldIntTimeC = IntTimeC;
}
void setup() {
  Serial2.begin(115200);
  tftsetup();

  pinMode(TRN, OUTPUT);
  pinMode(PBL, INPUT);
  pinMode(PBC, INPUT);
  pinMode(PBR, INPUT);
  attachInterrupt(digitalPinToInterrupt(PBL), left, FALLING);
  attachInterrupt(digitalPinToInterrupt(PBR), right, FALLING);
  LIF = 1;
  RIF = 1;
  attachInterrupt(digitalPinToInterrupt(PBC), centre, FALLING);
  delay(1000);
}

void loop() {
  if (Serial2.available()) {
    char cmd = Serial2.read(); //Incoming feedback from ESP32
    switch (cmd) { //Switch statements for parsing type of command
      case 'a': //Flashes green indicator that command has been ackowledged
        tft.fillCircle(210, 8, 4, ST77XX_GREEN);
        delay(1500);
        tft.fillCircle(210, 8, 4, ST77XX_BLACK);
        break;
      case 'n': //Briefly flashes message on screen
        delay(100);
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(0, YO + 128);
        tft.print("Laser Not Found");
        delay(1500);
        tft.setTextColor(ST77XX_BLACK);
        tft.setCursor(0, YO + 128);
        tft.print("Laser Not Found");
        break;
      case 'f'://Briefly flashes message on screen
        laseroff();
        screenstate('f');
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(0, YO + 128);
        tft.print("Laser Found");
        delay(1500);
        tft.setTextColor(ST77XX_BLACK);
        tft.setCursor(0, YO + 128);
        tft.print("Laser Found");
        laseroff();
        screenstate('f');
        break;
      case 'c': //Briefly flashes message on screen and returns to default menu state
        screenstate(cmd);
        delay(100);
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(0, YO + 128);
        tft.print("Movement\nComplete");
        delay(1500);
        tft.setTextColor(ST77XX_BLACK);
        tft.setCursor(0, YO + 128);
        tft.print("Movement\nComplete");
        break;
      case 'w': //Draws a connected symbol on the screen
        tft.drawRoundRect(170, 0, 25, 14, 8, ST77XX_WHITE);
        tft.drawRoundRect(171, 1, 23, 12, 8, ST77XX_WHITE);
        tft.drawRoundRect(160, 4, 25, 14, 8, ST77XX_WHITE);
        tft.drawRoundRect(161, 5, 23, 12, 8, ST77XX_WHITE);
        break;
      case 'q': //Removes the connected symbol on the screen
        tft.drawRoundRect(170, 0, 25, 14, 8, ST77XX_BLACK);
        tft.drawRoundRect(171, 1, 23, 12, 8, ST77XX_BLACK);
        tft.drawRoundRect(160, 4, 25, 14, 8, ST77XX_BLACK);
        tft.drawRoundRect(161, 5, 23, 12, 8, ST77XX_BLACK);
        break;
    }
  }
}
void clr() {//Clears the lower end of the screen
  tft.fillRect(0, YO - 4, 240, 240 - 4 - YO, ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
}
void screenstate(char ScrSt) {
  switch (ScrSt) {
    case 'A': //Menu State for Aiming
      clr();
      laseron();
      guipos=0;
      guipg=2;
      printmenu();
      command = commandtxt[guipg][guipos];
      DF = 0; 
      break;
    case 'O': //Exiting aiming sate and turning off laser
      clr();
      laseroff();
      guipos=0;
      guipg=0;
      printmenu();
      command = commandtxt[guipg][guipos];
      DF = 1;
      break;
    case 'f': //State for while the robot is in motion 
      detachInterrupt(digitalPinToInterrupt(PBL));
      detachInterrupt(digitalPinToInterrupt(PBR));
      LIF = 0;
      RIF = 0;
      tft.setCursor(10, guipos * LW + YO);
      tft.setTextColor(ST77XX_BLACK);
      tft.print(">");
      command = 'S';
      clr();
      tft.setCursor(0, YO + 0);
      tft.print("> Stop");
      DF = 0;
      break;
    case 'T': //State for the turning mode
      detachInterrupt(digitalPinToInterrupt(PBL));
      detachInterrupt(digitalPinToInterrupt(PBR));
      attachInterrupt(digitalPinToInterrupt(PBL), Tleft, FALLING);
      attachInterrupt(digitalPinToInterrupt(PBR), Tright, FALLING);
      LIF = 1;
      RIF = 1;
      clr();
      tft.setCursor(0, YO + 0);
      tft.print("Turning mode initated \nPress Left or\nRight to turnPress Fire tostop");
      command = 'B';
      DF = 0;
      break;
    default: //Default menu state
      if (DF == 0) {

        //Reset Interrupts to default state 
        if  (LIF == 1){detachInterrupt(digitalPinToInterrupt(PBL));}
        if (RIF == 1){detachInterrupt(digitalPinToInterrupt(PBR));}
        delay (100);
        attachInterrupt(digitalPinToInterrupt(PBL), left, FALLING);
        attachInterrupt(digitalPinToInterrupt(PBR), right, FALLING);
        guipos = 0;
        guipg = 0;
        command = commandtxt[guipg][guipos];
        printmenu();
      }
      DF = 1;
      break;
  }
}
void printmenu() {//Function that prints the menu updates the battery level
  clr();
  for (int i = 0; i < guilen; i++) {
    tft.setCursor(TO, YO + i * LW);
    tft.print(guitext[guipg][i]);
  }
  tft.setCursor(10, YO + guipos * LW);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(">");
  tft.drawLine(0, guilen*LW+YO,240,guilen*LW+YO, ST77XX_WHITE);
  printbl();
}

void printbl(){//Function that prints the battery level indicator
  //Values obtained from the EasyPack XL charge curve in datasheet
  int lvl = analogRead(A2);
  tft.drawRect(0, 0, 30, 18, ST77XX_WHITE);
  tft.drawRect(1, 1, 28, 16, ST77XX_WHITE);
  tft.fillRect(30, 5, 3, 8, ST77XX_WHITE);
  if (lvl > 0) {
    tft.fillRect(4, 4, 6, 10, ST77XX_WHITE);
    tft.fillRect(12, 4, 6, 10, ST77XX_BLACK);
    tft.fillRect(20, 4, 6, 10, ST77XX_BLACK);
  }
  if (lvl > 664) {
    tft.fillRect(4, 4, 6, 10, ST77XX_WHITE);
    tft.fillRect(12, 4, 6, 10, ST77XX_WHITE);
    tft.fillRect(20, 4, 6, 10, ST77XX_BLACK);
  }
  if (lvl > 845) {
    tft.fillRect(4, 4, 6, 10, ST77XX_WHITE);
    tft.fillRect(12, 4, 6, 10, ST77XX_WHITE);
    tft.fillRect(20, 4, 6, 10, ST77XX_WHITE);
  }
}

void tftsetup() {
  //Initial Setup of screen
  tft.init(240, 240);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);

  //TopGUI
  tft.drawLine(0, YO - 5, 240, YO - 5, ST77XX_WHITE); //Line Separator
  tft.drawCircle(230, 8, 8, ST77XX_WHITE); //Laser indicator circle
  tft.drawCircle(230, 8, 7, ST77XX_WHITE);
  tft.drawCircle(210, 8, 8, ST77XX_WHITE); //Acknowledgement indicator circle
  tft.drawCircle(210, 8, 7, ST77XX_WHITE);
  printbl();
  
  //Print UI menu
  guipos = 0;
  printmenu();
}
