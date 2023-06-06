  
#include <WiFi.h> //Library for wifi on the ESP32
#include <ros.h> //Library for ROS functionality
#include <std_msgs/String.h>

#include <HardwareSerial.h> //Library for UART serial comms

WiFiClient espClient; //WiFi Client Object

const char* ssid = "BLA-L09"; //Luka
const char* password = "12345678"; //Luka

//const char* ssid = "qinle"; //Qin
//const char* password = "qinle010108"; //Qin

//const char* ssid = "ESP32"; //Trey
//const char* password = "14531453"; //Trey

IPAddress server(192, 168, 43, 185); //Public IP of device listening Qin/Luka
//IPAddress server(192, 168, 178, 47); //Public IP of device listening Trey

const uint16_t serverPort = 11411; //ROSserial Port (default is 11411)

int WF=0;//Wifi Connection flag 1 is connected, <1 is disconnected
#define LED 2
#define LENGTH(X) (sizeof X/sizeof X[0])

HardwareSerial RCV(2); //UART port pair number

void messageCb( const std_msgs::String& Feedback) {
  RCV.write(Feedback.data); //Forward feedback to spresense
}

ros::NodeHandle nh; //Node hanlde for local ROS node
std_msgs::String str_msg;
ros::Publisher output("Team10", &str_msg); //Publisher for commands
ros::Subscriber<std_msgs::String> feedback("Feedback", &messageCb);  //Subscriber for feedback  

void setup() {
  RCV.begin(115200, SERIAL_8N1, 16, 17); //Serial to Spresense
  configureWIFI();//Configure Comms
  nh.getHardware()->setConnection(server, serverPort); //Configure ROS node socket
  nh.initNode(); 
  nh.advertise(output); //Advertise the publisher to the ROS system
  nh.subscribe(feedback); //Subcribe to the Feedback Topic
  delay(1000);
}
void loop() {
  if ((WiFi.status()==(WL_CONNECTED))&&(WF<=1)){
    RCV.write('w'); //Command that displays the connected icon on the spresense
    WF++; //Incrementing the Wifi Flag
  }else if(WiFi.status()!=(WL_CONNECTED)){
    RCV.write('q'); //Command that 
    WF--;  //Decrementing the Wifi Flag
  }
  
  if (RCV.available()) {
    char cmd = RCV.read(); //Reads the serial data from UART from the spresense
    mapcmd(cmd); //Maps the char to a string to maintain compatibility with rossiarl
    output.publish(&str_msg); //Publishes to rosserial
  }
  nh.spinOnce();
  delay(100);
}

void configureWIFI() {
  WiFi.begin(ssid, password);//Begins Wifi Connection
  while (WiFi.status() != WL_CONNECTED) {//Wait for the wifi to connect
    delay(1000);
  }
}

void mapcmd(int cmd){
  switch (cmd){
    case 'M':
      str_msg.data = "M";
      break;
    case 'S':
      str_msg.data = "S";
      break;
    case 'R':
     str_msg.data = "R";
      break;
    case 'L':
     str_msg.data = "L";
      break;
    case 'P':
     str_msg.data = "P";
     break;
    default:
    str_msg.data= "";
    break;
  }
}
