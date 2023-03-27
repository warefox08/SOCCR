#include <WiFi.h> //Library for wifi on the ESP32
#include <ros.h>
#include <std_msgs/String.h>
#include <HardwareSerial.h>

WiFiClient espClient; //WiFi Client Obj
const char* ssid = "ESP32";
const char* password = "14531453";
IPAddress server(192,168,175,124); //Public IP of device listening
const uint16_t serverPort = 11411; //Default ROS Port

#define LED 2

HardwareSerial RCV(2);

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("Team10", &str_msg);
//ros::Subscriber<std_msgs::Empty> chitter("Acks", &str_msg);

bool transmit=false;
/*
void IRAM_ATTR trans(){
    transmit=true; //Interrupt function for the button
}
*/
void setup() {
  Serial.begin(9600);//Serial for debugging
  RCV.begin(115200, SERIAL_8N1, 16, 17); //Serial to Spresense
  configureWIFI();//Configure Comms
  nh.getHardware()->setConnection(server, serverPort); //Configure ROS node socket
  nh.initNode(); 
  nh.advertise(chatter); 
  pinMode(LED,OUTPUT);
  delay(1000);
}

void loop() {   
  if (RCV.available()){
    char cmd = RCV.read();
    transmit = true;
    if (cmd = 'M'){
      str_msg.data = "Move";
    }
    if (cmd = 'I'){
      str_msg.data = "Interact";
    }
    if (cmd = 'S'){
      str_msg.data = "Status";
    }
    else{
      transmit = false;
    }
  }
  if (transmit){
    chatter.publish(&str_msg); //Send Message
    //Serial.print(received);
    Serial.print("Sent"); 
    transmit=false;
    delay(100);
    str_msg.data = "";
    }
  nh.spinOnce();
}
void configureWIFI() {
  WiFi.begin(ssid, password);//Begins Wifi Connection
  while (WiFi.status() != WL_CONNECTED) {//Block Until Connected
    delay(1000);
    Serial.print("."); 
  }
  Serial.println("Connection Established at:");
  Serial.println(WiFi.localIP());
  digitalWrite(LED,HIGH);
}
