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

void messageCb( const std_msgs::String& Feedback){
  Serial.write("Received:");
  Serial.write(Feedback.data);
}

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("Team10", &str_msg);
ros::Subscriber<std_msgs::String> chitter("Feedback", &messageCb);


bool transmit=false;
/*
void IRAM_ATTR trans(){
    transmit=true; //Interrupt function for the button
}
*/
void setup() {
  Serial.begin(9600);//Serial for debugging
  RCV.begin(115200, SERIAL_8N1, 16, 17); //Serial to Spresense
  //configureWIFI();//Configure Comms
  //nh.getHardware()->setConnection(server, serverPort); //Configure ROS node socket
  //nh.initNode(); 
  //nh.advertise(chatter); 
  //nh.subscribe(chitter);
  pinMode(LED,OUTPUT);
  delay(1000);
}

void loop() {   
  if (RCV.available()){
    char cmd = RCV.read();
    if (cmd == 'M'){
      str_msg.data = "Move";
      transmit = true;
    }
    if (cmd == 'I'){
      str_msg.data = "Interact";
      transmit = true;
    }
    if (cmd == 'S'){
      str_msg.data = "Status";
      transmit = true;
    }
    else{
      transmit = false;
    }
  
  //chatter.publish(&str_msg); //Send Message
  Serial.print(str_msg.data);
  }
  delay(100);
 // nh.spinOnce();
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
