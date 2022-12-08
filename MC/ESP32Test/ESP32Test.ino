
#include <WiFi.h> //Library for wifi on the ESP32
#include <ros.h>
#include <std_msgs/String.h>

WiFiClient espClient; //WiFi Client Obj
const char* ssid = "ESP32";
const char* password = "14531453";
IPAddress server(192,168,215,124); //Public IP of device listening
const uint16_t serverPort = 11411; //Default ROS Port

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("Team10", &str_msg);


bool transmit=false;
void IRAM_ATTR trans(){
    transmit=true; //Interrupt function for the button
}
void setup() {
  Serial.begin(115200);//Serial for debugging
  configureWIFI();//Configure Comms
  nh.getHardware()->setConnection(server, serverPort); //Configure ROS node socket
  nh.initNode(); 
  nh.advertise(chatter);
  pinMode(15, INPUT_PULLUP);
  attachInterrupt(15, trans, FALLING);
  delay(1000);
}

void loop() {
  str_msg.data = "1";
  if (transmit){
    chatter.publish(&str_msg); //Send Message
    Serial.print("Sent"); 
    transmit=false;
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
}
