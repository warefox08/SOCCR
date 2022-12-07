
#include <WiFi.h> //Library for wifi on the ESP32
#include <ros.h>
#include <std_msgs/String.h>

//#include <PubSubClient.h> //Library for MQTT and Publish and Subscribe 
WiFiClient espClient; //WiFi Client
//PubSubClient client(espClient); //Attaching the MQTTClient to the WiFi client
const char* ssid = "ESP32";
const char* password = "14531453";
//const char* mqtt_server = "192.168.215.183";
IPAddress server(192,168,0,11);
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);


bool transmit=false;
void IRAM_ATTR trans(){
    transmit=true;
}
void setup() {
  Serial.begin(115200);//Serial for debugging
  configureWIFI();//Configure Comms
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  //configureMQTT();
  pinMode(15, INPUT_PULLUP);
  attachInterrupt(15, trans, FALLING);
  delay(1000);
}

void loop() {
  str_msg.data = "1";
  if (transmit){
    chatter.publish(&str_msg);
    Serial.print("Sent"); 
    transmit=false;
    }
  //client.loop();
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

/*void configureMQTT() {
  client.setServer(mqtt_server, 1883);//Begins MQTT Connection
  while (!client.connected()) {//Block Until Connected
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe("esp32/#");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(1000);
    }
  }
}*/
