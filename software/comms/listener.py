#!/usr/bin/env python
import rospy
import paho.mqtt.client as mqtt
from std_msgs.msg import String

<<<<<<< HEAD
def on_message(client, userdata, msg):
   return True

def init():
   client = mqtt.Client()    
   client.on_message = on_message
   client.connect("192.168.43.221", 1883, 60)
=======
def onmsg(client, userdata, msg):
   command_received_from_microcontroller=true
   return command_received_from_microcontroller

client = mqtt.Client()    
client.on_message = onmsg
client.connect(192.168.43.221, 1883, 60)
>>>>>>> a7496432ef73bf16b1e33dc9eec2393f04789939

if __name__ == '__main__':
    init()
    on_message()