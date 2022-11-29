ssh ubuntu@<ip_address>
pw: turtlebot

roslaunch turtlebot3_bringup turtlebot3_robot.launch

-------------------
**On turtlebot:**
ifconfig (to check ip address)

nano ~/.bashr

scroll all the way down.

ROS_MASTER_URI, HOSTNAME, and IP should all be turtlebot's IP


**On laptop:**
MASTER_URI: turtlebot IP
HOSTNAME: laptop IP

source ~/.bashrc

-----------------------------

roslaunch <package_name> <package_name>.launch

-----------------------------------------------

sudo apt update
sudo apt-get install <package_name>
sudo apt update

sudo poweroff