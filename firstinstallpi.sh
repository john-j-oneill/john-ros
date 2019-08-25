
## Change the swap file to 4GB, since I have plenty of SD space
sudo swapoff -a
sudo dd if=/dev/zero of=/swapfile bs=1M count=4096
sudo mkswap /swapfile 
sudo swapon /swapfile

## ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update

## Upgrade so everything is up to date
sudo apt-get dist-upgrade -y

## Some bug?
sudo dpkg -i --force-overwrite /var/cache/apt/archives/linux-firmware-raspi2_1.20190215-0ubuntu0.18.04.1_armhf.deb
sudo apt-get -f install

sudo apt-get install -y htop git openssh-server screen libzbar-dev libraspberrypi-dev
sudo apt-get install -y ros-melodic-desktop-full python-catkin-tools
sudo apt-get install -y ros-melodic-rosserial-arduino ros-melodic-rosserial
sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential

sudo rosdep init
rosdep update

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source /opt/ros/melodic/setup.bash

mkdir ~/catkin_ws
mkdir ~/catkin_ws/src
cd ~/catkin_ws
catkin_init_workspace
cd ~/catkin_ws/src
git clone git@github.com:john-j-oneill/john-ros.git
touch ~/catkin_ws/src/john-ros/anttweakbar_library/CATKIN_IGNORE
cd ~/catkin_ws
catkin build

# Arduino & teensy
wget https://downloads.arduino.cc/arduino-1.8.9-linuxarm.tar.xz
#TODO: Extract it! tar -something?
wget https://www.pjrc.com/teensy/49-teensy.rules
sudo cp 49-teensy.rules /etc/udev/rules.d/
wget https://www.pjrc.com/teensy/td_146/TeensyduinoInstall.linuxarm
chmod +x TeensyduinoInstall.linuxarm
./TeensyduinoInstall.linuxarm 

cd ~/Downloads/arduino-1.8.9/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .

wget https://www.realvnc.com/download/file/vnc.files/VNC-Server-6.5.0-Linux-ARM.deb
sudo dpkg -i VNC-Server-6.5.0-Linux-ARM.deb
