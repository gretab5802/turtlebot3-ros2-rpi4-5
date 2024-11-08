# turtlebot3-ros2-rpi4-5
Steps to get ROS2 on Turtlebot3 running on Raspberry Pi 4 or 5 (Ubuntu 24.04), steps to teleoperate it

# On Raspberry Pi 5
sudo nano /etc/netplan/50-cloud-init.yaml

# Should become:
network:
    ethernets:
        eth0:
            dhcp4: true
            optional: true
    version: 2
    wifis:
        wlan0:
            dhcp4: true
            access-points:
                "[network username]":
                    password: "[network password]"

###

sudo nano /etc/apt/apt.conf.d/20auto-upgrades

# Should become:
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";

###

systemctl mask systemd-networkd-wait-online.service

sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

reboot

hostname -I
## ^^ for IP address

# ssh into RPi

ssh ubuntu@[ip]

# Install Docker

# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

sudo docker run hello-world

sudo docker pull ubuntu:22.04

sudo docker run -it --device=/dev/ttyACM0 --name ubuntu_container ubuntu:22.04 /bin/bash

# Installing ROS 2 Humble
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
apt update
apt install locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
# locale is to verify, should be UTF-8

apt install software-properties-common
add-apt-repository universe
apt update
apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt update
apt upgrade
apt install ros-humble-ros-base
# 11 then 4 for time zone

# Installing Turtlebot3
# https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup
apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
apt install ros-humble-hls-lfcd-lds-driver
apt install ros-humble-turtlebot3-msgs
apt install ros-humble-dynamixel-sdk
apt install libudev-dev

apt install -y git

mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
cd ~/turtlebot3_ws/src/turtlebot3
rm -r turtlebot3_cartographer turtlebot3_navigation2
cd ~/turtlebot3_ws/
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc

## This is were the RPi 4's are freezing, 28% of the waay through 1/7
colcon build --symlink-install --parallel-workers 1

echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc

echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
source ~/.bashrc

# LDS-02 is full circle lidar, LDS-01 has a straight edge!
echo 'export LDS_MODEL=LDS-01' >> ~/.bashrc
source ~/.bashrc

# You need this too!
apt install -y ros-humble-turtlebot3-bringup

# Okay now we need to go back to host!
exit

# Do this to get container ID
sudo docker ps -a

sudo docker cp [container id]:/opt/ros/humble/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /tmp/
sudo cp /tmp/99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Still in host!
# https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/
sudo dpkg --add-architecture armhf
sudo apt update
sudo apt install libc6:armhf

export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger
rm -rf ./opencr_update.tar.bz2

wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
tar -xvf ./opencr_update.tar.bz2

cd opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr

# Back to docker!!!
sudo docker start -i ubuntu_container

# Model = burger (could be waffle)
# https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
ros2 launch turtlebot3_bringup robot.launch.py

# We need another instance!
# Open other terminal
# ssh into RPi
ssh ubuntu@[ip]

sudo docker exec -it ubuntu_container bash

ros2 run turtlebot3_teleop teleop_keyboard

# Boom!
