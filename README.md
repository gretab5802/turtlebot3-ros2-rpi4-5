# ROS2 Humble (Ubuntu 22.04) on Raspberry Pi 4 or 5 (Ubuntu 24.04) through docker container, Turtlebot3 teleoperation
Steps to get ROS2 on Turtlebot3 running on Raspberry Pi 5 (Ubuntu 24.04) and then to teleoperate it

## Prequisites:
# Hardware needed:
* Raspberry Pi 4 or 5
* Turtlebot3 with OpenCR controller
+ Hardware assembly for Turtlebot3: https://emanual.robotis.com/docs/en/platform/turtlebot3/hardware_setup/#hardware-assembly
* Power cord for OpenCR board
* Monitor with HDMI cable -> Either HDMI to micro HDMI, or HDMI to HDMI with a micro HDMI adapter (monitor uses HDMI, Raspberry Pi uses micro HDMI)
* USB keyboard
* Micro SD card (32 GB or greater, I think 16 GB is too small)
* A way for your computer to flash the micro SD card (either a micro SD slot built in to the computer or a USB to micro SD card adapter)

# Setting up the OS
* Flash SD card with Raspberry Pi OS
+ Raspberry Pi OS: https://www.raspberrypi.com/software/
* **Device**: Raspberry Pi 4 or 5 (whichever you are using)
* **Operating System**: Other General Purpose OS -> Ubuntu -> Ubuntu Server 24.04.1 LTS (64-bit)
* **Storage**: SD card
* When flashing OS onto SD card, apply customization settings to set username and password when prompted. This will be the login information you need to sign into the Raspberry Pi when connecting it to a monitor/keyboard and when SSHing into it

## On Raspberry Pi 5
Plug in power to OpenCR board, HDMI from monitor to Raspberry Pi, and keyboard into Raspberry Pi. You should get a login prompt to which you will put in whatever username and password you flashed on to the SD card when you set it up. If the monitor says something along the lines of "Frequency not supported", try switching between the two micro HDMI slots in the Pi and turning the board off and then on again.
```
sudo nano /etc/netplan/50-cloud-init.yaml
```
**Should display**
```
network:
    ethernets:
        eth0:
            dhcp4: true
            optional: true
    version: 2
```
**Add the following, make sure you change [network username] and [network password] to your own network username and password**
<pre>
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
                "<b>[network username]</b>":
                    password: "<b>[network password]</b>"
</pre>

Next:
```
sudo nano /etc/apt/apt.conf.d/20auto-upgrades
```
**Should display**
```
APT::Periodic::Update-Package-Lists "1";
APT::Periodic::Unattended-Upgrade "1";
```
**Change the 1's to 0's:**
```
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";
```

Next:
```
systemctl mask systemd-networkd-wait-online.service

sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

reboot
```

The Raspberry Pi will reboot and ask for login information again. Then, to get its IP address:
```
hostname -I
```

## On a separate computer
**ssh into RPi, replace _[ip]_ with the ip you just got from hostname -I**
<pre>
ssh ubuntu@<b>[ip]</b>
</pre>

## Install Docker
**Add Docker's official GPG key:**
```
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```
**Add the repository to Apt sources:**
```
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

sudo docker run hello-world
```
**_Make sure this runs successfully, hello-world is our test for this_**
Time to get Ubuntu 22.04!
```
sudo docker pull ubuntu:22.04
```

**For the following, including --device=/dev/ttyACM0 is essential to make sure we can access the USB port where the Raspberry Pi connects to the OpenCR controller board later**
```
sudo docker run -it --device=/dev/ttyACM0 --name ubuntu_container ubuntu:22.04 /bin/bash
```

## Installing ROS 2 Humble
**https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html**
```
apt update
apt install locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
# locale is to verify, should be UTF-8
```

```
apt install software-properties-common
add-apt-repository universe
apt update
apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt update
apt upgrade
```

Here, we install ROS2 Humble. Base version is all we need for the Turtlebot3s
```
apt install ros-humble-ros-base
```

## Installing Turtlebot3
**https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup**
```
apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
apt install ros-humble-hls-lfcd-lds-driver
apt install ros-humble-turtlebot3-msgs
apt install ros-humble-dynamixel-sdk
apt install libudev-dev
apt install -y git
```

```
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
cd ~/turtlebot3_ws/src/turtlebot3
rm -r turtlebot3_cartographer turtlebot3_navigation2
cd ~/turtlebot3_ws/
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## Stop!
**This next part is different for Raspberry Pi 4 and 5's.** If you are doing this on a Pi 5, you can skip the next few steps and [continue](#continue)
For Pi 4's, they are running much slower for this next step than the Pi 5's and are sometimes never completing it. To try and mitigate this, there are some things we can do.
First, to help the Pi 4 if it runs out of RAM, we can create a swap file to provide additional virtual memory. To do this, we must exit docker momentarily.
```
exit
```
```
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo swapon --show
```
Something like this should show:
<pre>
NAME      TYPE SIZE USED PRIO
/swapfile file   4G   0B   -2
</pre>
Now we can go back into our docker container
```
sudo docker start -i ubuntu_container
```
After this, you can try and run the next command
```
colcon build --symlink-install --parallel-workers 1
```
If that doesn't work, I had some success getting rid of the parallel-workers argument (defaults to 0, this means colcon will automatically detect and use all available CPU cores for parallel building)
```
colcon build --symlink-install
```
For the first Pi 4, I did both steps - adding virtual memory and then running the colcon build command with no parallel-workers parameter - and it completed in a few minutes. I did these same steps for a second Pi 4 and it took **16 minutes** to complete, so be patient!

## Continue
**Continuing on for Pi 5s. If you are working with a Pi 4, only run this if you did not previously run colcon build**
```
colcon build --symlink-install --parallel-workers 1
```
**Okay, the rest of the next steps apply to Pi 4s and 5s no matter what**
```
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc

echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
source ~/.bashrc
```

**Make sure you replce _[LDS-01 or LDS-02]_ with whichever lidar you actually have. Lidar 1 has a slight edge on it whereas Lidar 2 is a full circle**
<pre>
echo 'export LDS_MODEL=<b>[LDS-01 or LDS-02]</b>' >> ~/.bashrc
</pre>
```
source ~/.bashrc
```

You need this too!
```
apt install -y ros-humble-turtlebot3-bringup
```

Now we need to go back to host! Still SSHed in but no longer in the Docker container
```
exit
```

Get container ID
```
sudo docker ps -a
```

**Replace _[container id]_ with the container id you just got above!**
<pre>
sudo docker cp <b>[container id]</b>:/opt/ros/humble/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /tmp/
</pre>
```
sudo cp /tmp/99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Still in host!
**https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/**
```
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
```

Back to docker!!!
```
sudo docker start -i ubuntu_container
```

Here we export the model of our Turtlebot, change **burger** to **waffle** if that is the model you are using
**https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup**
```
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
ros2 launch turtlebot3_bringup robot.launch.py
```

We need another terminal! Open other one and ssh into RPi
<pre>
ssh ubuntu@<b>[ip]</b>
</pre>

In the new terminal:
```
sudo docker exec -it ubuntu_container bash
```
```
ros2 run turtlebot3_teleop teleop_keyboard
```
# Boom!
