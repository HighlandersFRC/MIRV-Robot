# MIRV AGX Setup Guide

This document outlines how the nvidia agx unit was setup for the MIRV project. For the purpose of this document, it is assumed that the user had an ubuntu 18 linux system available (called the "host" system).

## Flashing the Nvidia AGX
### Setting up Nvidia Jetpack
Nvidia Jetpack is used to flash nvidia products with the latest versions of software and operating systems. For MIRV we will be using Jetpack Version 4.6. To start, download Jetpack 4.6 to your linux system* [here](https://developer.nvidia.com/jetpack-sdk-46)

*You will need to be apart of the Nvidia Developer Program to download the sdkmanager. If needed create a free account with Nvidia.


Install the Jetpack on your system

```
sudo apt install ~/Downloads/sdkmanager_1.7.3-9053_amd64.deb
```


Run the nvidia SDK Manager
```
sdkmanager
```

Select Nvidia AGX as the target module and press continue
On the next page select all components related to Machine learning, Computer Vision, or CUDA. Set the Download folder and the target HW image folder. Take note of what folders are being used for downloading and hardware. Then click the checkbox for Download now, Install Later.

**Important** Make sure you only download components. Manual modifications need to be made to the image before the agx is flashed.

### Modifying AGX Kernel Clocks
In order to achieve the proper clock speed on the Nvidia AGX can bus, modifications will need to be made to the kernel hardware file which enable a higher clock speed for the canbus clock.

We are using a Neousys NRU-110v agx for this project, which ships with a few hardware modifications for the agx kernel file. To start download the updated .dtb and .dts files provided by neousys. These are located on the Neousys FTP site.

You will also need to install a dtc compiler in order to make the necessary modifications.

```
sudo apt install device-tree-compiler
```

Once the files are dowloaded, modify the downloaded dts file to use PLLAN as the parent of the CAN bus. This can be done by following the instructions under "To use PLLAON as a clock source" section of the nvidia docs [here](https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/Clocks.html?highlight=PLLAON)

Once the modifications have been made compile the dts file into a dtb 

```
dtc -O dtb -o modified_dtb.dtb NRU_JetPack4.6.1_v1.1.dts
```


```
cp modified_dtb.dtb ~/nvidia/nvidia_sdk/JetPack_4.6_Linux_JETSON_AGX_XAVIER_TARGETS/Linux_for_Tegra/kernel/dtb/tegra194-p2888-0001-p2822-0000.dtb
```

### Flash the AGX with the updated software

With the kernel modifications complete, the next step is to reflash the Nvidia board with the updated system image. Following the Neousys docs* [here](https://neousys.gitbook.io/nru-series/nru-series/3.-reflash-nru-110v-or-nru-120s) for the best way to do this.

*Note, skip the steps involving the .dtb file, this was already done previously with the modified version of the dtb file. However still copy the image file.


## Setting up ROS on nvidia agx

This procedure is very similar to the procedure for setting up ROS on a developer workstation, with some additional actions that must be performed (Exceptions are all noted below). MIRV was designed such that developer workstations mirror the robot as closely as possible to mitigate risks of software incompatibility. Please follow the developer instructions* [here](installation.md).

*Note: Skip the sections related to app development, flutter and android studio.


## Setting up Nvidia AGX Networks
### LTE Network

**Note: MIRV no longer uses the Sierra 7455 LTE module for LTE connectivity. The module is installed and available in the MIRV Rover; however, it is not currently in use and lacks a valid SIM card. The instructions here are left for legacy reference purposes and can be skipped.
The AGX units we are using are installed with a Sierra 7455 LTE module. Please follow the Neousys LTE setup guide [here](https://neousys.gitbook.io/nru-series/nru-series/peripherals/4g-lte-_-sierra-em7455-on-nru-100). When selecting contract type, there may be multiple connections with the same name, each of these maps to a different APN so make sure you selected the one the matches the desired APN. For Verizon data plans the desired APN is vzwinternet.

### Setting up Subnetworks
The NRU 110S has 4 independent ethernet ports that can each behave as their own network. This allows for incredible networking flexibility when setting up a network on the AGX. By default each of these ports will use dhcp to be assigned an IP and connect to the internet. For MIRV, each of these ports will receive a different network configuration to allow for many devices to connect to the AGX quickly and effeciently. See below

Port 1 (eth0): This port is left in the default DHCP configuration. It connects to the ethernet port located behind the battery panel on MIRV. This port is helpful for connecting MIRV to the open internet for setup and maintinence purposes.
Port 2 (eth1): This port is connected to the RUT240 LTE / WiFi router. 
Port 3 (eth2): Port 3 establishes a network for connecting with the front IP camera. 
Port 4 (eth3): Port 4 establishes a network for connecting with the rear IP camera.


Verizon LTE (ppp0): This is the LTE network configured for the integrated LTE chip on the AGX. It is no longer in use. 

To configure the AGX network connections edit the `/etc/network/interfaces` file and insert the following network configuration data.

```
sudo vim /etc/network/interfaces
```

Copy in the following configuration
```
auto eth2
iface eth2 inet static
        address 192.168.1.10
        netmask 255.255.255.0
        gateway 0.0.0.0

auto eth3
iface eth3 inet static
        address 10.0.10.1
        netmask 255.255.255.0
        gateway 0.0.0.0
    
auto eth4 inet static
        address 10.0.20.1
        netmask 255.255.255.0
        gateway 0.0.0.0
        
up route add -net 0.0.0.0 gw 0.255.80.0 dev ppp0 metric 1
```

## Setup MIRV Service
MIRV uses a systemd service to automatically start on boot. This service needs to be manually configured on the rover via the process below:
**Note in the general usecase it is better to use ros packages such as robot_upstart for this type of operation. However, robot_upstart doesn't behave nicely with python3 scripts. So a manual installation is needed instead.

Create a file in /etc/systemd/system/mirv.service
Add the following to the file
```
[Unit]
Description=MIRV Robot Program
After=network.target
StartLimitIntervalSec=0

[Service]
Type=simple
Restart=always
RestartSec=1
User=nvidia
ExecStart=/home/nvidia/mirv_ws/src/MIRV-Robot/service-start.sh

[Install]
WantedBy=multi-user.target
```

Once the file is created start the service 
```
sudo systemctl start mirv
```

Setup service to automatically start on boot
```
sudo systemctl enable mirv
```



## Setup CTRE
The Mirv Program uses the CTRE libraries [here](https://github.com/CrossTheRoadElec/Phoenix-Linux-SocketCAN-Example) for interfacing with CTRE products. This library has been integrated into the catkin_make build system and no longer needs to be compiled separately.

## Setup Pytorch
The Mirv program uses pytorch for Neural Network support. Follow the guide [here](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html) to setup pytorch on the AGX. This guide provides a lot of additional information and verification but the core installation is summarized below:
```
export TORCH_INSTALL=https://developer.download.nvidia.cn/compute/redist/jp/v461/pytorch/torch-1.11.0a0+17540c5+nv22.01-cp36-cp36m-linux_aarch64.whl
sudo python3 -m pip install --no-cache $TORCH_INSTALL
```

## Known Errors
The below section outlines some errors encountered during the above steps, and the steps taken to workaround those problems.

**Cannot install Curl**
curl may not install properly on this version of ubuntu, if this is the case you will receive an error message like the following:

```
The following packages have unmet dependencies:
 curl : Depends: libcurl4 (= 7.58.0-2ubuntu3) but 7.58.0-2ubuntu3.16 is to be installed
```

This error is caused by an invalid version of curl being shipped with Ubuntu 18. Run the below to fix the installation
```
sudo apt remove libcurl4
sudo apt install lubcurl4 curl
```

**Cannot install ROS**
ros-melodic-desktop-full may fail to install on the agx or other systems. This is fundamentally caused because Nvidia has hard locked certain packages to a version that is incompatible with ros melodic. There are a few solutions I have seen work, the first is to compile ROS from source code on the unit. The second is to use aptitude to downgrade incompatible packages then install everything. Neousys has some documentation on how to do this [here](https://neousys.gitbook.io/nru-series/nru-series/2.-software-related/ros-installation).

**Cannot install libsdl2-dev**
This error occures when the package manager cannot find or install libsdl2-dev. The solution is to compile libsdl2 from source. Install from source using the instruction described below:

```
git clone https://github.com/libsdl-org/SDL
cd SDL
mkdir build
cd build
../configure
make
sudo make install
```

Reference: https://wiki.libsdl.org/Installation

