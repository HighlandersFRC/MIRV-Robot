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

This procedure is very similar to the procedure for setting up ROS on a developer workstation, with some additional actions that must be performed (Exceptions are all noted below). MIRV was designed such that developer workstations mirror the robot as closely as mitigate risks of software compatibility. Please follow the developer instructions* [here](installation.md).

*Note: Skip the sections related to app development, flutter and android studio.

## Setup CTRE

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

