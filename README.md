# MIRV Development Environment Installation Guide

The below instructions outline the process for setting up a development environment for the MIRV Project. The instructions below outline how to setup a development environment for Ubuntu 18 and ROS melodic. While newer versions of both Ubuntu and ROS are available, these software versions were chosen to maintain compatibility with the current software build available for the Nvidia AGX unit. 

## General Dependencies 
### Core Packages
Install the following packages from the command line
```
sudo apt-get update
sudo apt-get install vim tmux git build-essential curl
```

Install Google Chrome here*: https://www.google.com/chrome/
*This is actually required, as some other software dependends on the chrome runtime to operate.
### Visual Studio Code
Download the VS Code installation package here: https://code.visualstudio.com/download
Install VS Code below
```
sudo apt install ./<download-name>.deb
```
Verify your installation by typing the following in a terminal
```
code
```

More information can be found here: https://code.visualstudio.com/docs/setup/linux
	

## Installing ROS (for rover code development)
Add Ros Packages to sources.list
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
Setup Package keys
```
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
Update Apt
```
sudo apt update
```

Install ROS Melodic
```
sudo apt install ros-melodic-desktop-full
```

Add ROS to system environment
```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Install ROS python dependencies
```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool
```

Initialize ROS
```
sudo rosdep init
rosdep update
```
For more information see ROS installation guide here: http://wiki.ros.org/melodic/Installation/Ubuntu

## Installing Android Studio and Flutter (for app code development)
### Installing Android Studio
Download Android Studio for Linux here: https://developer.android.com/studio Then extract android studio in to install it.
```
cd Downloads
tar -xaf android-studio-2021.1.1.23-linux.tar.gz
```
I would recommend moving the android studio sources to someplace besides the download directory on your system, then adding android studio to your system path for convienience.
```
sudo cp -r android-studio /opt
echo "export PATH=\"/opt/android-studio/bin:$PATH\"" >> ~/.bashrc
```

Once this is complete you can launch Android studio using the following
```
studio.sh
```
After launching android studio, click through the menu's and install the android 30 SDK


### Installing Flutter
Install Flutter using snapd
```
sudo snap install flutter --classic
flutter sdk-path
```

Verify that you have all parts of flutter installed by running
```
flutter doctor
```

Once flutter is installed add the flutter plugin to android studio
Launch android studio
```
studio.sh
```

On the left hand menu select plugins, then search for the flutter plugin. Click the flutter plugin, then click install. This will prompt you to also install the dart plugin, install that as well then restart android studio.


## Other Dependencies

