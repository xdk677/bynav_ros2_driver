#!/bin/bash

INSTALLATION_METHOD=$1

function print_usage() {
  echo "usage: ./install_dependencies.sh method"
  echo "method: online 或者 offline"
}

function install_online() {
  echo "install_online"
  sudo apt install ros-${ROS_VERSION}-nmea-msgs -y
  sudo apt install ros-${ROS_VERSION}-gps-msgs -y
  sudo apt install libgeographic-dev -y
}

function install_offline() {
  echo "install_offline"
  dir_name="dependencies/"$1
  if [ $1 = "foxy" ]; then
    sudo dpkg -i $dir_name"/ros-foxy-gps-msgs_1.0.9-1focal.20230609.185534_amd64.deb"
    sudo dpkg -i $dir_name"/ros-foxy-nmea-msgs_2.0.0-1focal.20230527.050446_amd64.deb"
    sudo dpkg -i $dir_name"/libgeographic19_1.50.1-1build1_amd64.deb"
    sudo dpkg -i $dir_name"/libgeographic-dev_1.50.1-1build1_amd64.deb"
  elif [ $1 = "galactic" ]; then
    sudo dpkg -i $dir_name"/ros-galactic-gps-msgs_1.0.5-1focal.20221203.095923_amd64.deb"
    sudo dpkg -i $dir_name"/ros-galactic-nmea-msgs_2.0.0-3focal.20221203.095843_amd64.deb"
    sudo dpkg -i $dir_name"/libgeographic19_1.50.1-1build1_amd64.deb"
    sudo dpkg -i $dir_name"/libgeographic-dev_1.50.1-1build1_amd64.deb"
  else
    sudo dpkg -i $dir_name"/ros-humble-gps-msgs_2.0.3-1jammy.20240217.052941_amd64.deb"
    sudo dpkg -i $dir_name"/ros-humble-nmea-msgs_2.0.0-4jammy.20240217.052751_amd64.deb"
    sudo dpkg -i $dir_name"/libgeographic19_1.52-1_amd64.deb"
    sudo dpkg -i $dir_name"/libgeographic-dev_1.52-1_amd64.deb"
  fi
}

if [ -z $INSTALLATION_METHOD ]; then
  print_usage
  exit
fi

if [ $INSTALLATION_METHOD != "online" -a $INSTALLATION_METHOD != "offline" ]; then
  print_usage
  exit
fi

ROS_VERSION=`printenv ROS_DISTRO`
echo "ROS version: ${ROS_VERSION}"
if [ -z $ROS_VERSION ]; then
  echo "ROS env not setup"
  exit
fi

if [ $INSTALLATION_METHOD = "online" ]; then
  install_online
else
  install_offline $ROS_VERSION
fi
