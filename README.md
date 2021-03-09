<img align="right" height="20" src="https://auterion.com/wp-content/uploads/2020/05/auterion_logo_default_sunrise.svg">

# Autopilot Manager
[![Build in Ubuntu](https://github.com/Auterion/autopilot_manager/workflows/Build%20in%20Ubuntu/badge.svg?branch=main)](https://github.com/Auterion/autopilot_manager/actions?query=workflow%3A%22Build+in+Ubuntu%22branch%3Amain) [![Build/deb packaging for Skynode and other archs](https://github.com/Auterion/autopilot_manager/workflows/Build/deb%20packaging%20for%20Skynode%20and%20other%20archs/badge.svg?branch=main)](https://github.com/Auterion/autopilot_manager/actions/workflows/build_pkg_multi_arch.yaml?query=branch%3Amain)

An AuterionOS service for higher level interactivity with onboard and flight controller components, using Auterion SDKs and MAVSDK.

# Dependencies

*Note: The host system should run Ubuntu 20.04 Focal*

1.  ROS 2 Foxy
1.  MAVSDK
1.  DBUS and Glib

```bash
# Set locale
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup ROS 2 sources
sudo apt install -y curl \
        gnupg2 \
        lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Foxy packages
sudo apt update
sudo apt install -y python3-colcon-common-extensions \
        ros-foxy-ros-base \
        ros-foxy-sensor-msgs \
        ros-foxy-image-pipeline

# Setup environment
source /opt/ros/foxy/setup.bash

# Install MAVSDK
dpkg -i libmavsdk0_0.37.0-1_amd64.deb
sudo ldconfig

# Install DBUS
sudo apt install -y libdbus-glib-1-dev
```

# Setup

```bash
mkdir -p colcon_ws/rc
cd colcon_ws/src
git clone git@github.com:Auterion/autopilot_manager.git
cd ..
colcon build
```

## DBUS config

Before installing it, make sure you configure `com.auterion.autopilot_manager.conf` with your user:

```xml
<!DOCTYPE busconfig PUBLIC
 "-//freedesktop//DTD D-BUS Bus Configuration 1.0//EN"
 "http://www.freedesktop.org/standards/dbus/1.0/busconfig.dtd">
<busconfig>
  <policy user="root">
    <allow own="com.auterion.autopilot_manager"/>
    <allow send_destination="com.auterion.autopilot_manager"
        send_interface="com.auterion.autopilot_manager.interface"/>
  </policy>
  <!-- example user -->
  <policy user="user1">
    <allow own="com.auterion.autopilot_manager"/>
    <allow send_destination="com.auterion.autopilot_manager"
        send_interface="com.auterion.autopilot_manager.interface"/>
  </policy>
  <!--**************-->
  <policy context="default">
        <deny own="com.auterion.autopilot_manager"/>

        <deny send_destination="com.auterion.autopilot_manager"/>

        <!-- Basic D-Bus API stuff -->
        <allow send_destination="com.auterion.autopilot_manager"
                send_interface="org.freedesktop.DBus.Introspectable"/>
        <allow send_destination="com.auterion.autopilot_manager"
                send_interface="org.freedesktop.DBus.Properties"/>
        <allow send_destination="com.auterion.autopilot_manager"
                send_interface="org.freedesktop.DBus.ObjectManager"/>
  </policy>
</busconfig>
```

## Install system-wide

TODO

# Usage

```bash
$ ros2 run autopilot-manager autopilot-manager [OPTIONS...]
  -a --file-custom-action-config	Absolute path to configuration file of the custom actions.
                                        Default: /usr/src/app/autopilot-manager/data/example/custom_action/custom_action.json
  -c --file-autopilot-manager-config	Absolute path to configuration file of the overall autopilot manager service.
                                        Default: /shared_container_dir/autopilot_manager.conf
  -m --mavlink-port			MAVLink port to connect the Autopilot Manager MAVSDK instance
                                        through UDP. Default: 14570
  -h --help				Print this message

```

# Packaging

Keep in mind that when cross-compiling, the correct toolchain should be installed. Also, MAVSDK should be also installed
for that same target.

## Dependencies

```sh
sudo apt install -y  \
        git \
        python3-pip \
        qemu-user-static
sudo pip3 install ros_cross_compile
```

## Use ros_cross_compile

TODO
