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
dpkg -i libmavsdk*.deb
sudo ldconfig

# Install DBUS
sudo apt install -y libdbus-glib-1-dev
```

# Setup

```bash
mkdir -p colcon_ws/src
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

## Install

```
echo "source colcon_ws/install/setup.bash" >> ~/.bashrc
```

And then `source ~/.bashrc` or open a new terminal window.

# Usage

```sh
$ ros2 run autopilot-manager autopilot-manager [OPTIONS...]
  -a --file-custom-action-config	Absolute path to configuration file of the custom actions.
                                        Default: /shared_container_dir/autopilot-manager/data/custom_action/custom_action.json
  -c --file-autopilot-manager-config	Absolute path to configuration file of the overall autopilot manager service.
                                        Default: /shared_container_dir/autopilot-manager/data/config/autopilot_manager.conf
  -m --mavlink-port			MAVLink port to connect the Autopilot Manager MAVSDK instance
                                        through UDP. Default: 14570
  -h --help				Print this message
```

A ROS bool parameter named `sim` can be set when one is using a simulation environment. For that, a ROS param file can be used
or rather be passed as an argument on `ros2 run`. E.g:

```sh
ros2 run autopilot-manager autopilot-manager \
  -a install/autopilot-manager/share/autopilot-manager/data/example/custom_action/custom_action.json \
  -c install/autopilot-manager/share/autopilot-manager/data/config/autopilot_manager.conf \
  --ros-args -p sim:=true
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

*ros_cross_compile* can be used to compile the package for other architectures, like `arm64`. For that, it uses *qemu* to
start an emulation in a container of the environment we want to use to build the package. It requires also that MAVSDK
gets built for that same architecture or installed during the *ros_cross_compile* process. The `--custom-setup-script`
`--custom-data-dir` options can be used. `scripts/cross_compile_dependencies.sh` gets loaded to the container so to
and install the required dependencies to the build process. The following commands can be used to build the package for
the `arm64` (`aarch64`) arch and ROS 2 Foxy (we will be building MAVSDK and installing it from source):

```sh
# Clone MAVSDK to be built from source
git clone --recursive https://github.com/Auterion/MAVSDK.git -b main /tmp/MAVSDK
# Add COLCON_IGNORE to the MAVSDK dir so colcon doesn't build it
touch /tmp/MAVSDK/COLCON_IGNORE
# Run cross-compilation
ros_cross_compile colcon_ws/src \
  --arch aarch64 \
  --os ubuntu \
  --rosdistro foxy \
  --custom-setup-script scripts/cross_compile_dependencies.sh \
  --custom-data-dir /tmp/MAVSDK
```

The resulting package installation files can be found in `colcon_ws/install_aarch64` and can be copied to the target device,
like the Skynode. To use it, just source the `setup.bash` file inside the `install_aarch64` directory:

```
echo "source <prefix_path>/install_aarch64/setup.bash" >> ~/.bashrc
```
