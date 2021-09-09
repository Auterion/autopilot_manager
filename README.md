<img align="right" height="20" src="https://auterion.com/wp-content/uploads/2020/05/auterion_logo_default_sunrise.svg">

# Autopilot Manager

[![Build in Ubuntu](https://github.com/Auterion/autopilot_manager/workflows/Build%20in%20Ubuntu/badge.svg?branch=main)](https://github.com/Auterion/autopilot_manager/actions?query=workflow%3A%22Build+in+Ubuntu%22branch%3Amain) [![Build/deb packaging for Skynode and other archs](https://github.com/Auterion/autopilot_manager/workflows/Build/deb%20packaging%20for%20Skynode%20and%20other%20archs/badge.svg?branch=main)](https://github.com/Auterion/autopilot_manager/actions/workflows/build_pkg_multi_arch.yaml?query=branch%3Amain)

An AuterionOS service for higher level interactivity with onboard and flight controller components, using MAVSDK and ROS 2.

# Dependencies

_Note: The host system is considered to run Ubuntu 20.04 Focal. Other OS's might be supported, but the provided instructions only cover this OS._

1.  ROS 2 Foxy
2.  MAVSDK
3.  `mavlink-router`
4.  Auterion's `configuration-manager`
5.  DBUS and Glib
6.  `pymavlink` (optional, to run the available examples)

## ROS packages and workspace library dependencies

1.  Auterion's `landing_mapper` and `image_downsampler` library packages
2.  microRTPS bridge, composed by the `px4_msgs` and `px4_ros_com` packages
3.  Eigen v3.3.9

## ROS 2 Foxy

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
        lsb-release \
        python3-pip \
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Foxy packages
sudo apt update
sudo apt install -y python3-colcon-common-extensions \
        ros-foxy-ros-base \
        ros-foxy-sensor-msgs \
        ros-foxy-visualization-msgs\
        ros-foxy-image-pipeline
```

## MAVSDK

**Note: please request to Auterion the deb files of MAVSDK with the supported features, or request the access to the Auterion/MAVSDK repository**

### Installation from deb (recommended)

```bash
dpkg -i libmavsdk*.deb
sudo ldconfig
```

### Installation from source

```bash
git clone --recursive https://github.com/Auterion/MAVSDK.git -b main
cd MAVSDK
cmake -Bbuild/default -DCMAKE_BUILD_TYPE=Release -H.
cmake --build build/default -j$(nproc --all)
# install system-wide
sudo cmake --build build/default --target install
sudo ldconfig
```

## mavlink-router

Please request to Auterion a deb file to install it directly, or follow the instructions in
<https://github.com/mavlink-router/mavlink-router> to build it and install it from source.

## configuration-manager

Please request to Auterion a deb file to install it directly, or follow the instructions in
bellow to build it and install it from source.

```bash
git clone git@github.com:Auterion/configuration-manager.git
cd configuration-manager
cmake -Bbuild -DCMAKE_BUILD_TYPE=Release -H.
cmake --build build -j$(nproc --all)
```

After that, make sure to copy `com.auterion.configuration_manager.conf` to `/usr/share/dbus-1/system.d/`
and edit it in order to add your system user:

```xml
<!DOCTYPE busconfig PUBLIC
 "-//freedesktop//DTD D-BUS Bus Configuration 1.0//EN"
 "http://www.freedesktop.org/standards/dbus/1.0/busconfig.dtd">
<busconfig>
  <policy user="root">
    <allow own="com.auterion.configuration_manager"/>
    <allow send_destination="com.auterion.configuration_manager"
        send_interface="com.auterion.configuration_manager.interface"/>
  </policy>
  <!-- Add your user here -->
  <!-- example user -->
  <policy user="user1"> <!-- replace "user1" with your system username -->
    <allow own="com.auterion.configuration_manager"/>
    <allow send_destination="com.auterion.configuration_manager"
        send_interface="com.auterion.configuration_manager.interface"/>
  </policy>
  <!--**************-->
</busconfig>
```

## DBUS and Glib

```bash
sudo apt install -y libdbus-glib-1-dev
```

# pymavlink and its dependencies

```bash
pip3 install --user --upgrade pymavlink
```

# Setup

## Build

```bash
source /opt/ros/foxy/setup.bash
# Create colcon workspace
mkdir -p colcon_ws/src
cd colcon_ws
# Clone the autopilot-manager package
git clone git@github.com:Auterion/autopilot_manager.git src/autopilot_manager
# Clone the required dependencies
git clone https://gitlab.com/libeigen/eigen.git -b 3.3.9 src/eigen
git clone git@github.com:Auterion/image_downsampler.git src/image_downsampler
git clone git@github.com:Auterion/landing_mapper.git -b develop src/landing_mapper
git clone git@github.com:Auterion/px4_msgs.git -b develop src/px4_msgs
# Build with Release optimizations
colcon build --cmake-force-configure --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## mavlink-router config

_Warning: if you one installs mavlink-router system-wide, it needs to take into account that it is going to route MAVLink data on the system._
This might influence the behavior of some of some MAVLink-related applications that the user might have
installed on its system. Use it cautiously, making sure that 1. one enables or disables the systemd service,
i.e., for testing/using this application locally, enable the service with `systemctl enable mavlink-router`,
and after that, disable it if not using it with `systemctl disable mavlink-router`, or 2. make sure that the
`main.conf` config file has the correct configuration for all MAVLink endpoints.

To configure the needed endpoints to use with this application, use the following and save it under
`/etc/mavlink-router/main.conf` (or replace/extend the content of the file if it already exists on the system).
Note that this one has an endpoint configured for PX4 SITL, which should be replaced with a UART connection in
the case one is installing this on a Mission Computer connect to an autopilot through a serial port.

```
    [General]
    TCPServerPort = 4
    ReportStats = False
    DebugLogLevel= info
    MavlinkDialect = common
    Log = /data/log/flight-stack
    LogMode = while-armed
    MinFreeSpace = 0
    MaxLogFiles=50

    [UdpEndpoint px4-sitl]
    Mode = Eavesdropping
    Address = 0.0.0.0
    Port = 14540

    [UdpEndpoint configuration-manager]
    Mode = Normal
    Address = 127.0.0.1
    Port = 14542

    [UdpEndpoint groundcontrol]
    Mode = Normal
    Address = 127.0.0.1
    Port = 14550

    [UdpEndpoint autopilot-manager]
    Mode = Normal
    Address = 127.0.0.1
    Port = 14590

    # autopilot-manager example scripts
    [UdpEndpoint pymavlink]
    Mode = Normal
    Address = 127.0.0.1
    Port = 14561
```

After saving the file, restart the `mavlink-router` service with:

```bash
systemctl restart mavlink-router
```

## DBUS config

Copy the `com.auterion.autopilot_manager.conf` on the root of the repo to `/usr/share/dbus-1/system.d/`:

```bash
sudo cp com.auterion.autopilot_manager.conf /usr/share/dbus-1/system.d/
```

Then, make sure you configure `com.auterion.autopilot_manager.conf` with your user. For that, you need to edit
the file you just copied. Example bellow:

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
  <!-- Add your user here -->
  <!-- example user -->
  <policy user="user1"> <!-- replace "user1" with your system username -->
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

## Install the autopilot-manager

```bash
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
                                        through UDP. Default: 14590
  -h --help				Print this message
```

The autopilot-manager will be looking for autopilot HEARTBEATs, which will result and timeout and exit if they are not received.
So `mavlink-router` should be running and properly configured with the endpoint where the autopilot is connected.

The `configuration-manager` should also be running (being it through a systemd service or started manually locally), since it is
the service that allows the configuration of the Autopilot Mnager parameters through AMC/QGC. To start it manually, use:

```bash
configuration-manager
```

If built from source and not installed system-wide:

```bash
cd configuration-manager/build/src
./configuration-manager
```

## Simulation

A ROS bool parameter named `sim` can be set when one is using a simulation environment. For that, a ROS param file can be used
or rather be passed as an argument on `ros2 run`.

**Note: Before running the autopilot-manager, make sure that the PX4 SITL daemon, `mavlink-router` and the `configuration-manager` are running.**

#### Terminal 1

```bash
source colcon_ws/install/setup.bash
ros2 launch autopilot-manager static_tf.launch
```

#### Terminal 2

```bash
source colcon_ws/install/setup.bash
cd colcon_ws
ros2 run autopilot-manager autopilot-manager \
  -a install/autopilot-manager/share/autopilot-manager/data/example/custom_action/custom_action_sitl.json \
  -c install/autopilot-manager/share/autopilot-manager/data/config/autopilot_manager.conf \
  --ros-args -p sim:=true
```

# Packaging

Keep in mind that when cross-compiling, the correct toolchain should be installed. Also, MAVSDK should be also installed
for that same target.

## Dependencies

```bash
sudo apt install -y  \
        git \
        qemu-user-static
sudo pip3 install ros_cross_compile
```

## Use ros_cross_compile

_ros_cross_compile_ can be used to compile the package for other architectures, like `arm64`. For that, it uses _qemu_ to
start an emulation in a container of the environment we want to use to build the package. It requires also that MAVSDK
gets built for that same architecture or installed during the _ros_cross_compile_ process. The `--custom-setup-script`
`--custom-data-dir` options can be used. `scripts/cross_compile_dependencies.sh` gets loaded to the container so to
and install the required dependencies to the build process. The following commands can be used to build the package for
the `arm64` (`aarch64`) arch and ROS 2 Foxy (we will be building MAVSDK and installing it from source):

```bash
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

```bash
echo "source <prefix_path>/install_aarch64/setup.bash" >> ~/.bashrc
```

# Contributing

## Code format

One requires clang-format-10 to format the code. Use the following command to format it:

```sh
sh tools/fix_style.sh .
```
