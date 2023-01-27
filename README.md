<img align="right" height="20" src="https://auterion.com/wp-content/uploads/2020/05/auterion_logo_default_sunrise.svg">

# Autopilot Manager

[![Build in Ubuntu](https://github.com/Auterion/autopilot_manager/workflows/Build%20in%20Ubuntu/badge.svg?branch=main)](https://github.com/Auterion/autopilot_manager/actions?query=workflow%3A%22Build+in+Ubuntu%22branch%3Amain) [![Build/deb packaging for Skynode and other archs](https://github.com/Auterion/autopilot_manager/workflows/Build/deb%20packaging%20for%20Skynode%20and%20other%20archs/badge.svg?branch=main)](https://github.com/Auterion/autopilot_manager/actions/workflows/build_pkg_multi_arch.yaml?query=branch%3Amain)

An AuterionOS service for higher level interactivity with onboard and flight controller components, using MAVSDK and ROS 2.

## Dependencies

_Note: These instructions assume the host system is running Ubuntu 20.04. Other OSs may be supported, but are not covered here._

1.  ROS 2 Foxy
2.  MAVSDK (C++ libs)
3.  MAVLink Router
4.  Auterion's Configuration Manager
5.  DBUS and Glib
6.  MAVSDK-Python (optional, to run the available examples)

### ROS packages and workspace library dependencies

1.  Auterion library packages:
    - `image_downsampler`
    - `landing_mapper`
    - `landing_planner`
    - `px4_msgs`
    - `ros2bagger`
    - `timing_tools`
2.  Eigen v3.3.9
3.  `rosbag2` at `foxy-future`

These packages make up the ROS2 workspace for the Autopilot Manager. The installation procedure is described later in this guide.

### ROS 2 Foxy

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
        ros-foxy-gazebo-ros-pkgs \
        ros-foxy-image-pipeline \
        ros-foxy-pybind11-vendor \
        ros-foxy-rmw-cyclonedds-cpp \
        ros-foxy-sensor-msgs \
        ros-foxy-test-msgs \
        ros-foxy-visualization-msgs
```

It is recommended that the ROS2 middleware is set to Cyclone DDS. Add the following to `~/.bashrc`:

```sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### MAVSDK

Please contact Auterion to request the `.deb` files of MAVSDK with the supported features, or to request access to the Auterion/MAVSDK repository.

#### Installation from Debian package (recommended)

```bash
dpkg -i libmavsdk*.deb
sudo ldconfig
```

#### Installation from source

```bash
git clone --recursive https://github.com/Auterion/MAVSDK.git -b main
cd MAVSDK
cmake -Bbuild/default -DCMAKE_BUILD_TYPE=Release -H.
cmake --build build/default -j$(nproc --all)
# install system-wide
sudo cmake --build build/default --target install
sudo ldconfig
```

### MAVLink Router

Please contact Auterion to obtain a `.deb` file to install it directly, or follow the instructions in
<https://github.com/mavlink-router/mavlink-router> to build and install it from source.

### Configuration Manager

Please contact Auterion to obtain a `.deb` file to install it directly, or follow the instructions
below to build and install it from source.

```bash
git clone git@github.com:Auterion/configuration-manager.git
cd configuration-manager
cmake -Bbuild -DCMAKE_BUILD_TYPE=Release -H.
cmake --build build -j$(nproc --all)
```

After that, make sure to copy `com.auterion.configuration_manager.conf` to `/usr/share/dbus-1/system.d/`
and edit it to add your system user:

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

### DBUS and Glib

```bash
sudo apt install -y libdbus-glib-1-dev
```

### MAVSDK-Python and its dependencies

```bash
pip3 install --user --upgrade mavsdk
```

## Setup

### Build

```bash
source /opt/ros/foxy/setup.bash
# Create colcon workspace
mkdir -p colcon_ws/src
cd colcon_ws
# Clone the autopilot_manager package
git clone git@github.com:Auterion/autopilot_manager.git src/autopilot_manager
# Clone the required dependencies
cd src/autopilot_manager
rosws update -t .
# Build with Release optimizations
cd ../../
colcon build --cmake-force-configure --cmake-args -DCMAKE_BUILD_TYPE=Release
```

_Note:_ If using ROS2 Galactic, replace `.rosinstall` with `.rosinstall.galactic` before running `rosws`.

#### Installing private repos

Some of the repositories listed above are private but are required components to enable the Safe Landing feature. To use Safe Landing, please contact Auterion either to request access to the necessary repositories or to obtain Debian packages to install the libraries system-wide.

### MAVLink Router config

**Warning: If one installs `mavlink-router` system-wide, one needs to take into account that it is going to route MAVLink data on the system.**
This may influence the behaviour of some MAVLink-related applications that the user might have
installed on their system. Use it cautiously, making sure that
1. one enables or disables the systemd service
   (i.e., for testing/using this application locally, enable the service with `systemctl enable mavlink-router`, and after that, disable it if not using it with `systemctl disable mavlink-router`), or
2. the `main.conf` config file has the correct configuration for all MAVLink endpoints.

To configure the needed endpoints for this application, use the following and save it under
`/etc/mavlink-router/main.conf` (or replace/extend the content of the file if it already exists on the system).
Note that this example has an endpoint configured for PX4 SITL, which should be replaced with a UART connection in
the case one is installing this on a Mission Computer connected to an autopilot through a serial port.

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
    [UdpEndpoint autopilot-manager-example-scripts]
    Mode = Normal
    Address = 127.0.0.1
    Port = 14591
```

After saving the file, restart the `mavlink-router` service with:

```bash
systemctl restart mavlink-router
```

### DBUS config

Copy the `com.auterion.autopilot_manager.conf` in the root of the repo to `/usr/share/dbus-1/system.d/`:

```bash
sudo cp com.auterion.autopilot_manager.conf /usr/share/dbus-1/system.d/
```

Then, make sure you configure `com.auterion.autopilot_manager.conf` with your user. For that, you need to edit
the file you just copied. For example:

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

### Install the Autopilot Manager

```bash
echo "source colcon_ws/install/setup.bash" >> ~/.bashrc
```

And then `source ~/.bashrc` or open a new terminal window.

## Usage

Start the Autopilot Manager with the ROS2 launch file:

```sh
ros2 launch autopilot-manager autopilot_manager.launch
```

The Autopilot Manager will be looking for autopilot heartbeats. It will exit if they are not received within a certain time after starting, so `mavlink-router` should be running and properly configured with the endpoint where the autopilot is connected.

The `configuration-manager` should also be running (either through a systemd service or started manually), since it allows the configuration of the Autopilot Manager parameters through AMC/QGC. To start it manually, use:

```bash
configuration-manager
```

If built from source and not installed system-wide:

```bash
cd configuration-manager/build/src
./configuration-manager
```

### Simulation

_Before running the Autopilot Manager, make sure that the PX4 SITL daemon, `mavlink-router` and the `configuration-manager` are running._

Launch the Autopilot Manager with the simulation-specific launch file:

```bash
ros2 launch autopilot-manager autopilot_manager_sim.launch
```

## Packaging

When cross-compiling, keep in mind that the correct toolchain for the target architecture should be installed. MAVSDK should also be installed for that architecture.

### Dependencies

```bash
sudo apt install -y  \
        git \
        qemu-user-static
sudo pip3 install ros_cross_compile
```

`ros_cross_compile` requires that the `docker` Python package is version 2, which is not the latest. Install it with:

```sh
pip3 remove docker # if a different version is already installed
pip3 install docker=2.7.0
```

### Use `ros_cross_compile`

`ros_cross_compile` can be used to compile the package for other architectures, like `arm64`. For that, it uses _qemu_ to
start an emulation in a container of the environment we want to use to build the package. It also requires that MAVSDK
is built for that architecture or installed during the `ros_cross_compile` process. The `--custom-setup-script` and
`--custom-data-dir` options can be used. `tools/packaging/cross_compile_dependencies.sh` gets loaded to the container to
install the required dependencies during the build process.

The following commands can be used to build the package for `arm64` (`aarch64`) and ROS 2 Foxy, building and installing MAVSDK from source:

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
  --custom-setup-script colcon_ws/src/autopilot_manager/tools/packaging/cross_compile_dependencies.sh \
  --custom-data-dir /tmp/MAVSDK \
  --skip-rosdep-keys Eigen3 image_downsampler landing_mapper landing_planner px4_msgs timing_tools ros2bagger rosbag2 \
  --colcon-defaults ~/colcon_ws/src/autopilot_manager/tools/packaging/defaults.yaml
```

The resulting package installation files can be found in `colcon_ws/install_aarch64` and can be copied to the target device,
like the Skynode. To use it, just source the `setup.bash` file inside the `install_aarch64` directory:

```bash
echo "source <prefix_path>/install_aarch64/setup.bash" >> ~/.bashrc
```

### Releases

Releases are automatically published by GitHub Actions whenever a tag starting with the letter `v` is created.
They create a `.zip` file containing the contents of the `install_aarch64` or `install_amd64` directories described above.

The recommended way to publish a release is simply to create a new tag on the release commit and push it to GitHub:

```bash
git tag <new-tag-name>
git push --tags
```

## Contributing

Please follow the contribution guidelines in [CONTRIBUTING](CONTRIBUTING.md).

### Code format

Ensure that `clang-format-10` is installed.

```sh
sudo apt install -y clang-format-10
```

Format the code with:

```sh
sh tools/dev/fix_style.sh .
```
