<img align="right" height="20" src="https://auterion.com/wp-content/uploads/2020/05/auterion_logo_default_sunrise.svg">

# Autopilot Manager
[![Build in Ubuntu](https://github.com/Auterion/autopilot_manager/workflows/Build%20in%20Ubuntu/badge.svg?branch=main)](https://github.com/Auterion/autopilot_manager/actions?query=workflow%3A%22Build+in+Ubuntu%22) [![Build/deb packaging for Skynode and other archs](https://github.com/Auterion/autopilot_manager/workflows/Build/deb%20packaging%20for%20Skynode%20and%20other%20archs/badge.svg?branch=develop)](https://github.com/Auterion/autopilot_manager/actions?query=workflow%3A%22Build+for+Skynode+with+cross-compilation%22)

An AuterionOS service for higher level interactivity with onboard and flight controller components, using Auterion SDKs and MAVSDK.

# Dependencies

1.  MAVSDK
1.  DBUS
1.  GLib

```bash
apt install libdbus-glib-1-dev
```

# Build

```bash
cmake -Bbuild -DCUSTOM_ACTION_DATA_DIR=<desired_path_to_install_data> -S.
cmake --build build -j$(nproc --all)
```

To install system-wide:
```bash
cmake --build build -j$(nproc --all) -- target install
```

# Usage

```bash
$ autopilot-manager [OPTIONS...]
  -f --file-custom-action-config	 Absolute path to configuration file of the custom actions.
                                         Default: /usr/src/app/autopilot-manager/data/example/custom_action/custom_action.json
  -m --mavlink-port			 MAVLink port to connect the Autopilot Manager MAVSDK instance
                                         through UDP. Default: 14570
  -h --help				 Print this message
```

# Packaging

Keep in mind that when cross-compiling, the correct toolchain should be installed. Also, MAVSDK should be also installed
for that same target.

```sh
./tools/generate_debian_changelog.sh > debian/changelog;
dpkg-buildpackage -us -uc -nc -b --host-arch <desired_arch>
```
