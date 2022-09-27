#!/bin/bash

set -e

apt update && apt install -y colordiff \
                                git \
                                libdbus-glib-1-dev


PWD=$(pwd)

# Install MAVSDK from source
# used by ros_cross_compile
cd custom-data \
        && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_SHARED_LIBS=OFF -Bbuild/default -H. \
        && cmake --build build/default --target install -j`nproc --all` \
        && ldconfig

cd ${PWD} \
	&& rm -rf /tmp/MAVSDK \
	&& rm -rf custom-data
