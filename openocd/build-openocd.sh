#!/bin/sh
# installsh: install everything required for OpenOCD on a rPI
# Copyright (C) 2017 Aleksa Sarai <cyphar@cyphar.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

set -ex

SRC="$(mktemp -d --tmpdir openocd-src.XXXXXX)"

# Update the system.
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update
sudo apt-get upgrade -y

# Install OpenCAD's dependencies.
sudo apt-get install -y git autoconf libtool make pkg-config libusb-1.0-0 libusb-1.0-0-dev

# Download ...
git clone git://git.code.sf.net/p/openocd/code "$SRC"
cd "$SRC"

# ... build ...
./bootstrap
./configure --enable-sysfsgpio --enable-bcm2835gpio
make -j"$(nproc)"

# ... and install OpenOCD.
sudo make install
