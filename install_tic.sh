#!/bin/bash

tic_software=https://www.pololu.com/file/0J1348/pololu-tic-1.8.1-linux-x86.tar.xz
wget $tic_software
tar -xvf pololu-tic-*.tar.xz
sudo pololu-tic-*/install.sh

sudo apt-get install build-essential git cmake libudev-dev qtbase5-dev
git clone https://github.com/pololu/libusbp -b v1-latest
cd libusbp
mkdir build
cd build
cmake ..
make
sudo make install
cd ../..

read

git clone https://github.com/pololu/pololu-tic-software tic
cd tic
mkdir build
cd build
cmake ..
make
sudo make install
cd ../..

read

sudo cp tic/udev-rules/99-pololu.rules /etc/udev/rules.d/
sudo sh -c 'echo /usr/local/lib > /etc/ld.so.conf.d/local.conf'
sudo ldconfig

# pkg-config libusbp-1 --cflags