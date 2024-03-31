#!/bin/bash

echo "Installing pololu tic software"
tic_software=https://www.pololu.com/file/0J1348/pololu-tic-1.8.1-linux-x86.tar.xz
wget $tic_software
tar -xvf pololu-tic-*.tar.xz
sudo pololu-tic-*/install.sh

echo "Installing build essentials"
sudo apt-get install build-essential git cmake libudev-dev qtbase5-dev
echo "Updating submodules"
git submodule foreach git pull

cd modules

echo "Building and installing libusbp"
cd libusbp
mkdir build
cd build
cmake ..
make
sudo make install
cd ../..

read

echo "Building and installing pololu-tic-software"
cd tic
mkdir build
cd build
cmake ..
make
sudo make install
cd ../..

echo "Building and installing matplotlib-cpp"
cd matplotlib-cpp
mkdir build
cd build
cmake ..
make
sudo make install
cd ../..

sudo cp tic/udev-rules/99-pololu.rules /etc/udev/rules.d/
sudo sh -c 'echo /usr/local/lib > /etc/ld.so.conf.d/local.conf'
sudo ldconfig

echo "Installing sfml for visualization"
sudo apt-get install libsfml-dev

# pkg-config libusbp-1 --cflags