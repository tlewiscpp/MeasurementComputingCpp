#!/bin/bash -e

#cd hidapi-root || { echo "Could not enter hidapi-root, exiting"; exit 1; }
#./clone-source.sh
#cd ../

#cd libusb-root || { echo "Could not enter libusb-root, exiting"; exit 1; }
#./clone-source.sh
#cd ../

#cd MeasurementComputingCore/mccusb-root || { echo "Could not enter mccusb-root, exiting"; exit 1; }
#./clone-source.sh
#cd ../

if [[ ! -f ".repo-init" ]]; then
    git submodule update --init --recursive
    touch .repo-init
else
    git submodule update --recursive --remote
fi

cp -R libusb-root/libusb/libusb/libusb.h mccusb-root/libusb-1.0/
