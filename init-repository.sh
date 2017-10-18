#!/bin/bash -e

cd hidapi-root || { echo "Could not enter hidapi-root, exiting"; exit 1; }
./clone-source.sh
cd ../

cd libusb-root || { echo "Could not enter libusb-root, exiting"; exit 1; }
./clone-source.sh
cd ../

cd MeasurementComputingCore/mccusb-root || { echo "Could not enter mccusb-root, exiting"; exit 1; }
./clone-source.sh
cd ../
