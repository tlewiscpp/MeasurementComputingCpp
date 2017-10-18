# MeasurementComputingCpp
A simple c++ wrapper for the open source Linux MeasurementComputing drivers (https://github.com/wjasper/Linux_Drivers) 

Clone the source:
    git clone https://github.com/tlewiscpp/MeasurementComputingCpp

Initialize the repository:
    cd MeasurementComputingCpp/
    ./init-repository.sh

Build and install the source:
    ./install-measurementcomputingcpp.sh [YourBuildDirHere]

All header includes will be placed in /usr/include/MeasurementComputingCpp/

To include a MeasurementComputingCpp file in your source (for example a USB 1608FS board):
    #include <MeasurementComputingCpp/USB_1608FS.hpp>

To compile a program using MeasurementComputingCpp:
    g++ -Wall -std=c++11 YourSourceFile.cpp -lMeasurementComputingCpp

