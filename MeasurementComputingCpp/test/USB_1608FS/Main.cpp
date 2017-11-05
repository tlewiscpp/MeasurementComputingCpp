#include <iostream>
#include <memory>
#include <string>
#include <algorithm>

#include "USB_1608FS.hpp"

using namespace MeasurementComputingCpp;

void doReadCounter();
void doAnalogIO(const std::string &str);
void doDigitalIO(const std::string &str);
void doDigitalRead(int pinNumber);
void doDigitalWrite(int pinNumber, const std::string &str);

static std::unique_ptr<USB_1608FS> mccBoard{nullptr};

int main()
{
    try {
        mccBoard.reset(new USB_1608FS{});
    } catch (std::exception &e) {
        std::cout << "USB_1608FS board not found. Please connect device and rerun program" << std::endl;
        return 1;
    }
    std::cout << "Enter pin type and number, and whether to read or write:" << std::endl;
    std::cout << "Ex: To read from digital pin 0, type Dr0" << std::endl;
    std::cout << "Ex: To write a 1 to digital pin 0, type Dw01" << std::endl;
    std::cout << "Ex: To read from the 32 bit counter, type C" << std::endl;
    std::cout << "Since only analog reads are available, only pin type and number are required for analog" << std::endl;
    std::cout << "Ex: To read from analog pin 0, type A0" << std::endl;
    std::cout << std::endl << std::endl;
    std::string rawString{""};
    while (true) {
        std::cout << "Enter command: ";
        std::getline(std::cin, rawString);
        std::transform(rawString.begin(), rawString.end(), rawString.begin(), ::toupper);
        if ((rawString.length() > 0) && (rawString[0] == 'C') ) {
            doReadCounter();
            continue;
        }
        if (rawString.length() < 2) {
            std::cout << "ERROR: Entry too short" << std::endl;
            continue;
        }

        if (rawString[0] == 'D') {
            doDigitalIO(rawString.substr(1));
        } else if (rawString[0] == 'A') {
            doAnalogIO(rawString.substr(1));
        } else {
            std::cout << "ERROR: Pin type \'" << rawString[0] << "\' is not a valid pin type (D for digital, A for analog)" << std::endl;
            continue;
        }
    }
}

void doReadCounter()
{
    std::cout << mccBoard->readCounter() << std::endl;
}

void doDigitalIO(const std::string &str)
{
    if (str.length() < 2) {
        std::cout << "ERROR: Digital command entry too short" << std::endl;
        return;
    }
    bool read{false};
    if (str[0] == 'R') {
        read = true;
    } else if (str[0] == 'W') {
        read = false;
    } else {
        std::cout << "ERROR: Pin command \'" << str[0] << "\' is not a valid pin command (\'r\' for read, \'w\' for write)" << std::endl;
        return;
    }
    int maybePinNumber{0};
    try {
        maybePinNumber = std::stoi(std::string(1, str[1]));
    } catch (std::exception &e) {
        std::cout << "ERROR: Pin number \'" << str[0] << "\' is not a valid pin number (between 0 and 7, inclusive)" << std::endl;
        return;
    }
    
    if (maybePinNumber < 0) {
        std::cout << "ERROR: Pin number \'" << maybePinNumber << "\' cannot be less than 0" << std::endl;
        return;
    } else if (maybePinNumber > 7) {
        std::cout << "ERROR: Pin number \'" << maybePinNumber << "\' cannot be greater than 7" << std::endl;
        return;
    }
    if (read) {
        doDigitalRead(maybePinNumber);
    } else {
        doDigitalWrite(maybePinNumber, str.substr(2));
    }
}

void doDigitalRead(int pinNumber)
{
    auto portNumber = static_cast<USB_1608FS::DigitalPinNumber>(pinNumber);
    mccBoard->setDigitalPortDirection(portNumber, USB_1608FS::PortDirection::DigitalInput);
    std::cout << mccBoard->digitalRead(portNumber) << std::endl;
}

void doDigitalWrite(int pinNumber, const std::string &str)
{
    if (str.length() < 1) {
        std::cout << "ERROR: Digital write command entry too short (expected 1 for HIGH, or 0 for LOW)" << std::endl;
        return;
    }
    auto portNumber = static_cast<USB_1608FS::DigitalPinNumber>(pinNumber);
    if (str[0] == '1') {
        mccBoard->setDigitalPortDirection(portNumber, USB_1608FS::PortDirection::DigitalOutput);
        mccBoard->digitalWrite(portNumber, true);
        std::cout << "Wrote pin " << pinNumber << " HIGH" << std::endl;
    } else if (str[0] == '0') {
        mccBoard->setDigitalPortDirection(portNumber, USB_1608FS::PortDirection::DigitalOutput);
        mccBoard->digitalWrite(portNumber, false);
        std::cout << "Wrote pin " << pinNumber << " LOW" << std::endl;
    } else {
        std::cout << "ERROR: Digital write command \'" << str[0] << "\' is not a valid command (1 for HIGH, 0 for LOW)" << std::endl;
    }
}

void doAnalogIO(const std::string &str)
{
    int maybePinNumber{0};
    try {
        maybePinNumber = std::stoi(std::string(1, str[0]));
    } catch (std::exception &e) {
        std::cout << "ERROR: Pin number \'" << str[0] << "\' is not a valid pin number (between 0 and 8, inclusive)" << std::endl;
        return;
    }
    if (maybePinNumber < 0) {
        std::cout << "ERROR: Pin number \'" << maybePinNumber << "\' cannot be less than 0" << std::endl;
        return;
    } else if (maybePinNumber > 7) {
        std::cout << "ERROR: Pin number \'" << maybePinNumber << "\' cannot be greater than 7" << std::endl;
        return;
    }
    std::cout << mccBoard->analogRead(static_cast<USB_1608FS::AnalogPinNumber>(maybePinNumber)) << std::endl;
}
