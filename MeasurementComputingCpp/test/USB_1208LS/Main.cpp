#include <iostream>
#include <memory>
#include <string>
#include <algorithm>

#include "USB_1208LS.hpp"

using namespace MeasurementComputingCpp;

void doReadCounter();
void doAnalogIO(const std::string &str);
void doDigitalIO(const std::string &str);
void doDigitalRead(USB_1208LS::DigitalPortID portNumber, int pinNumber);
void doDigitalWrite(USB_1208LS::DigitalPortID portNumber, int pinNumber, const std::string &str);

static std::unique_ptr<USB_1208LS> mccBoard{nullptr};

int main()
{
    try {
        mccBoard.reset(new USB_1208LS{});
    } catch (std::exception &e) {
        std::cout << "USB_1208LS board not found. Please connect device and rerun program" << std::endl;
        return 1;
    }
    std::cout << "Enter pin type and number, and whether to read or write:" << std::endl;
    std::cout << "Ex: To read 32 bit ounter, type C" << std::endl;
    std::cout << "Ex: To read from digital pin 0 (port A), type DrA0" << std::endl;
    std::cout << "Ex: To write a 1 to digital pin 0 (port A), type DwA01" << std::endl;
    std::cout << "Ex: To read from digital pin 0 (port B), you'd request DB0r" << std::endl;
    std::cout << "Since the USB1208LS can only set entire ports to Input or Output, a previously set digital output can be implicitly reset by reading a pin from the same port" << std::endl;
    std::cout << "Ex: If digital pin 0 is written high, then a read is requested from digital pin 2 (both from port A), digital pin 0 will not long be set high" << std::endl;
    std::cout << "Since only analog reads are available, only pin type and number are required for analog" << std::endl;
    std::cout << "Ex: To read from analog pin 0, type A0" << std::endl;
    std::cout << std::endl << std::endl;
    std::string rawString{""};
    
    mccBoard->setDigitalPortDirection(USB_1208LS::DigitalPortID::PortA, USB_1208LS::PortDirection::DigitalInput);
    mccBoard->setDigitalPortDirection(USB_1208LS::DigitalPortID::PortB, USB_1208LS::PortDirection::DigitalInput);
    mccBoard->setAnalogInputMode(USB_1208LS::AnalogInputMode::SingleEnded);
    while (true) {
        std::cout << "Enter command: ";
        std::getline(std::cin, rawString);
        std::transform(rawString.begin(), rawString.end(), rawString.begin(), ::toupper);
        if ( (rawString.length() > 0) && (rawString[0] == 'C') ) {
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
    if (str.length() < 3) {
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
    auto portNumber = USB_1208LS::DigitalPortID::PortA;
    if (str[1] == 'A') {
        portNumber = USB_1208LS::DigitalPortID::PortA;
    } else if (str[1] == 'B') {
        portNumber = USB_1208LS::DigitalPortID::PortB;
    } else {
        std::cout << "ERROR: Digital port ID \"" << str[1] << "\" is not a valid port ID (A for port A, B for port B)" << std::endl;
        return;
    }
    int maybePinNumber{0};
    try {
        maybePinNumber = std::stoi(std::string(1, str[2]));
    } catch (std::exception &e) {
        std::cout << "ERROR: Pin number \'" << str[2] << "\' is not a valid pin number (between 0 and 8, inclusive)" << std::endl;
        return;
    }
    if (maybePinNumber < 0) {
        std::cout << "ERROR: Pin number \'" << maybePinNumber << "\' must be greater than 0" << std::endl;
        return;
    } else if (maybePinNumber >= 8) {
        std::cout << "ERROR: Pin number \'" << maybePinNumber << "\' must be less than " << 8 << std::endl;
        return;
    }
    if (read) {
        doDigitalRead(portNumber, maybePinNumber);
    } else {
        doDigitalWrite(portNumber, maybePinNumber, str.substr(3));
    }
}

void doDigitalRead(USB_1208LS::DigitalPortID portNumber, int pinNumber)
{
    mccBoard->setDigitalPortDirection(portNumber, USB_1208LS::PortDirection::DigitalInput);
    std::cout << mccBoard->digitalRead(portNumber, pinNumber) << std::endl;
}

void doDigitalWrite(USB_1208LS::DigitalPortID portNumber, int pinNumber, const std::string &str)
{
    if (str.length() < 1) {
        std::cout << "ERROR: Digital write command entry too short (expected 1 for HIGH, or 0 for LOW)" << std::endl;
        return;
    }
    if (str[0] == '1') {
        mccBoard->setDigitalPortDirection(portNumber, USB_1208LS::PortDirection::DigitalOutput);
        mccBoard->digitalWrite(portNumber, pinNumber, true);
        std::cout << "Wrote pin " << pinNumber << " HIGH" << std::endl;
    } else if (str[0] == '0') {
        mccBoard->setDigitalPortDirection(portNumber, USB_1208LS::PortDirection::DigitalOutput);
        mccBoard->digitalWrite(portNumber, pinNumber, false);
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
        std::cout << "ERROR: Pin number \'" << str[0] << "\' is not a valid pin number (between 0 and 7, inclusive)" << std::endl;
        return;
    }
    if (maybePinNumber < 0) {
        std::cout << "ERROR: Pin number \'" << maybePinNumber << "\' must be greater than 0" << std::endl;
    } else if (maybePinNumber >= 8) {
        std::cout << "ERROR: Pin number \'" << maybePinNumber << "\' must be less than " << 8 << std::endl;
    } else {
        std::cout << mccBoard->analogRead(maybePinNumber) << std::endl;
    }
}
