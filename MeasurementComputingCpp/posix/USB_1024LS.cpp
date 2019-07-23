#include <mutex>
#include <cstring>
#include <cstdlib>

#include <mcc-libusb/pmd.h>
#include <mcc-libusb/usb-1024LS.h>

#include "USB_1024LS.hpp"

#define CHECK_BIT(value,bit) ((value) & (1 << (bit)))

namespace MeasurementComputingCpp {

constexpr auto PORT_A_MAX_PIN_NUMBER = BITS_PER_PORT_1024LS;
constexpr auto PORT_B_MAX_PIN_NUMBER = (BITS_PER_PORT_1024LS*2);
constexpr auto PORT_C_LOW_MAX_PIN_NUMBER = (BITS_PER_PORT_1024LS*3) - (BITS_PER_PORT_1024LS/2);
constexpr auto PORT_C_HIGH_MAX_PIN_NUMBER = (BITS_PER_PORT_1024LS*3);

USB_1024LS::USB_1024LS() :
    USB_IO_Base{"USB_1024LS"},
    m_hidDevice{hid_open(MCC_VID, USB1024LS_PID, nullptr)},
    m_serialNumber{""},
    m_digitalPortMap{}
{
    int initResult{hid_init()};
    if (initResult != 0) {
        throw std::runtime_error("USB_1024LS::USB_1024LS(): hid_init failed with return code " + toStdString(initResult));
    }

    if (!this->m_hidDevice) {
        throw std::runtime_error("USB_1024LS::USB_1024LS(): USB1024LS device NOT found");
    }

    usbDConfigPort_USB1024LS(this->m_hidDevice, DIO_PORTA, DIO_DIR_IN);
    usbDConfigPort_USB1024LS(this->m_hidDevice, DIO_PORTB, DIO_DIR_IN);
    usbDConfigPort_USB1024LS(this->m_hidDevice, DIO_PORTC_LOW, DIO_DIR_IN);
    usbDConfigPort_USB1024LS(this->m_hidDevice, DIO_PORTC_HI, DIO_DIR_IN);

    this->m_digitalPortMap.emplace(USB_1024LS::DigitalPortID::PortA, USB_1024LS::PortDirection::DigitalInput);
    this->m_digitalPortMap.emplace(USB_1024LS::DigitalPortID::PortB, USB_1024LS::PortDirection::DigitalInput);
    this->m_digitalPortMap.emplace(USB_1024LS::DigitalPortID::PortCLow, USB_1024LS::PortDirection::DigitalInput);
    this->m_digitalPortMap.emplace(USB_1024LS::DigitalPortID::PortCHigh, USB_1024LS::PortDirection::DigitalInput);

    this->resetCounter();

}

USB_1024LS::USB_1024LS(USB_1024LS &&rhs) noexcept :
    USB_IO_Base{"USB_1024LS"},
    m_hidDevice{rhs.m_hidDevice},
    m_serialNumber{std::move(rhs.m_serialNumber)},
    m_digitalPortMap{std::move(rhs.m_digitalPortMap)}
{

}

USB_1024LS& USB_1024LS::operator=(USB_1024LS &&rhs) noexcept {
    this->m_hidDevice = rhs.m_hidDevice;
    this->m_serialNumber = std::move(rhs.m_serialNumber);
    this->m_digitalPortMap = std::move(rhs.m_digitalPortMap);
    return *this;
}

uint8_t USB_1024LS::digitalPortIDToUInt8(DigitalPortID portID) {
    if (portID == USB_1024LS::DigitalPortID::PortA) {
        return DIO_PORTA;
    } else if (portID == USB_1024LS::DigitalPortID::PortB) {
        return DIO_PORTB;
    } else if (portID == USB_1024LS::DigitalPortID::PortCLow) {
        return DIO_PORTC_LOW;
    } else if (portID == USB_1024LS::DigitalPortID::PortCHigh) {
        return DIO_PORTC_HI;
    } else {
        throw std::runtime_error("USB_1024LS::digitalPortIDToUInt8(DigitalPortID): Invalid DigitalPortID");
    }
}

uint8_t USB_1024LS::digitalPortDirectionToUInt8(PortDirection direction) {
    if (direction == USB_1024LS::PortDirection::DigitalInput) {
        return DIO_DIR_IN;
    } else if (direction == USB_1024LS::PortDirection::DigitalOutput) {
        return DIO_DIR_OUT;
    } else {
        throw std::runtime_error("USB_1024LS::digitalPortDirectionToUInt8(PortDirection): Invalid PortDirection");
    }
}

void USB_1024LS::setDigitalPortDirection(DigitalPortID portID, PortDirection direction) {
    auto currentPortDirection = this->m_digitalPortMap.find(portID)->second;
    if (currentPortDirection == direction) {
        return;
    }
    usbDConfigPort_USB1024LS(this->m_hidDevice, digitalPortIDToUInt8(portID), digitalPortDirectionToUInt8(direction));
    if (direction == PortDirection::DigitalOutput) {
        usbDOut_USB1024LS(this->m_hidDevice, digitalPortIDToUInt8(portID), 0x00); //0b00000000
    }
    this->m_digitalPortMap.find(portID)->second = direction;
}

USB_1024LS::PortDirection USB_1024LS::digitalPortDirection(USB_1024LS::DigitalPortID portID) const {
    return this->m_digitalPortMap.find(portID)->second;
}

bool USB_1024LS::digitalWrite(DigitalPortID portID, uint8_t pinNumber, bool state) {
    int upperPinNumber{0};
    if ( (portID == DigitalPortID::PortA) || (portID == DigitalPortID::PortB) ) {
        upperPinNumber = BITS_PER_PORT_1024LS;
        if (pinNumber >= upperPinNumber) {
            throw std::runtime_error("USB_1024LS::digitalWrite(DigitalPortID, uint8_t, bool): pinNumber for ports A and B must be between 0 and " + toStdString(upperPinNumber) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(upperPinNumber));
        }
    } else {
        upperPinNumber = BITS_PER_PORT_1024LS/2;
        if (pinNumber >= upperPinNumber) {
            throw std::runtime_error("USB_1024LS::digitalWrite(DigitalPortID, uint8_t, bool): pinNumber for ports A and B must be between 0 and " + toStdString(upperPinNumber) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(upperPinNumber));
        }
    }
    if (this->m_digitalPortMap.find(portID)->second != USB_1024LS::PortDirection::DigitalOutput) {
        return false;
    }
    usbDBitOut_USB1024LS(this->m_hidDevice, digitalPortIDToUInt8(portID), pinNumber, static_cast<uint8_t>(state));
    return true;
}

bool USB_1024LS::digitalRead(uint8_t pinNumber) {
    DigitalPortID portID{};
    uint8_t adjustedPinNumber{};
    auto result = getDigitalPortIDAndPinNumber(pinNumber, &portID, &adjustedPinNumber);
    if (!result) {
        throw std::runtime_error("USB_1024LS::digitalRead(bool): pinNumber must be between 0 and " + toStdString(PORT_C_HIGH_MAX_PIN_NUMBER) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(PORT_C_HIGH_MAX_PIN_NUMBER));
    }
    return this->digitalRead(portID, adjustedPinNumber);
}

bool USB_1024LS::digitalWrite(uint8_t pinNumber, bool state) {
    DigitalPortID portID{};
    uint8_t adjustedPinNumber{};
    auto result = getDigitalPortIDAndPinNumber(pinNumber, &portID, &adjustedPinNumber);
    if (!result) {
        throw std::runtime_error("USB_1024LS::digitalWrite(uint8_t, bool): pinNumber must be between 0 and " + toStdString(PORT_C_HIGH_MAX_PIN_NUMBER) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(PORT_C_HIGH_MAX_PIN_NUMBER));
    }
    return this->digitalWrite(portID, adjustedPinNumber, state);
}

bool USB_1024LS::getDigitalPortIDAndPinNumber(uint8_t pinNumber, DigitalPortID *outPortID, uint8_t *outAdjustedPinNumber) {
    if (pinNumber < PORT_A_MAX_PIN_NUMBER) {
        *outPortID = DigitalPortID::PortA;
        *outAdjustedPinNumber = pinNumber;
    } else if ( (pinNumber >= PORT_A_MAX_PIN_NUMBER) && (pinNumber < PORT_B_MAX_PIN_NUMBER) ) {
        *outPortID = DigitalPortID::PortB;
        *outAdjustedPinNumber = pinNumber - BITS_PER_PORT_1024LS;
    } else if ( (pinNumber >= PORT_B_MAX_PIN_NUMBER) && (pinNumber < PORT_C_LOW_MAX_PIN_NUMBER) ) {
        *outPortID = DigitalPortID::PortCLow;
        *outAdjustedPinNumber = pinNumber - (BITS_PER_PORT_1024LS*2);
    } else if ( (pinNumber >= PORT_C_LOW_MAX_PIN_NUMBER) && (pinNumber < PORT_C_HIGH_MAX_PIN_NUMBER) ) {
        *outPortID = DigitalPortID::PortCHigh;
        *outAdjustedPinNumber = pinNumber - (BITS_PER_PORT_1024LS*2) - (BITS_PER_PORT_1024LS/2);
    } else {
        return false;
    }
    return true;
}

bool USB_1024LS::digitalRead(DigitalPortID portID, uint8_t pinNumber)
{
    int upperPinNumber{0};
    if ( (portID == DigitalPortID::PortA) || (portID == DigitalPortID::PortB) ) {
        upperPinNumber = BITS_PER_PORT_1024LS;
        if (pinNumber >= upperPinNumber) {
            throw std::runtime_error("USB_1024LS::digitalWrite(DigitalPortID, uint8_t, bool): pinNumber for ports A and B must be between 0 and " + toStdString(upperPinNumber) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(upperPinNumber));
        }
    } else {
        upperPinNumber = BITS_PER_PORT_1024LS/2;
        if (pinNumber >= upperPinNumber) {
            throw std::runtime_error("USB_1024LS::digitalWrite(DigitalPortID, uint8_t, bool): pinNumber for ports A and B must be between 0 and " + toStdString(upperPinNumber) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(upperPinNumber));
        }
    }
    if (this->m_digitalPortMap.find(portID)->second != USB_1024LS::PortDirection::DigitalInput) {
        return false;
    }
    uint8_t allValues{0};
    usbDIn_USB1024LS(this->m_hidDevice, this->digitalPortIDToUInt8(portID), &allValues);
    return static_cast<bool>(CHECK_BIT(allValues, pinNumber));
}

std::string USB_1024LS::serialNumber() const {
    if (!this->m_serialNumber.empty()) {
        return this->m_serialNumber;
    }
    wchar_t tempWideSerialNumber[SERIAL_NUMBER_BUFFER_1024LS];
    wmemset(tempWideSerialNumber, '\0', SERIAL_NUMBER_BUFFER_1024LS);
    hid_get_serial_number_string(this->m_hidDevice, tempWideSerialNumber, SERIAL_NUMBER_BUFFER_1024LS);

    char tempSerialNumber[SERIAL_NUMBER_BUFFER_1024LS];
    memset(tempSerialNumber, '\0', SERIAL_NUMBER_BUFFER_1024LS);
    wcstombs(tempSerialNumber, tempWideSerialNumber, SERIAL_NUMBER_BUFFER_1024LS);
    this->m_serialNumber = std::string{tempSerialNumber};
    return this->m_serialNumber;
}

void USB_1024LS::resetDevice() {
    usbReset_USB1024LS(this->m_hidDevice);
}

void USB_1024LS::resetCounter() {
    usbInitCounter_USB1024LS(this->m_hidDevice);
}

uint32_t USB_1024LS::readCounter() {
    return usbReadCounter_USB1024LS(this->m_hidDevice);
}

USB_1024LS::~USB_1024LS() {
    hid_close(this->m_hidDevice);
}


} //namespace MeasurementComputingCpp
