#include <mutex>
#include <cstring>

#include <mcc-libusb/pmd.h>
#include <mcc-libusb/usb-1024LS.h>

#include "USB_1024LS.hpp"

#define CHECK_BIT(value,bit) ((value) & (1 << (bit)))

namespace MeasurementComputingCpp {

USB_1024LS::USB_1024LS() :
    m_hidDevice{hid_open(MCC_VID, USB1024LS_PID, nullptr)},
    m_digitalPortMap{},
    m_serialNumber{""}
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
    m_hidDevice{rhs.m_hidDevice},
    m_digitalPortMap{std::move(rhs.m_digitalPortMap)},
    m_serialNumber{std::move(rhs.m_serialNumber)}
{

}

USB_1024LS& USB_1024LS::operator=(USB_1024LS &&rhs) noexcept
{
    this->m_hidDevice = rhs.m_hidDevice;
    this->m_digitalPortMap = std::move(rhs.m_digitalPortMap);
    this->m_serialNumber = std::move(rhs.m_serialNumber);
    return *this;
}

uint8_t USB_1024LS::digitalPortIDToUInt8(DigitalPortID portID)
{
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

uint8_t USB_1024LS::digitalPortDirectionToUInt8(PortDirection direction)
{
    if (direction == USB_1024LS::PortDirection::DigitalInput) {
        return DIO_DIR_IN;
    } else if (direction == USB_1024LS::PortDirection::DigitalOutput) {
        return DIO_DIR_OUT;
    } else {
        throw std::runtime_error("USB_1024LS::digitalPortDirectionToUInt8(PortDirection): Invalid PortDirection");
    }
}

void USB_1024LS::setDigitalPortDirection(DigitalPortID portID, PortDirection direction)
{
    usbDConfigPort_USB1024LS(this->m_hidDevice, digitalPortIDToUInt8(portID), digitalPortDirectionToUInt8(direction));
    if (direction == PortDirection::DigitalOutput) {
        usbDOut_USB1024LS(this->m_hidDevice, digitalPortIDToUInt8(portID), 0x00); //0b00000000
    }
    this->m_digitalPortMap.find(portID)->second = direction;
}

USB_1024LS::PortDirection USB_1024LS::digitalPortDirection(USB_1024LS::DigitalPortID portID) const
{
    return this->m_digitalPortMap.find(portID)->second;
}

bool USB_1024LS::digitalWrite(DigitalPortID portID, uint8_t pinNumber, bool state)
{
    int upperPinNumber{0};
    if ( (portID == DigitalPortID::PortA) || (portID == DigitalPortID::PortB) ) {
        upperPinNumber = BITS_PER_PORT_1024LS;
        if (pinNumber >= upperPinNumber) {
            throw std::runtime_error("ERROR: USB_1024LS::digitalWrite(DigitalPortID, uint8_t, bool): pinNumber for ports A and B must be between 0 and " + toStdString(upperPinNumber) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(upperPinNumber));
        }
    } else {
        upperPinNumber = BITS_PER_PORT_1024LS/2;
        if (pinNumber >= upperPinNumber) {
            throw std::runtime_error("ERROR: USB_1024LS::digitalWrite(DigitalPortID, uint8_t, bool): pinNumber for ports A and B must be between 0 and " + toStdString(upperPinNumber) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(upperPinNumber));
        }
    }
    if (this->m_digitalPortMap.find(portID)->second != USB_1024LS::PortDirection::DigitalOutput) {
        return false;
    }
    usbDBitOut_USB1024LS(this->m_hidDevice, digitalPortIDToUInt8(portID), pinNumber, static_cast<uint8_t>(state));
    return true;
}

bool USB_1024LS::digitalRead(DigitalPortID portID, uint8_t pinNumber)
{
    int upperPinNumber{0};
    if ( (portID == DigitalPortID::PortA) || (portID == DigitalPortID::PortB) ) {
        upperPinNumber = BITS_PER_PORT_1024LS;
        if (pinNumber >= upperPinNumber) {
            throw std::runtime_error("ERROR: USB_1024LS::digitalWrite(DigitalPortID, uint8_t, bool): pinNumber for ports A and B must be between 0 and " + toStdString(upperPinNumber) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(upperPinNumber));
        }
    } else {
        upperPinNumber = BITS_PER_PORT_1024LS/2;
        if (pinNumber >= upperPinNumber) {
            throw std::runtime_error("ERROR: USB_1024LS::digitalWrite(DigitalPortID, uint8_t, bool): pinNumber for ports A and B must be between 0 and " + toStdString(upperPinNumber) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(upperPinNumber));
        }
    }
    if (this->m_digitalPortMap.find(portID)->second != USB_1024LS::PortDirection::DigitalInput) {
        return false;
    }
    uint8_t allValues{0};
    usbDIn_USB1024LS(this->m_hidDevice, this->digitalPortIDToUInt8(portID), &allValues);
    return static_cast<bool>(CHECK_BIT(allValues, pinNumber));
}

std::string USB_1024LS::serialNumber() const
{
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

USB_1024LS::~USB_1024LS()
{
    hid_close(this->m_hidDevice);
}


} //namespace MeasurementComputingCpp
