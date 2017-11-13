#include <mcc-libusb/pmd.h>
#include <mcc-libusb/usb-1208LS.h>

#include "USB_1208LS.hpp"

#include <cstring>

#define CHECK_BIT(value,bit) ((value) & (1 << (bit)))

namespace MeasurementComputingCpp {


#define BITS_PER_PORT_1208LS 8
#define MAXIMUM_ANALOG_OUTPUT_VALUE_1208LS 65535
#define NUMBER_OF_ANALOG_OUTPUT_PINS 2
#define SERIAL_NUMBER_BUFFER_1208LS 255

USB_1208LS::USB_1208LS() :
    USB_IO_Base{"USB_1208LS"},
        m_hidDevice{hid_open(MCC_VID, USB1208LS_PID, nullptr)},
    m_digitalPortMap{},
    m_analogInputMode{AnalogInputMode::SingleEnded},
    m_serialNumber{""}
{

    int initResult{hid_init()};
    if (initResult != 0) {
        throw std::runtime_error("USB_1024LS::USB_1024LS(): hid_init failed with return code " + toStdString(initResult));
    }

    if (!this->m_hidDevice) {
        throw std::runtime_error("USB_1024LS::USB_1024LS(): USB1024LS device NOT found");
    }

    usbDConfigPort_USB1208LS(this->m_hidDevice, DIO_PORTA, DIO_DIR_IN);
    usbDConfigPort_USB1208LS(this->m_hidDevice, DIO_PORTB, DIO_DIR_IN);

    this->m_digitalPortMap.emplace(USB_1208LS::DigitalPortID::PortA, USB_1208LS::PortDirection::DigitalInput);
    this->m_digitalPortMap.emplace(USB_1208LS::DigitalPortID::PortB, USB_1208LS::PortDirection::DigitalInput);

    this->resetCounter();
}


USB_1208LS::USB_1208LS(USB_1208LS &&rhs) noexcept :
    USB_IO_Base{"USB_1208LS"},
    m_hidDevice{rhs.m_hidDevice},
    m_digitalPortMap{std::move(rhs.m_digitalPortMap)},
    m_analogInputMode{rhs.m_analogInputMode},
    m_serialNumber{std::move(rhs.m_serialNumber)}
{

}

USB_1208LS& USB_1208LS::operator=(USB_1208LS &&rhs) noexcept
{
    this->m_hidDevice = rhs.m_hidDevice;
    this->m_digitalPortMap = std::move(rhs.m_digitalPortMap);
    this->m_analogInputMode = rhs.m_analogInputMode;
    this->m_serialNumber = std::move(rhs.m_serialNumber);
    return *this;
}

void USB_1208LS::setDigitalPortDirection(DigitalPortID portID, PortDirection direction)
{
    auto currentPortDirection = this->m_digitalPortMap.find(portID)->second;
    if (currentPortDirection == direction) {
        return;
    }
    usbDConfigPort_USB1208LS(this->m_hidDevice, digitalPortIDToUInt8(portID), digitalPortDirectionToUInt8(direction));
    if (direction == PortDirection::DigitalOutput) {
        usbDOut_USB1208LS(this->m_hidDevice, digitalPortIDToUInt8(portID), 0x00); //0b00000000
    }
    this->m_digitalPortMap.find(portID)->second = direction;
}

uint8_t USB_1208LS::digitalPortIDToUInt8(DigitalPortID portID)
{
    if (portID == USB_1208LS::DigitalPortID::PortA) {
        return DIO_PORTA;
    } else if (portID == USB_1208LS::DigitalPortID::PortB) {
        return DIO_PORTB;
    } else {
        throw std::runtime_error("USB_1208LS::digitalPortIDToUInt8(DigitalPortID): Invalid DigitalPortID");
    }
}

uint8_t USB_1208LS::digitalPortDirectionToUInt8(PortDirection direction)
{
    if (direction == USB_1208LS::PortDirection::DigitalInput) {
        return DIO_DIR_IN;
    } else if (direction == USB_1208LS::PortDirection::DigitalOutput) {
        return DIO_DIR_OUT;
    } else {
        throw std::runtime_error("USB_1208LS::digitalPortDirectionToUInt8(PortDirection): Invalid PortDirection");
    }
}


USB_1208LS::PortDirection USB_1208LS::digitalPortDirection(USB_1208LS::DigitalPortID portID) const
{
    return this->m_digitalPortMap.find(portID)->second;
}

bool USB_1208LS::digitalWrite(DigitalPortID portID, uint8_t pinNumber, bool state)
{
    int upperPinNumber{BITS_PER_PORT_1208LS};
    if (pinNumber >= upperPinNumber) {
        throw std::runtime_error("ERROR: USB_1208LS::digitalWrite(DigitalPortID, uint8_t, bool): pinNumber for ports A and B must be between 0 and " + toStdString(upperPinNumber) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(upperPinNumber));
    }
    if (this->m_digitalPortMap.find(portID)->second != USB_1208LS::PortDirection::DigitalOutput) {
        return false;
    }
    usbDBitOut_USB1208LS(this->m_hidDevice, digitalPortIDToUInt8(portID), pinNumber, static_cast<uint8_t>(state));
    return true;
}

bool USB_1208LS::digitalRead(DigitalPortID portID, uint8_t pinNumber)
{
    int upperPinNumber{BITS_PER_PORT_1208LS};
    if (pinNumber >= upperPinNumber) {
        throw std::runtime_error("ERROR: USB_1208LS::digitalWrite(DigitalPortID, uint8_t, bool): pinNumber for ports A and B must be between 0 and " + toStdString(upperPinNumber) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(upperPinNumber));
    }
    if (this->m_digitalPortMap.find(portID)->second != USB_1208LS::PortDirection::DigitalInput) {
        return false;
    }
    uint8_t allValues{0};
    usbDIn_USB1208LS(this->m_hidDevice, this->digitalPortIDToUInt8(portID), &allValues);
    return static_cast<bool>(CHECK_BIT(allValues, pinNumber));
}



uint8_t USB_1208LS::voltageRangeToDifferentialGain(USB_1208LS::VoltageRange voltageRange)
{
    if (voltageRange == USB_1208LS::VoltageRange::V_20) {
        return BP_20_00V;
    } else if (voltageRange == USB_1208LS::VoltageRange::V_10) {
        return BP_10_00V;
    } else if (voltageRange == USB_1208LS::VoltageRange::V_5) {
        return BP_5_00V;
    } else if (voltageRange == USB_1208LS::VoltageRange::V_4) {
        return BP_4_00V;
    } else if (voltageRange == USB_1208LS::VoltageRange::V_2_5) {
        return BP_2_50V;
    } else if (voltageRange == USB_1208LS::VoltageRange::V_2) {
        return BP_2_00V;
    } else if (voltageRange == USB_1208LS::VoltageRange::V_1_2_5) {
        return BP_1_25V;
    } else if (voltageRange == USB_1208LS::VoltageRange::V_1) {
        return BP_1_00V;
    }
    return 0;
}


short USB_1208LS::analogRead(uint8_t pinNumber, USB_1208LS::VoltageRange voltageRange)
{
    /*
    if ((this->m_analogInputMode == USB_1208LS::AnalogInputMode::SingleEnded) && (voltageRange != USB_1208LS::VoltageRange::V_10)) {
        throw std::runtime_error("analogRead voltage range can only be V_10 in SingleEnded AnalogInputMode");
    }
    */
    if ( (this->m_analogInputMode == USB_1208LS::AnalogInputMode::Differential) && (pinNumber >= (BITS_PER_PORT_1208LS/2) ) ) {
        throw std::runtime_error("USB_1208LS::analogRead(uint8_t, USB_1208LS::VoltageRange): analogRead pin number cannot be greater than \""
                                 + toStdString( (BITS_PER_PORT_1208LS/2) - 1)
                                 + "\" in Differential AnalogInputMode ("
                                 + toStdString(static_cast<int>(pinNumber))
                                 + " > "
                                 + toStdString( (BITS_PER_PORT_1208LS/2) - 1)
                                 + ")" );
    } else if ( (this->m_analogInputMode == USB_1208LS::AnalogInputMode::SingleEnded) && (pinNumber >= BITS_PER_PORT_1208LS) ) {
        throw std::runtime_error("USB_1208LS::analogRead(uint8_t, USB_1208LS::VoltageRange): analogRead pin number cannot be greater than \""
                                 + toStdString(BITS_PER_PORT_1208LS)
                                 + "\" in SingleEnded AnalogInputMode ("
                                 + toStdString(static_cast<int>(pinNumber))
                                 + " > "
                                 + toStdString(BITS_PER_PORT_1208LS)
                                 + ")" );
    }
    if (this->m_analogInputMode == USB_1208LS::AnalogInputMode::SingleEnded) {
        return usbAIn_USB1208LS(this->m_hidDevice, pinNumber, SE_10_00V);
    } else {
        return usbAIn_USB1208LS(this->m_hidDevice, pinNumber, this->voltageRangeToDifferentialGain(voltageRange));
    }

}

float USB_1208LS::voltageRead(uint8_t pinNumber, USB_1208LS::VoltageRange voltageRange)
{
    short analogReading{this->analogRead(pinNumber, voltageRange)};
    if (this->m_analogInputMode == USB_1208LS::AnalogInputMode::SingleEnded) {
        return volts_LS(analogReading, SE_10_00V);
    } else {
        return volts_LS(this->voltageRangeToDifferentialGain(voltageRange), analogReading);
    }

}

void USB_1208LS::setAnalogInputMode(USB_1208LS::AnalogInputMode analogInputMode)
{
    this->m_analogInputMode = analogInputMode;
}

USB_1208LS::AnalogInputMode USB_1208LS::analogInputMode() const
{
    return this->m_analogInputMode;
}

void USB_1208LS::analogWrite(uint8_t pinNumber, uint16_t state)
{
    if (pinNumber > (NUMBER_OF_ANALOG_OUTPUT_PINS - 1)) {
        throw std::runtime_error("USB_1208LS::analogWrite(uint8_t, uint16_t): analogWrite pin number exceeds maximum pin number (" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(NUMBER_OF_ANALOG_OUTPUT_PINS - 1));
    }
    usbAOut_USB1208LS(this->m_hidDevice, pinNumber, state);
}

std::string USB_1208LS::serialNumber() const
{
    if (!this->m_serialNumber.empty()) {
        return this->m_serialNumber;
    }
    wchar_t tempWideSerialNumber[SERIAL_NUMBER_BUFFER_1208LS];
    wmemset(tempWideSerialNumber, '\0', SERIAL_NUMBER_BUFFER_1208LS);
    hid_get_serial_number_string(this->m_hidDevice, tempWideSerialNumber, SERIAL_NUMBER_BUFFER_1208LS);

    char tempSerialNumber[SERIAL_NUMBER_BUFFER_1208LS];
    memset(tempSerialNumber, '\0', SERIAL_NUMBER_BUFFER_1208LS);
    wcstombs(tempSerialNumber, tempWideSerialNumber, SERIAL_NUMBER_BUFFER_1208LS);
    this->m_serialNumber = std::string{tempSerialNumber};
    return this->m_serialNumber;
}

float USB_1208LS::analogToVoltage(short analogReading, USB_1208LS::AnalogInputMode inputMode, USB_1208LS::VoltageRange voltageRange)
{
    if (inputMode == AnalogInputMode::SingleEnded) {
        return volts_LS(analogReading, SE_10_00V);
    } else {
        return volts_LS(analogReading, voltageRangeToDifferentialGain(voltageRange));
    }
}

void USB_1208LS::resetDevice()
{
    usbReset_USB1208LS(this->m_hidDevice);
}

void USB_1208LS::resetCounter() {
    usbInitCounter_USB1208LS(this->m_hidDevice);
}

uint32_t USB_1208LS::readCounter() {
    return usbReadCounter_USB1208LS(this->m_hidDevice);
}


USB_1208LS::~USB_1208LS()
{
    hid_close(this->m_hidDevice);
}


} //namespace MeasurementComputingCpp
