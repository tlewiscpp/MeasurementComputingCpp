#include "USB_1208LS.hpp"

#include "common/cbw.h"

#include <cstring>
#include <cstdlib>

#define CHECK_BIT(value,bit) ((value) & (1 << (bit)))

namespace MeasurementComputingCpp {


#define BITS_PER_PORT_1208LS 8
#define MAXIMUM_ANALOG_OUTPUT_VALUE_1208LS 65535
#define NUMBER_OF_ANALOG_OUTPUT_PINS 2
#define SERIAL_NUMBER_BUFFER_1208LS 255

USB_1208LS::USB_1208LS(unsigned int boardNumber) :
    USB_IO_Base{"USB-1208LS", boardNumber},
    m_boardNumber{boardNumber},
    m_digitalPortMap{},
    m_analogInputMode{AnalogInputMode::SingleEnded}
{

    auto boardName = this->getBoardName(boardNumber);
    if (boardName != this->name()) {
        throw std::runtime_error("USB_1208LS::USB_1208LS(): board number " + toStdString(boardNumber) + " is of type " + boardName + ", not " + this->name());
    }

    cbDConfigPort(this->m_boardNumber, this->digitalPortIDToUInt8(DigitalPortID::PortA), this->digitalPortDirectionToUInt8(PortDirection::DigitalInput));
    cbDConfigPort(this->m_boardNumber, this->digitalPortIDToUInt8(DigitalPortID::PortB), this->digitalPortDirectionToUInt8(PortDirection::DigitalInput));

    this->m_digitalPortMap.emplace(USB_1208LS::DigitalPortID::PortA, USB_1208LS::PortDirection::DigitalInput);
    this->m_digitalPortMap.emplace(USB_1208LS::DigitalPortID::PortB, USB_1208LS::PortDirection::DigitalInput);

    this->resetCounter();
}


USB_1208LS::USB_1208LS(USB_1208LS &&rhs) noexcept :
    USB_IO_Base{"USB_1208LS", rhs.m_boardNumber},
    m_boardNumber{rhs.m_boardNumber},
    m_digitalPortMap{std::move(rhs.m_digitalPortMap)},
    m_analogInputMode{rhs.m_analogInputMode}
{

}

USB_1208LS& USB_1208LS::operator=(USB_1208LS &&rhs) noexcept
{
    this->m_boardNumber = rhs.m_boardNumber;
    this->m_digitalPortMap = std::move(rhs.m_digitalPortMap);
    this->m_analogInputMode = rhs.m_analogInputMode;
    return *this;
}

void USB_1208LS::setDigitalPortDirection(DigitalPortID portID, PortDirection direction)
{
    auto currentPortDirection = this->m_digitalPortMap.find(portID)->second;
    if (currentPortDirection == direction) {
        return;
    }
    auto result = cbDConfigPort(this->m_boardNumber, digitalPortIDToUInt8(portID), digitalPortDirectionToUInt8(direction));
    if (result != NOERRORS) {
        throw std::runtime_error("USB_1208LS::setDigitalPortDirection(DigitalPortID, PortDirection): cbDConfigPort returned " + toStdString(result) + " (" + getErrorString(result) + ")");
    }
    if (direction == PortDirection::DigitalOutput) {
        cbDOut(this->m_boardNumber, digitalPortIDToUInt8(portID), 0x00);
    }
    this->m_digitalPortMap.find(portID)->second = direction;
}

uint8_t USB_1208LS::digitalPortIDToUInt8(DigitalPortID portID)
{
    if (portID == USB_1208LS::DigitalPortID::PortA) {
        return FIRSTPORTA;
    } else if (portID == USB_1208LS::DigitalPortID::PortB) {
        return FIRSTPORTB;
    } else {
        throw std::runtime_error("USB_1208LS::digitalPortIDToUInt8(DigitalPortID): Invalid DigitalPortID");
    }
}

uint8_t USB_1208LS::digitalPortDirectionToUInt8(PortDirection direction)
{
    if (direction == USB_1208LS::PortDirection::DigitalInput) {
        return SIGNAL_IN;
    } else if (direction == USB_1208LS::PortDirection::DigitalOutput) {
        return SIGNAL_OUT;
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
        throw std::runtime_error("USB_1208LS::digitalWrite(DigitalPortID, uint8_t, bool): pinNumber for ports A and B must be between 0 and " + toStdString(upperPinNumber) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(upperPinNumber));
    }
    if (this->m_digitalPortMap.find(portID)->second != USB_1208LS::PortDirection::DigitalOutput) {
        return false;
    }
    auto result = cbDBitOut(this->m_boardNumber,digitalPortIDToUInt8(portID), pinNumber, static_cast<uint8_t>(state));
    if (result != NOERRORS) {
        throw std::runtime_error("USB_1208LS::digitalWrite(DigitalPortID, uint8_t, bool): cbDBitOut returned " + toStdString(result) + " (" + getErrorString(result) + ")");
    }
    return true;
}

bool USB_1208LS::digitalRead(DigitalPortID portID, uint8_t pinNumber)
{
    int upperPinNumber{BITS_PER_PORT_1208LS};
    if (pinNumber >= upperPinNumber) {
        throw std::runtime_error("USB_1208LS::digitalWrite(DigitalPortID, uint8_t, bool): pinNumber for ports A and B must be between 0 and " + toStdString(upperPinNumber) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(upperPinNumber));
    }
    if (this->m_digitalPortMap.find(portID)->second != USB_1208LS::PortDirection::DigitalInput) {
        return false;
    }
    uint16_t allValues{0};
    cbDIn(this->m_boardNumber, digitalPortIDToUInt8(portID), &allValues);
    return static_cast<bool>(CHECK_BIT(allValues, pinNumber));
}



uint8_t USB_1208LS::voltageRangeToDifferentialGain(USB_1208LS::VoltageRange voltageRange)
{
    if (voltageRange == USB_1208LS::VoltageRange::V_20) {
        return BIP20VOLTS;
    } else if (voltageRange == USB_1208LS::VoltageRange::V_10) {
        return BIP10VOLTS;
    } else if (voltageRange == USB_1208LS::VoltageRange::V_5) {
        return BIP5VOLTS;
    } else if (voltageRange == USB_1208LS::VoltageRange::V_4) {
        return BIP4VOLTS;
    } else if (voltageRange == USB_1208LS::VoltageRange::V_2_5) {
        return BIP2PT5VOLTS;
    } else if (voltageRange == USB_1208LS::VoltageRange::V_2) {
        return BIP2VOLTS;
    } else if (voltageRange == USB_1208LS::VoltageRange::V_1_2_5) {
        return BIP1PT25VOLTS;
    } else if (voltageRange == USB_1208LS::VoltageRange::V_1) {
        return BIP1VOLTS;
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
    USHORT returnValue{0};
    if (this->m_analogInputMode == USB_1208LS::AnalogInputMode::SingleEnded) {
        auto result = cbAIn(this->m_boardNumber, pinNumber, UNI10VOLTS, &returnValue);
        if (result != NOERRORS) {
            throw std::runtime_error("USB_1208LS::analogRead(uint8_t, VoltageRange): cbAIn returned " + toStdString(result) + " (" + getErrorString(result) + ")");
        }
    } else {
        auto result = cbAIn(this->m_boardNumber, pinNumber, this->voltageRangeToDifferentialGain(voltageRange), &returnValue);
        if (result != NOERRORS) {
            throw std::runtime_error("USB_1208LS::analogRead(uint8_t, VoltageRange): cbAIn returned " + toStdString(result) + " (" + getErrorString(result) + ")");
        }
    }
    return returnValue;

}

float USB_1208LS::voltageRead(uint8_t pinNumber, USB_1208LS::VoltageRange voltageRange)
{
    short analogReading{this->analogRead(pinNumber, voltageRange)};
    float returnValue{0};
    if (this->m_analogInputMode == USB_1208LS::AnalogInputMode::SingleEnded) {
        auto result = cbToEngUnits (this->m_boardNumber, UNI10VOLTS, static_cast<USHORT>(analogReading), &returnValue);
        if (result != NOERRORS) {
            throw std::runtime_error("USB_1208LS::voltageRead(uint8_t, VoltageRange): cbToEngUnits returned " + toStdString(result) + " (" + getErrorString(result) + ")");
        }
    } else {
        auto result = cbToEngUnits(this->m_boardNumber, this->voltageRangeToDifferentialGain(voltageRange), static_cast<USHORT>(analogReading), &returnValue);
        if (result != NOERRORS) {
            throw std::runtime_error("USB_1208LS::voltageRead(uint8_t, VoltageRange): cbToEngUnits returned " + toStdString(result) + " (" + getErrorString(result) + ")");
        }
    }
    return returnValue;

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
        throw std::runtime_error("SB_1208LS::analogWrite(uint8_t, uint16_t): analogWrite pin number exceeds maximum pin number (" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(NUMBER_OF_ANALOG_OUTPUT_PINS - 1));
    }
    auto result = cbAOut(this->m_boardNumber, pinNumber, UNI5VOLTS, state);
    if (result != NOERRORS) {
        throw std::runtime_error("USB_1208LS::analogWrite(uint8_t, uint16_t): cbAOut returned " + toStdString(result) + " (" + getErrorString(result) + ")");
    }
}

std::string USB_1208LS::serialNumber() const
{
    return USB_IO_Base::getSerialNumber();
}

float USB_1208LS::analogToVoltage(short analogReading, USB_1208LS::AnalogInputMode inputMode, USB_1208LS::VoltageRange voltageRange)
{
    float returnValue{0};
    if (inputMode == USB_1208LS::AnalogInputMode::SingleEnded) {
        auto result = cbToEngUnits (this->m_boardNumber, UNI10VOLTS, static_cast<USHORT>(analogReading), &returnValue);
        if (result != NOERRORS) {
            throw std::runtime_error("USB_1208LS::voltageRead(uint8_t, VoltageRange): cbToEngUnits returned " + toStdString(result) + " (" + getErrorString(result) + ")");
        }
    } else {
        auto result = cbToEngUnits(this->m_boardNumber, this->voltageRangeToDifferentialGain(voltageRange), static_cast<USHORT>(analogReading), &returnValue);
        if (result != NOERRORS) {
            throw std::runtime_error("USB_1208LS::voltageRead(uint8_t, VoltageRange): cbToEngUnits returned " + toStdString(result) + " (" + getErrorString(result) + ")");
        }
    }
    return returnValue;
}

void USB_1208LS::resetDevice()
{
    //Nothing to do here
}

void USB_1208LS::resetCounter() {
    auto result = cbCClear (this->m_boardNumber, COUNT1);
    if (result != NOERRORS) {
        throw std::runtime_error("USB_1208LS::resetCounter(): cbCClear returned " + toStdString(result) + " (" + getErrorString(result) + ")");
    }
}

uint32_t USB_1208LS::readCounter() {
    ULONG returnValue{ 0 };
    auto result = cbCIn32(this->m_boardNumber, COUNT1, &returnValue);
    if (result != NOERRORS) {
        throw std::runtime_error("USB_1208LS::readCounter(): cbCIn32 returned " + toStdString(result) + " (" + getErrorString(result) + ")");
    }
    return static_cast<uint32_t>(returnValue);
}


USB_1208LS::~USB_1208LS()
{

}


} //namespace MeasurementComputingCpp
