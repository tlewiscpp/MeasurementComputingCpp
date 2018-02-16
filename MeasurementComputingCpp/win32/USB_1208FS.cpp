#include <cstring>

#include "USB_1208FS.hpp"

#include "common/cbw.h"

#define CHECK_BIT(value,bit) ((value) & (1 << (bit)))
#define SET_BIT(value,bit) ((value) |= (1 << (bit)))
#define CLEAR_BIT(value,bit) ((value) &= ~(1 << (bit)))
#define TOGGLE_BIT(value,bit) ((value) ^= (1 << (bit)))

namespace MeasurementComputingCpp {

USB_1208FS::USB_1208FS(unsigned int boardNumber) :
    USB_IO_Base{"USB-1208FS", boardNumber},
    m_boardNumber{boardNumber},
    m_digitalPortMap{},
    m_digitalOutputTracker{},
    m_analogInputMode{AnalogInputMode::SingleEnded}
{

	auto boardName = this->getBoardName(boardNumber);
	if (boardName != this->name()) {
		throw std::runtime_error("USB_1208FS::USB_1208FS(): board number " + toStdString(boardNumber) + " is of type " + boardName + ", not " + this->name());
	}

	cbDConfigPort(this->m_boardNumber, this->digitalPortIDToUInt8(DigitalPortID::PortA), this->digitalPortDirectionToUInt8(PortDirection::DigitalInput));
	cbDConfigPort(this->m_boardNumber, this->digitalPortIDToUInt8(DigitalPortID::PortB), this->digitalPortDirectionToUInt8(PortDirection::DigitalInput));

    this->m_digitalPortMap.emplace(USB_1208FS::DigitalPortID::PortA, USB_1208FS::PortDirection::DigitalInput);
    this->m_digitalPortMap.emplace(USB_1208FS::DigitalPortID::PortB, USB_1208FS::PortDirection::DigitalInput);

    this->m_digitalOutputTracker.emplace(USB_1208FS::DigitalPortID::PortA, 0);
    this->m_digitalOutputTracker.emplace(USB_1208FS::DigitalPortID::PortB, 0);

    this->resetCounter();
}


USB_1208FS::USB_1208FS(USB_1208FS &&rhs) noexcept :
    USB_IO_Base{"USB_1208FS", rhs.m_boardNumber},
    m_boardNumber{rhs.m_boardNumber},
    m_digitalPortMap{std::move(rhs.m_digitalPortMap)},
    m_digitalOutputTracker{std::move(rhs.m_digitalOutputTracker)},
    m_analogInputMode{rhs.m_analogInputMode}
{

}

USB_1208FS& USB_1208FS::operator=(USB_1208FS &&rhs) noexcept
{
    this->m_boardNumber = rhs.m_boardNumber;
    this->m_digitalPortMap = std::move(rhs.m_digitalPortMap);
    this->m_digitalOutputTracker = std::move(rhs.m_digitalOutputTracker);
    this->m_analogInputMode = rhs.m_analogInputMode;
    return *this;
}

void USB_1208FS::setDigitalPortDirection(DigitalPortID portID, PortDirection direction)
{
    auto currentPortDirection = this->m_digitalPortMap.find(portID)->second;
    if (currentPortDirection == direction) {
        return;
    }
    auto result = cbDConfigPort(this->m_boardNumber, digitalPortIDToUInt8(portID), digitalPortDirectionToUInt8(direction));
    if (result != NOERRORS) {
        throw std::runtime_error("USB_1208FS::setDigitalPortDirection(DigitalPortID, PortDirection): cbDConfigPort returned " + toStdString(result) + " (" + getErrorString(result) + ")");
    }
    if (direction == PortDirection::DigitalOutput) {
        cbDOut(this->m_boardNumber, digitalPortIDToUInt8(portID), 0x00);
    }
    this->m_digitalPortMap.find(portID)->second = direction;
}

uint8_t USB_1208FS::digitalPortIDToUInt8(DigitalPortID portID)
{
    if (portID == USB_1208FS::DigitalPortID::PortA) {
        return FIRSTPORTA;
    } else if (portID == USB_1208FS::DigitalPortID::PortB) {
        return FIRSTPORTB;
    } else {
        throw std::runtime_error("USB_1208FS::digitalPortIDToUInt8(DigitalPortID): Invalid DigitalPortID");
    }
}

uint8_t USB_1208FS::digitalPortDirectionToUInt8(PortDirection direction)
{
    if (direction == USB_1208FS::PortDirection::DigitalInput) {
        return SIGNAL_IN;
    } else if (direction == USB_1208FS::PortDirection::DigitalOutput) {
        return SIGNAL_OUT;
    } else {
        throw std::runtime_error("USB_1208FS::digitalPortDirectionToUInt8(PortDirection): Invalid PortDirection");
    }
}

USB_1208FS::PortDirection USB_1208FS::digitalPortDirection(USB_1208FS::DigitalPortID portID) const
{
    return this->m_digitalPortMap.find(portID)->second;
}

bool USB_1208FS::digitalWrite(DigitalPortID portID, uint8_t pinNumber, bool state)
{
    if (this->m_digitalPortMap.find(portID)->second != USB_1208FS::PortDirection::DigitalOutput) {
        return false;
    }
    //Load the last known bitset for the port (ex if pin 0 was already written high, the loaded value would be
    //0b10000000, so we need to preserve that while flipping our target bit
    //So if the pinNumber is 3, we toggle or reset that bit (depending on the state parameter).
    //For example, if state is high, the target bitset would change from 0b100000000 to 0b10010000
    //Then, we write it out the the port
    uint8_t *targetPortCurrentState{&(this->m_digitalOutputTracker.find(portID)->second)};
    state ? SET_BIT(*targetPortCurrentState, pinNumber) : CLEAR_BIT(*targetPortCurrentState, pinNumber);
    cbDOut(this->m_boardNumber, digitalPortIDToUInt8(portID), *targetPortCurrentState);
    return true;
}

bool USB_1208FS::digitalRead(DigitalPortID portID, uint8_t pinNumber)
{
    int upperPinNumber{BITS_PER_PORT_1208FS};
    if (pinNumber >= upperPinNumber) {
        throw std::runtime_error("USB_1208FS::digitalWrite(DigitalPortID, uint8_t, bool): pinNumber for ports A and B must be between 0 and " + toStdString(upperPinNumber) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(upperPinNumber));
    }
    if (this->m_digitalPortMap.find(portID)->second != USB_1208FS::PortDirection::DigitalInput) {
        return false;
    }
    uint16_t allValues{0};
    cbDIn(this->m_boardNumber, digitalPortIDToUInt8(portID), &allValues);
    return static_cast<bool>(CHECK_BIT(allValues, pinNumber));
}

USB_1208FS::~USB_1208FS()
{

}

uint8_t USB_1208FS::voltageRangeToDifferentialGain(USB_1208FS::VoltageRange voltageRange)
{
    if (voltageRange == USB_1208FS::VoltageRange::V_20) {
        return BIP20VOLTS;
    } else if (voltageRange == USB_1208FS::VoltageRange::V_10) {
        return BIP10VOLTS;
    } else if (voltageRange == USB_1208FS::VoltageRange::V_5) {
        return BIP5VOLTS;
    } else if (voltageRange == USB_1208FS::VoltageRange::V_4) {
        return BIP4VOLTS;
    } else if (voltageRange == USB_1208FS::VoltageRange::V_2_5) {
        return BIP2PT5VOLTS;
    } else if (voltageRange == USB_1208FS::VoltageRange::V_2) {
        return BIP2VOLTS;
    } else if (voltageRange == USB_1208FS::VoltageRange::V_1_2_5) {
        return BIP1PT25VOLTS;
    } else if (voltageRange == USB_1208FS::VoltageRange::V_1) {
        return BIP1VOLTS;
    }
    return 0;
}


short USB_1208FS::analogRead(uint8_t pinNumber, USB_1208FS::VoltageRange voltageRange)
{
    /*
    if ((this->m_analogInputMode == USB_1208FS::AnalogInputMode::SingleEnded) && (voltageRange != USB_1208FS::VoltageRange::V_10)) {
        throw std::runtime_error("analogRead voltage range can only be V_10 in SingleEnded AnalogInputMode");
    }
    */
    if ( (this->m_analogInputMode == USB_1208FS::AnalogInputMode::Differential) && (pinNumber >= (BITS_PER_PORT_1208FS/2) ) ) {
        throw std::runtime_error("USB_1208FS::analogRead(uint8_t, USB_1208FS::VoltageRange): analogRead pin number cannot be greater than \""
                                 + toStdString( (BITS_PER_PORT_1208FS/2) - 1)
                                 + "\" in Differential AnalogInputMode ("
                                 + toStdString(static_cast<int>(pinNumber))
                                 + " > "
                                 + toStdString( (BITS_PER_PORT_1208FS/2) - 1)
                                 + ")" );
    } else if ( (this->m_analogInputMode == USB_1208FS::AnalogInputMode::SingleEnded) && (pinNumber >= BITS_PER_PORT_1208FS) ) {
        throw std::runtime_error("USB_1208FS::analogRead(uint8_t, USB_1208FS::VoltageRange): analogRead pin number cannot be greater than \""
                                 + toStdString(BITS_PER_PORT_1208FS)
                                 + "\" in SingleEnded AnalogInputMode ("
                                 + toStdString(static_cast<int>(pinNumber))
                                 + " > "
                                 + toStdString(BITS_PER_PORT_1208FS)
                                 + ")" );
    }
    USHORT returnValue{0};
    if (this->m_analogInputMode == USB_1208FS::AnalogInputMode::SingleEnded) {
        auto result = cbAIn(this->m_boardNumber, pinNumber, UNI10VOLTS, &returnValue);
        if (result != NOERRORS) {
            throw std::runtime_error("USB_1208FS::analogRead(uint8_t, VoltageRange): cbAIn returned " + toStdString(result) + " (" + getErrorString(result) + ")");
        }
    } else {
        auto result = cbAIn(this->m_boardNumber, pinNumber, this->voltageRangeToDifferentialGain(voltageRange), &returnValue);
        if (result != NOERRORS) {
            throw std::runtime_error("USB_1208FS::analogRead(uint8_t, VoltageRange): cbAIn returned " + toStdString(result) + " (" + getErrorString(result) + ")");
        }
    }
    return returnValue;
}

float USB_1208FS::voltageRead(uint8_t pinNumber, USB_1208FS::VoltageRange voltageRange)
{
    short analogReading{this->analogRead(pinNumber, voltageRange)};
    float returnValue{0};
    if (this->m_analogInputMode == USB_1208FS::AnalogInputMode::SingleEnded) {
        auto result = cbToEngUnits (this->m_boardNumber, UNI10VOLTS, static_cast<USHORT>(analogReading), &returnValue);
        if (result != NOERRORS) {
            throw std::runtime_error("USB_1208FS::voltageRead(uint8_t, VoltageRange): cbToEngUnits returned " + toStdString(result) + " (" + getErrorString(result) + ")");
        }
    } else {
        auto result = cbToEngUnits(this->m_boardNumber, this->voltageRangeToDifferentialGain(voltageRange), static_cast<USHORT>(analogReading), &returnValue);
        if (result != NOERRORS) {
            throw std::runtime_error("USB_1208FS::voltageRead(uint8_t, VoltageRange): cbToEngUnits returned " + toStdString(result) + " (" + getErrorString(result) + ")");
        }
    }
    return returnValue;
}

void USB_1208FS::setAnalogInputMode(USB_1208FS::AnalogInputMode analogInputMode)
{
    this->m_analogInputMode = analogInputMode;
}

USB_1208FS::AnalogInputMode USB_1208FS::analogInputMode() const
{
    return this->m_analogInputMode;
}

void USB_1208FS::analogWrite(uint8_t pinNumber, uint16_t state)
{
    if (pinNumber > (NUMBER_OF_ANALOG_OUTPUT_PINS - 1)) {
        throw std::runtime_error("SB_1208FS::analogWrite(uint8_t, uint16_t): analogWrite pin number exceeds maximum pin number (" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(NUMBER_OF_ANALOG_OUTPUT_PINS - 1));
    }
    auto result = cbAOut(this->m_boardNumber, pinNumber, UNI5VOLTS, state);
    if (result != NOERRORS) {
        throw std::runtime_error("USB_1208FS::analogWrite(uint8_t, uint16_t): cbAOut returned " + toStdString(result) + " (" + getErrorString(result) + ")");
    }
}

std::string USB_1208FS::serialNumber() const
{
    return USB_IO_Base::getSerialNumber();
}

float USB_1208FS::analogToVoltage(short analogReading, AnalogInputMode inputMode, VoltageRange voltageRange) {
    float returnValue{0};
    if (inputMode == USB_1208FS::AnalogInputMode::SingleEnded) {
        auto result = cbToEngUnits (this->m_boardNumber, UNI10VOLTS, static_cast<USHORT>(analogReading), &returnValue);
        if (result != NOERRORS) {
            throw std::runtime_error("USB_1208FS::voltageRead(uint8_t, VoltageRange): cbToEngUnits returned " + toStdString(result) + " (" + getErrorString(result) + ")");
        }
    } else {
        auto result = cbToEngUnits(this->m_boardNumber, this->voltageRangeToDifferentialGain(voltageRange), static_cast<USHORT>(analogReading), &returnValue);
        if (result != NOERRORS) {
            throw std::runtime_error("USB_1208FS::voltageRead(uint8_t, VoltageRange): cbToEngUnits returned " + toStdString(result) + " (" + getErrorString(result) + ")");
        }
    }
    return returnValue;
}

void USB_1208FS::resetDevice() {
    //Nothing to do here
}

void USB_1208FS::resetCounter() {
    auto result = cbCClear (this->m_boardNumber, COUNT1);
    if (result != NOERRORS) {
        throw std::runtime_error("USB_1208FS::resetCounter(): cbCClear returned " + toStdString(result) + " (" + getErrorString(result) + ")");
    }
}

uint32_t USB_1208FS::readCounter() {
    ULONG returnValue{ 0 };
    auto result = cbCIn32(this->m_boardNumber, COUNT1, &returnValue);
    if (result != NOERRORS) {
        throw std::runtime_error("USB_1208FS::readCounter(): cbCIn32 returned " + toStdString(result) + " (" + getErrorString(result) + ")");
    }
    return static_cast<uint32_t>(returnValue);
}



} //namespace MeasurementComputingCpp
