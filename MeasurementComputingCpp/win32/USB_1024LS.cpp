#include <mutex>
#include <cstring>
#include <cstdlib>

#include "common/cbw.h"

#include "USB_1024LS.hpp"

#define CHECK_BIT(value,bit) ((value) & (1 << (bit)))

namespace MeasurementComputingCpp {

USB_1024LS::USB_1024LS(unsigned int boardNumber) :
    USB_IO_Base{"USB_1024LS", boardNumber},
	m_boardNumber{ boardNumber },
    m_digitalPortMap{}
{

    auto boardName =this->getBoardName(boardNumber);
    if (boardName != this->name()) {
        throw std::runtime_error("USB_1024LS::USB_1024LS(): board number " + toStdString(boardNumber) + " is of type " + boardName + ", not " + this->name());
    }

    cbDConfigPort (this->m_boardNumber, this->digitalPortIDToUInt8(DigitalPortID::PortA), this->digitalPortDirectionToUInt8(PortDirection::DigitalInput));
    cbDConfigPort (this->m_boardNumber, this->digitalPortIDToUInt8(DigitalPortID::PortB), this->digitalPortDirectionToUInt8(PortDirection::DigitalInput));
    cbDConfigPort (this->m_boardNumber, this->digitalPortIDToUInt8(DigitalPortID::PortCLow), this->digitalPortDirectionToUInt8(PortDirection::DigitalInput));
    cbDConfigPort (this->m_boardNumber, this->digitalPortIDToUInt8(DigitalPortID::PortCHigh), this->digitalPortDirectionToUInt8(PortDirection::DigitalInput));

    this->m_digitalPortMap.emplace(USB_1024LS::DigitalPortID::PortA, USB_1024LS::PortDirection::DigitalInput);
    this->m_digitalPortMap.emplace(USB_1024LS::DigitalPortID::PortB, USB_1024LS::PortDirection::DigitalInput);
    this->m_digitalPortMap.emplace(USB_1024LS::DigitalPortID::PortCLow, USB_1024LS::PortDirection::DigitalInput);
    this->m_digitalPortMap.emplace(USB_1024LS::DigitalPortID::PortCHigh, USB_1024LS::PortDirection::DigitalInput);

    this->resetCounter();

}

USB_1024LS::USB_1024LS(USB_1024LS &&rhs) noexcept :
    USB_IO_Base{"USB_1024LS", rhs.m_boardNumber},
    m_boardNumber{rhs.m_boardNumber},
    m_digitalPortMap{std::move(rhs.m_digitalPortMap)}
{

}

USB_1024LS& USB_1024LS::operator=(USB_1024LS &&rhs) noexcept
{
    this->m_boardNumber = rhs.m_boardNumber;
    this->m_digitalPortMap = std::move(rhs.m_digitalPortMap);
    return *this;
}

uint8_t USB_1024LS::digitalPortIDToUInt8(DigitalPortID portID)
{
    if (portID == USB_1024LS::DigitalPortID::PortA) {
        return FIRSTPORTA;
    } else if (portID == USB_1024LS::DigitalPortID::PortB) {
        return FIRSTPORTB;
    } else if (portID == USB_1024LS::DigitalPortID::PortCLow) {
        return FIRSTPORTCL;
    } else if (portID == USB_1024LS::DigitalPortID::PortCHigh) {
		return FIRSTPORTCH;
    } else {
        throw std::runtime_error("USB_1024LS::digitalPortIDToUInt8(DigitalPortID): Invalid DigitalPortID");
    }
}

uint8_t USB_1024LS::digitalPortDirectionToUInt8(PortDirection direction)
{
    if (direction == USB_1024LS::PortDirection::DigitalInput) {
        return SIGNAL_IN;
    } else if (direction == USB_1024LS::PortDirection::DigitalOutput) {
        return SIGNAL_OUT;
    } else {
        throw std::runtime_error("USB_1024LS::digitalPortDirectionToUInt8(PortDirection): Invalid PortDirection");
    }
}

void USB_1024LS::setDigitalPortDirection(DigitalPortID portID, PortDirection direction)
{
    auto currentPortDirection = this->m_digitalPortMap.find(portID)->second;
    if (currentPortDirection == direction) {
        return;
    }
    auto result = cbDConfigPort(this->m_boardNumber, digitalPortIDToUInt8(portID), digitalPortDirectionToUInt8(direction));
	if (result != NOERRORS) {
		throw std::runtime_error("USB_1024LS::setDigitalPortDirection(DigitalPortID, PortDirection): cbDConfigPort returned " + toStdString(result) + " (" + getErrorString(result) + ")");
	}
    if (direction == PortDirection::DigitalOutput) {
        result  = cbDOut(this->m_boardNumber, digitalPortIDToUInt8(portID), 0x00); //0b00000000
		if (result != NOERRORS) {
			throw std::runtime_error("USB_1024LS::setDigitalPortDirection(DigitalPortID, PortDirection): cbDOut returned " + toStdString(result) + " (" + getErrorString(result) + ")");
		}
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
    auto result = cbDBitOut(this->m_boardNumber, digitalPortIDToUInt8(portID), pinNumber, static_cast<uint8_t>(state));
	if (result != NOERRORS) {
		throw std::runtime_error("USB_1024LS::digitalWrite(DigitalPortID, uint8_t, bool): cbDBitOut returned " + toStdString(result) + " (" + getErrorString(result) + ")");
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
    uint16_t allValues{0};
    auto result = cbDIn(this->m_boardNumber, this->digitalPortIDToUInt8(portID), &allValues);
    if (result != NOERRORS) {
        throw std::runtime_error("USB_1024LS::digitalRead(DigitalPortID, uint8_t): cbDIn returned " + toStdString(result) + " (" + getErrorString(result) + ")");
    }
    return static_cast<bool>(CHECK_BIT(allValues, pinNumber));
}

std::string USB_1024LS::serialNumber() const
{
	return USB_IO_Base::getSerialNumber();
}

void USB_1024LS::resetDevice() {
	//Nothing to do here
}

void USB_1024LS::resetCounter() {
    auto result = cbCClear (this->m_boardNumber, COUNT1);
	if (result != NOERRORS) {
		throw std::runtime_error("USB_1024LS::resetCounter(): cbCClear returned " + toStdString(result) + " (" + getErrorString(result) + ")");
	}
}

uint32_t USB_1024LS::readCounter() {
	ULONG returnValue{ 0 };
	auto result = cbCIn32(this->m_boardNumber, COUNT1, &returnValue);
	if (result != NOERRORS) {
		throw std::runtime_error("USB_1024LS::readCounter(): cbCIn32 returned " + toStdString(result) + " (" + getErrorString(result) + ")");
	}
	return static_cast<uint32_t>(returnValue);
}

USB_1024LS::~USB_1024LS()
{

}


} //namespace MeasurementComputingCpp
