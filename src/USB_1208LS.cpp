#include <mcc-libusb/pmd.h>
#include <mcc-libusb/usb-1208LS.h>

#include "USB_1208LS.hpp"

#include <cstring>
#include <cstdlib>

#define CHECK_BIT(value,bit) ((value) & (1 << (bit)))
#define SET_BIT(value,bit) ((value) |= (1 << (bit)))
#define CLEAR_BIT(value,bit) ((value) &= ~(1 << (bit)))
#define TOGGLE_BIT(value,bit) ((value) ^= (1 << (bit)))

namespace MeasurementComputingCpp {

#define BITS_PER_PORT_1208LS 8
#define MAXIMUM_ANALOG_OUTPUT_VALUE_1208LS 65535
#define NUMBER_OF_ANALOG_OUTPUT_PINS 2
#define SERIAL_NUMBER_BUFFER_1208LS 255

constexpr auto PORT_A_MAX_PIN_NUMBER = BITS_PER_PORT_1208LS;
constexpr auto PORT_B_MAX_PIN_NUMBER = (BITS_PER_PORT_1208LS*2);

USB_1208LS::USB_1208LS() :
    USB_IO_Base{"USB_1208LS"},
    m_hidDevice{nullptr},
    m_serialNumber{""},
    m_digitalPortMap{},
    m_digitalOutputTracker{},
    m_analogOutputTracker{},
    m_analogInputMode{AnalogInputMode::SingleEnded}
{

    int initResult{hid_init()};
    if (initResult != 0) {
        throw std::runtime_error("USB_1208S::USB_1208LS: hid_init failed with return code " + toStdString(initResult));
    }

    this->m_digitalPortMap.emplace(USB_1208LS::DigitalPortID::PortA, USB_1208LS::PortDirection::DigitalInput);
    this->m_digitalPortMap.emplace(USB_1208LS::DigitalPortID::PortB, USB_1208LS::PortDirection::DigitalInput);

    this->m_digitalOutputTracker.emplace(USB_1208LS::DigitalPortID::PortA, 0);
    this->m_digitalOutputTracker.emplace(USB_1208LS::DigitalPortID::PortB, 0);

    this->m_analogOutputTracker.emplace(0, 0); //Analog Output 0
    this->m_analogOutputTracker.emplace(1, 0); //Analog Output 1

    this->initialize();

}


USB_1208LS::USB_1208LS(USB_1208LS &&rhs) noexcept :
    USB_IO_Base{"USB_1208LS"},
    m_hidDevice{rhs.m_hidDevice},
    m_serialNumber{std::move(rhs.m_serialNumber)},
    m_digitalPortMap{std::move(rhs.m_digitalPortMap)},
    m_digitalOutputTracker{std::move(rhs.m_digitalOutputTracker)},
    m_analogOutputTracker{std::move(rhs.m_analogOutputTracker)},
    m_analogInputMode{rhs.m_analogInputMode}
{

}

USB_1208LS& USB_1208LS::operator=(USB_1208LS &&rhs) noexcept {
    this->m_hidDevice = rhs.m_hidDevice;
    this->m_serialNumber = std::move(rhs.m_serialNumber);
    this->m_digitalPortMap = std::move(rhs.m_digitalPortMap);
    this->m_digitalOutputTracker = std::move(rhs.m_digitalOutputTracker);
    this->m_analogOutputTracker = std::move(rhs.m_analogOutputTracker);
    this->m_analogInputMode = rhs.m_analogInputMode;
    return *this;
}

USB_1208LS &USB_1208LS::setDigitalPortDirection(DigitalPortID portID, PortDirection direction) {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    auto currentPortDirection = this->m_digitalPortMap.find(portID)->second;
    //if (currentPortDirection == direction) {
    //    return *this;
    //}
    usbDConfigPort_USB1208LS(this->m_hidDevice, digitalPortIDToUInt8(portID), digitalPortDirectionToUInt8(direction));
    if (direction == PortDirection::DigitalOutput) {
        if (direction == currentPortDirection) {
            //If the direction is equal to the port direction, this is a reinitialization
            auto lastKnownState = this->m_digitalOutputTracker.find(portID)->second;
            if (this->m_hidDevice) {
                usbDOut_USB1208LS(this->m_hidDevice, digitalPortIDToUInt8(portID), lastKnownState); //0b00000000
            }
        } else {
            //Otherwise, clear out and write zeroes to port
            if (this->m_hidDevice) {
                usbDOut_USB1208LS(this->m_hidDevice, digitalPortIDToUInt8(portID), 0x00); //0b00000000
                this->m_digitalOutputTracker.find(portID)->second = 0x00;
            }
        }
    }
    this->m_digitalPortMap.find(portID)->second = direction;
    return *this;
}

uint8_t USB_1208LS::digitalPortIDToUInt8(DigitalPortID portID) {
    if (portID == USB_1208LS::DigitalPortID::PortA) {
        return DIO_PORTA;
    } else if (portID == USB_1208LS::DigitalPortID::PortB) {
        return DIO_PORTB;
    } else {
        throw std::runtime_error("USB_1208LS::digitalPortIDToUInt8: Invalid DigitalPortID");
    }
}

uint8_t USB_1208LS::digitalPortDirectionToUInt8(PortDirection direction) {
    if (direction == USB_1208LS::PortDirection::DigitalInput) {
        return DIO_DIR_IN;
    } else if (direction == USB_1208LS::PortDirection::DigitalOutput) {
        return DIO_DIR_OUT;
    } else {
        throw std::runtime_error("USB_1208LS::digitalPortDirectionToUInt8: Invalid PortDirection");
    }
}


USB_1208LS::PortDirection USB_1208LS::digitalPortDirection(USB_1208LS::DigitalPortID portID) const {
    return this->m_digitalPortMap.find(portID)->second;
}

bool USB_1208LS::digitalWrite(DigitalPortID portID, uint8_t pinNumber, bool state) {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    if (this->m_digitalPortMap.find(portID)->second != USB_1208LS::PortDirection::DigitalOutput) {
        this->setDigitalPortDirection(portID, USB_1208LS::PortDirection::DigitalOutput);
    }
    //Track the change in the digital output tracker
    uint8_t *targetPortCurrentState{&(this->m_digitalOutputTracker.find(portID)->second)};
    state ? SET_BIT(*targetPortCurrentState, pinNumber) : CLEAR_BIT(*targetPortCurrentState, pinNumber);
    this->digitalWritePort(portID, *targetPortCurrentState);
    return true;
}


USB_1208LS &USB_1208LS::digitalWritePort(DigitalPortID portID, uint8_t state) {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    if (this->m_digitalPortMap.find(portID)->second != USB_1208LS::PortDirection::DigitalOutput) {
        this->setDigitalPortDirection(portID, USB_1208LS::PortDirection::DigitalOutput);
    }
    if (this->m_hidDevice) {
        uint8_t *targetPortCurrentState{&(this->m_digitalOutputTracker.find(portID)->second)};
        *targetPortCurrentState = state;
        usbDOut_USB1208LS(this->m_hidDevice, digitalPortIDToUInt8(portID), *targetPortCurrentState);
    }
    return *this;
}

uint8_t USB_1208LS::digitalReadPort(DigitalPortID portID) {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    if (this->m_digitalPortMap.find(portID)->second != USB_1208LS::PortDirection::DigitalInput) {
        this->setDigitalPortDirection(portID, USB_1208LS::PortDirection::DigitalInput);
    }
    uint8_t allValues{0};
    if (this->m_hidDevice) {
        usbDIn_USB1208LS(this->m_hidDevice, digitalPortIDToUInt8(portID), &allValues);
        return allValues;
    } else {
        return 0;
    }
}

bool USB_1208LS::digitalRead(DigitalPortID portID, uint8_t pinNumber) {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    int upperPinNumber{BITS_PER_PORT_1208LS};
    if (pinNumber >= upperPinNumber) {
        throw std::runtime_error("USB_1208LS::digitalRead: pinNumber for ports A and B must be between 0 and " + toStdString(upperPinNumber) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(upperPinNumber));
    }
    if (this->m_digitalPortMap.find(portID)->second != USB_1208LS::PortDirection::DigitalInput) {
        this->setDigitalPortDirection(portID, USB_1208LS::PortDirection::DigitalInput);
    }
    uint8_t allValues{this->digitalReadPort(portID)};
    return static_cast<bool>(CHECK_BIT(allValues, pinNumber));
}

bool USB_1208LS::digitalRead(uint8_t pinNumber) {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    DigitalPortID portID{};
    uint8_t adjustedPinNumber{};
    auto result = getDigitalPortIDAndPinNumber(pinNumber, &portID, &adjustedPinNumber);
    if (!result) {
        throw std::runtime_error("USB_1208LS::digitalRead: pinNumber must be between 0 and " + toStdString(PORT_B_MAX_PIN_NUMBER) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(PORT_B_MAX_PIN_NUMBER));
    }
    return this->digitalRead(portID, adjustedPinNumber);
}

bool USB_1208LS::digitalWrite(uint8_t pinNumber, bool state) {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    DigitalPortID portID{};
    uint8_t adjustedPinNumber{};
    auto result = getDigitalPortIDAndPinNumber(pinNumber, &portID, &adjustedPinNumber);
    if (!result) {
        throw std::runtime_error("USB_1208LS::digitalWrite: pinNumber must be between 0 and " + toStdString(PORT_B_MAX_PIN_NUMBER) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(PORT_B_MAX_PIN_NUMBER));
    }
    return this->digitalWrite(portID, adjustedPinNumber, state);
}

bool USB_1208LS::getDigitalPortIDAndPinNumber(uint8_t pinNumber, DigitalPortID *outPortID, uint8_t *outAdjustedPinNumber) {
    if (pinNumber < PORT_A_MAX_PIN_NUMBER) {
        *outPortID = DigitalPortID::PortA;
        *outAdjustedPinNumber = pinNumber;
    } else if ( (pinNumber >= PORT_A_MAX_PIN_NUMBER) && (pinNumber < PORT_B_MAX_PIN_NUMBER) ) {
        *outPortID = DigitalPortID::PortB;
        *outAdjustedPinNumber = pinNumber - BITS_PER_PORT_1208LS;
    } else {
        return false;
    }
    return true;
}


uint8_t USB_1208LS::voltageRangeToDifferentialGain(USB_1208LS::VoltageRange voltageRange) {
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


short USB_1208LS::analogRead(uint8_t pinNumber, USB_1208LS::VoltageRange voltageRange) {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    /*
    if ((this->m_analogInputMode == USB_1208LS::AnalogInputMode::SingleEnded) && (voltageRange != USB_1208LS::VoltageRange::V_10)) {
        throw std::runtime_error("analogRead voltage range can only be V_10 in SingleEnded AnalogInputMode");
    }
    */
    if ( (this->m_analogInputMode == USB_1208LS::AnalogInputMode::Differential) && (pinNumber >= (BITS_PER_PORT_1208LS/2) ) ) {
        throw std::runtime_error("USB_1208LS::analogRead: analogRead pin number cannot be greater than \""
                                 + toStdString( (BITS_PER_PORT_1208LS/2) - 1)
                                 + "\" in Differential AnalogInputMode ("
                                 + toStdString(static_cast<int>(pinNumber))
                                 + " > "
                                 + toStdString( (BITS_PER_PORT_1208LS/2) - 1)
                                 + ")" );
    } else if ( (this->m_analogInputMode == USB_1208LS::AnalogInputMode::SingleEnded) && (pinNumber >= BITS_PER_PORT_1208LS) ) {
        throw std::runtime_error("USB_1208LS::analogRead: analogRead pin number cannot be greater than \""
                                 + toStdString(BITS_PER_PORT_1208LS)
                                 + "\" in SingleEnded AnalogInputMode ("
                                 + toStdString(static_cast<int>(pinNumber))
                                 + " > "
                                 + toStdString(BITS_PER_PORT_1208LS)
                                 + ")" );
    }
    if (this->m_analogInputMode == USB_1208LS::AnalogInputMode::SingleEnded) {
        if (this->m_hidDevice) {
            return usbAIn_USB1208LS(this->m_hidDevice, pinNumber, SE_10_00V);
        } else {
            return 0;
        }
    } else {
        if (this->m_hidDevice) {
            return usbAIn_USB1208LS(this->m_hidDevice, pinNumber, this->voltageRangeToDifferentialGain(voltageRange));
        } else {
            return 0;
        }
    }

}

float USB_1208LS::voltageRead(uint8_t pinNumber, USB_1208LS::VoltageRange voltageRange) {
    short analogReading{this->analogRead(pinNumber, voltageRange)};
    if (this->m_analogInputMode == USB_1208LS::AnalogInputMode::SingleEnded) {
        return volts_LS(analogReading, SE_10_00V);
    } else {
        return volts_LS(this->voltageRangeToDifferentialGain(voltageRange), analogReading);
    }

}

USB_1208LS & USB_1208LS::setAnalogInputMode(USB_1208LS::AnalogInputMode analogInputMode) {
    this->m_analogInputMode = analogInputMode;
    return *this;
}

USB_1208LS::AnalogInputMode USB_1208LS::analogInputMode() const {
    return this->m_analogInputMode;
}

USB_1208LS & USB_1208LS::analogWrite(uint8_t pinNumber, uint16_t state) {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    if (pinNumber > (NUMBER_OF_ANALOG_OUTPUT_PINS - 1)) {
        throw std::runtime_error("USB_1208LS::analogWrite: analogWrite pin number exceeds maximum pin number (" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(NUMBER_OF_ANALOG_OUTPUT_PINS - 1));
    }
    this->m_analogOutputTracker.find(pinNumber)->second = state;
    usbAOut_USB1208LS(this->m_hidDevice, pinNumber, state);
    return *this;
}

std::string USB_1208LS::serialNumber() const {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    if (!this->m_serialNumber.empty()) {
        return this->m_serialNumber;
    }
    wchar_t tempWideSerialNumber[SERIAL_NUMBER_BUFFER_1208LS];
    wmemset(tempWideSerialNumber, '\0', SERIAL_NUMBER_BUFFER_1208LS);
    if (this->m_hidDevice) {
        hid_get_serial_number_string(this->m_hidDevice, tempWideSerialNumber, SERIAL_NUMBER_BUFFER_1208LS);
    }

    char tempSerialNumber[SERIAL_NUMBER_BUFFER_1208LS];
    memset(tempSerialNumber, '\0', SERIAL_NUMBER_BUFFER_1208LS);
    wcstombs(tempSerialNumber, tempWideSerialNumber, SERIAL_NUMBER_BUFFER_1208LS);
    this->m_serialNumber = std::string{tempSerialNumber};
    return this->m_serialNumber;
}

float USB_1208LS::analogToVoltage(short analogReading, USB_1208LS::AnalogInputMode inputMode, USB_1208LS::VoltageRange voltageRange) {
    if (inputMode == AnalogInputMode::SingleEnded) {
        return volts_LS(analogReading, SE_10_00V);
    } else {
        return volts_LS(analogReading, voltageRangeToDifferentialGain(voltageRange));
    }
}

USB_1208LS & USB_1208LS::resetDevice() {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    if (this->m_hidDevice) {
        usbReset_USB1208LS(this->m_hidDevice);
    }
    return *this;
}

USB_1208LS & USB_1208LS::resetCounter() {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    if (this->m_hidDevice) {
        usbInitCounter_USB1208LS(this->m_hidDevice);
    }
    return *this;
}

uint32_t USB_1208LS::readCounter() {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    if (this->m_hidDevice) {
        return usbReadCounter_USB1208LS(this->m_hidDevice);
    } else {
        return 0;
    }
}

USB_IO_Base &USB_1208LS::reinitialize() {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    this->deinitialize();
    this->initialize();
    return *this;
}

USB_IO_Base &USB_1208LS::initialize() {
    if (this->m_hidDevice != nullptr) {
        this->deinitialize();
    }
    this->m_hidDevice = hid_open(MCC_VID, USB1208LS_PID, nullptr);
    if (!this->m_hidDevice) {
        throw std::runtime_error("USB_1208LS::USB_1208LS: USB1208LS device NOT found");
    }

    for (const auto &it : this->m_digitalPortMap) {
        this->setDigitalPortDirection(it.first, it.second);
    }

    this->resetCounter();
    return *this;
}

USB_IO_Base &USB_1208LS::deinitialize() {
    if (this->m_hidDevice != nullptr) {
        hid_close(this->m_hidDevice);
    }
    this->m_hidDevice = nullptr;
    this->m_serialNumber = "";
    return *this;
}


USB_1208LS::~USB_1208LS() {
    this->deinitialize();
}


} //namespace src
