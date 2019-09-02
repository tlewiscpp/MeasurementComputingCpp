#include <cstring>

#include <mcc-libusb/pmd.h>
#include <mcc-libusb/usb-1208FS.h>

#include "USB_1208FS.hpp"

#define CHECK_BIT(value,bit) ((value) & (1 << (bit)))
#define SET_BIT(value,bit) ((value) |= (1 << (bit)))
#define CLEAR_BIT(value,bit) ((value) &= ~(1 << (bit)))
#define TOGGLE_BIT(value,bit) ((value) ^= (1 << (bit)))

namespace MeasurementComputingCpp {

constexpr auto PORT_A_MAX_PIN_NUMBER = BITS_PER_PORT_1208FS;
constexpr auto PORT_B_MAX_PIN_NUMBER = (BITS_PER_PORT_1208FS*2);

USB_1208FS::USB_1208FS() :
    USB_IO_Base{"USB_1208FS"},
    m_usbDeviceHandle{nullptr},
    m_serialNumber{""},
    m_digitalPortMap{},
    m_digitalOutputTracker{},
    m_analogInputMode{AnalogInputMode::SingleEnded}
{

    int initResult{libusb_init(nullptr)};
    if (initResult != 0) {
        throw std::runtime_error("USB_1208FS::USB_1208FS(): libusb_init failed with return code " + toStdString(initResult));
    }

    this->m_digitalPortMap.emplace(USB_1208FS::DigitalPortID::PortA, USB_1208FS::PortDirection::DigitalInput);
    this->m_digitalPortMap.emplace(USB_1208FS::DigitalPortID::PortB, USB_1208FS::PortDirection::DigitalInput);

    this->m_digitalOutputTracker.emplace(USB_1208FS::DigitalPortID::PortA, 0);
    this->m_digitalOutputTracker.emplace(USB_1208FS::DigitalPortID::PortB, 0);

    this->m_analogOutputTracker.emplace(0, 0); //Analog Output 0
    this->m_analogOutputTracker.emplace(1, 0); //Analog Output 1

    this->initialize();
}


USB_1208FS::USB_1208FS(USB_1208FS &&rhs) noexcept :
    USB_IO_Base{"USB_1208FS"},
    m_usbDeviceHandle{rhs.m_usbDeviceHandle},
    m_serialNumber{std::move(rhs.m_serialNumber)},
    m_digitalPortMap{std::move(rhs.m_digitalPortMap)},
    m_digitalOutputTracker{std::move(rhs.m_digitalOutputTracker)},
    m_analogInputMode{rhs.m_analogInputMode}
{

}

USB_1208FS &USB_1208FS::operator=(USB_1208FS &&rhs) noexcept {
    this->m_usbDeviceHandle = rhs.m_usbDeviceHandle;
    this->m_serialNumber = std::move(rhs.m_serialNumber);
    this->m_digitalPortMap = std::move(rhs.m_digitalPortMap);
    this->m_digitalOutputTracker = std::move(rhs.m_digitalOutputTracker);
    this->m_analogInputMode = rhs.m_analogInputMode;
    return *this;
}

uint8_t USB_1208FS::digitalReadPort(DigitalPortID portID) {

}

USB1208FS &USB_1208FS::digitalWritePort(DigitalPortID portID, uint8_t state) {

}

USB_1208FS &USB_1208FS::setDigitalPortDirection(DigitalPortID portID, PortDirection direction) {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    auto currentPortDirection = this->m_digitalPortMap.find(portID)->second;
    //if (currentPortDirection == direction) {
    //    return *this;
    //}
    auto configResult = usbDConfigPort_USB1208FS(this->m_usbDeviceHandle, digitalPortIDToUInt8(portID), digitalPortDirectionToUInt8(direction));
    if (configResult < 0) {
        throw std::runtime_error("USB1208FS::setDigitalPortDirection: Failed to config port");
    }
    if (direction == PortDirection::DigitalOutput) {
        if (direction == currentPortDirection) {
            //If the direction is equal to the port direction, this is a reinitialization
            auto lastKnownState = this->m_digitalOutputTracker.find(portID)->second;
            if (this->m_usbDeviceHandle) {
                auto result = usbDOut_USB1208FS(this->m_usbDeviceHandle, digitalPortIDToUInt8(portID), lastKnownState); //0b00000000
                if (result < 0) {
                    throw std::runtime_error("USB1208FS::setDigitalPortDirection: Failed to write port");
                }
            }
        } else {
            //Otherwise, clear out and write zeroes to port
            if (this->m_usbDeviceHandle) {
                auto result = usbDOut_USB1208FS(this->m_usbDeviceHandle, digitalPortIDToUInt8(portID), 0x00); //0b00000000
                if (result < 0) {
                    throw std::runtime_error("USB1208FS::setDigitalPortDirection: Failed to write port");
                }
                this->m_digitalOutputTracker.find(portID)->second = 0x00;
            }
        }
    }
    this->m_digitalPortMap.find(portID)->second = direction;
    return *this;
}

uint8_t USB_1208FS::digitalPortIDToUInt8(DigitalPortID portID) {
    if (portID == USB_1208FS::DigitalPortID::PortA) {
        return DIO_PORTA;
    } else if (portID == USB_1208FS::DigitalPortID::PortB) {
        return DIO_PORTB;
    } else {
        throw std::runtime_error("USB_1208FS::digitalPortIDToUInt8(DigitalPortID): Invalid DigitalPortID");
    }
}

uint8_t USB_1208FS::digitalPortDirectionToUInt8(PortDirection direction) {
    if (direction == USB_1208FS::PortDirection::DigitalInput) {
        return DIO_DIR_IN;
    } else if (direction == USB_1208FS::PortDirection::DigitalOutput) {
        return DIO_DIR_OUT;
    } else {
        throw std::runtime_error("USB_1208FS::digitalPortDirectionToUInt8(PortDirection): Invalid PortDirection");
    }
}

USB_1208FS::PortDirection USB_1208FS::digitalPortDirection(USB_1208FS::DigitalPortID portID) const {
    return this->m_digitalPortMap.find(portID)->second;
}

bool USB_1208FS::digitalWrite(DigitalPortID portID, uint8_t pinNumber, bool state) {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    if (this->m_digitalPortMap.find(portID)->second != USB_1208FS::PortDirection::DigitalOutput) {
        this->setDigitalPortDirection(portID, USB_1208FS::PortDirection::DigitalOutput);
    }
    //Load the last known bitset for the port (ex if pin 0 was already written high, the loaded value would be
    //0b10000000, so we need to preserve that while flipping our target bit
    //So if the pinNumber is 3, we toggle or reset that bit (depending on the state parameter).
    //For example, if state is high, the target bitset would change from 0b100000000 to 0b10010000
    //Then, we write it out the the port
    uint8_t *targetPortCurrentState{&(this->m_digitalOutputTracker.find(portID)->second)};
    state ? SET_BIT(*targetPortCurrentState, pinNumber) : CLEAR_BIT(*targetPortCurrentState, pinNumber);
    if (this->m_usbDeviceHandle) {
        auto result = usbDOut_USB1208FS(this->m_usbDeviceHandle, digitalPortIDToUInt8(portID), *targetPortCurrentState);
        if (result < 0) {
            throw std::runtime_error("USB1208FS::digitalWrite: Failed to write port");
        }
    }
    return true;
}

bool USB_1208FS::digitalRead(DigitalPortID portID, uint8_t pinNumber) {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    int upperPinNumber{BITS_PER_PORT_1208FS};
    if (pinNumber >= upperPinNumber) {
        throw std::runtime_error("USB_1208FS::digitalWrite(DigitalPortID, uint8_t, bool): pinNumber for ports A and B must be between 0 and " + toStdString(upperPinNumber) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(upperPinNumber));
    }
    if (this->m_digitalPortMap.find(portID)->second != USB_1208FS::PortDirection::DigitalInput) {
        return false;
    }
    uint8_t allValues{0};
    if (this->m_usbDeviceHandle) {
        usbDIn_USB1208FS(this->m_usbDeviceHandle, digitalPortIDToUInt8(portID), &allValues);
        return static_cast<bool>(CHECK_BIT(allValues, pinNumber));
    } else {
        return false;
    }

}

bool USB_1208FS::digitalRead(uint8_t pinNumber) {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    DigitalPortID portID{};
    uint8_t adjustedPinNumber{};
    auto result = getDigitalPortIDAndPinNumber(pinNumber, &portID, &adjustedPinNumber);
    if (!result) {
        throw std::runtime_error("USB_1024LS::digitalRead(bool): pinNumber must be between 0 and " + toStdString(PORT_B_MAX_PIN_NUMBER) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(PORT_B_MAX_PIN_NUMBER));
    }
    return this->digitalRead(portID, adjustedPinNumber);
}

bool USB_1208FS::digitalWrite(uint8_t pinNumber, bool state) {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    DigitalPortID portID{};
    uint8_t adjustedPinNumber{};
    auto result = getDigitalPortIDAndPinNumber(pinNumber, &portID, &adjustedPinNumber);
    if (!result) {
        throw std::runtime_error("USB_1024LS::digitalWrite(uint8_t, bool): pinNumber must be between 0 and " + toStdString(PORT_B_MAX_PIN_NUMBER) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(PORT_B_MAX_PIN_NUMBER));
    }
    return this->digitalWrite(portID, adjustedPinNumber, state);
}

bool USB_1208FS::getDigitalPortIDAndPinNumber(uint8_t pinNumber, DigitalPortID *outPortID, uint8_t *outAdjustedPinNumber) {
    if (pinNumber < PORT_A_MAX_PIN_NUMBER) {
        *outPortID = DigitalPortID::PortA;
        *outAdjustedPinNumber = pinNumber;
    } else if ( (pinNumber >= PORT_A_MAX_PIN_NUMBER) && (pinNumber < PORT_B_MAX_PIN_NUMBER) ) {
        *outPortID = DigitalPortID::PortB;
        *outAdjustedPinNumber = pinNumber - BITS_PER_PORT_1208FS;
    } else {
        return false;
    }
    return true;
}

uint8_t USB_1208FS::voltageRangeToDifferentialGain(USB_1208FS::VoltageRange voltageRange) {
    if (voltageRange == USB_1208FS::VoltageRange::V_20) {
        return BP_20_00V;
    } else if (voltageRange == USB_1208FS::VoltageRange::V_10) {
        return BP_10_00V;
    } else if (voltageRange == USB_1208FS::VoltageRange::V_5) {
        return BP_5_00V;
    } else if (voltageRange == USB_1208FS::VoltageRange::V_4) {
        return BP_4_00V;
    } else if (voltageRange == USB_1208FS::VoltageRange::V_2_5) {
        return BP_2_50V;
    } else if (voltageRange == USB_1208FS::VoltageRange::V_2) {
        return BP_2_00V;
    } else if (voltageRange == USB_1208FS::VoltageRange::V_1_2_5) {
        return BP_1_25V;
    } else if (voltageRange == USB_1208FS::VoltageRange::V_1) {
        return BP_1_00V;
    }
    return 0;
}


short USB_1208FS::analogRead(uint8_t pinNumber, USB_1208FS::VoltageRange voltageRange) {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
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
    if (this->m_analogInputMode == USB_1208FS::AnalogInputMode::SingleEnded) {
        if (this->m_usbDeviceHandle) {
            return usbAIn_USB1208FS(this->m_usbDeviceHandle, pinNumber, SE_10_00V);
        } else {
            return 0;
        }
    } else {
        if (this->m_usbDeviceHandle) {
            return usbAIn_USB1208FS(this->m_usbDeviceHandle, pinNumber, this->voltageRangeToDifferentialGain(voltageRange));
        } else {
            return 0;
        }
    }

}

float USB_1208FS::voltageRead(uint8_t pinNumber, USB_1208FS::VoltageRange voltageRange) {
    short analogReading{this->analogRead(pinNumber, voltageRange)};
    if (this->m_analogInputMode == USB_1208FS::AnalogInputMode::SingleEnded) {
        return volts_SE(analogReading);
    } else {
        return volts_FS(this->voltageRangeToDifferentialGain(voltageRange), analogReading);
    }

}

USB_1208FS &USB_1208FS::setAnalogInputMode(USB_1208FS::AnalogInputMode analogInputMode) {
    this->m_analogInputMode = analogInputMode;
    return *this;
}

USB_1208FS::AnalogInputMode USB_1208FS::analogInputMode() const {
    return this->m_analogInputMode;
}

USB_1208FS &USB_1208FS::analogWrite(uint8_t pinNumber, uint16_t state) {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    if (pinNumber > (NUMBER_OF_ANALOG_OUTPUT_PINS - 1)) {
        throw std::runtime_error("USB_1208FS::analogWrite(uint8_t, uint16_t): analogWrite pin number exceeds maximum pin number (" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(NUMBER_OF_ANALOG_OUTPUT_PINS - 1));
    }
    this->m_analogOutputTracker.find(pinNumber)->second = state;
    auto result = usbAOut_USB1208FS(this->m_usbDeviceHandle, pinNumber, state);
    if (result < 0) {
        throw std::runtime_error("USB1208FS::analogWrite: Failed to write port");
    }
    return *this;
}

std::string USB_1208FS::serialNumber() const {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    if (!this->m_serialNumber.empty()) {
        return this->m_serialNumber;
    }
    unsigned char tempSerialNumber[SERIAL_NUMBER_BUFFER_1208FS];
    memset(tempSerialNumber, '\0', SERIAL_NUMBER_BUFFER_1208FS);
    getUsbSerialNumber(this->m_usbDeviceHandle, tempSerialNumber);

    char tempSerialNumberChar[SERIAL_NUMBER_BUFFER_1208FS];
    memset(tempSerialNumberChar, '\0', SERIAL_NUMBER_BUFFER_1208FS);

    for (int i = 0; i < SERIAL_NUMBER_BUFFER_1208FS; i++) {
        tempSerialNumberChar[i] = static_cast<char>(tempSerialNumber[i]);
    }
    this->m_serialNumber = std::string{tempSerialNumberChar};
    return this->m_serialNumber;
}

float USB_1208FS::analogToVoltage(short analogReading, AnalogInputMode inputMode, VoltageRange voltageRange) {
    if (inputMode == AnalogInputMode::SingleEnded) {
        return volts_SE(analogReading);
    } else {
        return volts_FS(analogReading, voltageRangeToDifferentialGain(voltageRange));
    }
}

USB_1208FS &USB_1208FS::resetDevice() {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    if (this->m_usbDeviceHandle) {
        usbReset_USB1208FS(this->m_usbDeviceHandle);
    }
    return *this;
}

USB_1208FS &USB_1208FS::resetCounter() {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    if (this->m_usbDeviceHandle) {
        usbInitCounter_USB1208FS(this->m_usbDeviceHandle);
    }
    return *this;
}

uint32_t USB_1208FS::readCounter() {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    if (this->m_usbDeviceHandle) {
        return usbReadCounter_USB1208FS(this->m_usbDeviceHandle);
    } else {
        return 0;
    }
}

USB_IO_Base &USB_1208FS::reinitialize() {
    std::lock_guard<std::recursive_mutex> ioLock{this->m_ioMutex};
    this->deinitialize();
    this->initialize();
    return *this;
}

USB_IO_Base &USB_1208FS::initialize() {
    if (this->m_usbDeviceHandle != nullptr) {
        this->deinitialize();
    }


    this->m_usbDeviceHandle = usb_device_find_USB_MCC(USB1208FS_PID, nullptr);
    if (!this->m_usbDeviceHandle) {
        throw std::runtime_error("USB_1208FS::USB_1208FS(): USB1208FS device not found");
    }
    auto result = init_USB1208FS(this->m_usbDeviceHandle);
    if (result < 0) {
        throw std::runtime_error("USB1208FS::initialize: Failed to initialize");
    }

    for (const auto &it : this->m_digitalPortMap) {
        this->setDigitalPortDirection(it.first, it.second);
    }

    this->resetCounter();

    return *this;
}

USB_IO_Base &USB_1208FS::deinitialize() {
    if (this->m_usbDeviceHandle != nullptr) {
        libusb_clear_halt(this->m_usbDeviceHandle, LIBUSB_ENDPOINT_IN | 1);
        libusb_clear_halt(this->m_usbDeviceHandle, LIBUSB_ENDPOINT_OUT | 2);
        libusb_clear_halt(this->m_usbDeviceHandle, LIBUSB_ENDPOINT_IN | 3);
        libusb_clear_halt(this->m_usbDeviceHandle, LIBUSB_ENDPOINT_IN | 4);
        libusb_clear_halt(this->m_usbDeviceHandle, LIBUSB_ENDPOINT_IN | 5);
        for (uint8_t i = 0; i < 4; i++) {
            libusb_release_interface(this->m_usbDeviceHandle, i);
        }
        libusb_close(this->m_usbDeviceHandle);
    }
    this->m_usbDeviceHandle = nullptr;
    this->m_serialNumber.clear();
    return *this;
}

USB_1208FS::~USB_1208FS() {
    this->deinitialize();
}



} //namespace MeasurementComputingCpp
