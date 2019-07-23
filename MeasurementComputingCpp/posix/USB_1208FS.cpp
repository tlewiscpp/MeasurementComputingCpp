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

    this->m_usbDeviceHandle = usb_device_find_USB_MCC(USB1208FS_PID, nullptr);
    if (!this->m_usbDeviceHandle) {
        throw std::runtime_error("USB_1208FS::USB_1208FS(): USB1208FS device not found");
    }
    init_USB1208FS(this->m_usbDeviceHandle);

    usbDConfigPort_USB1208FS(this->m_usbDeviceHandle, DIO_PORTA, DIO_DIR_IN);
    usbDConfigPort_USB1208FS(this->m_usbDeviceHandle, DIO_PORTB, DIO_DIR_IN);

    this->m_digitalPortMap.emplace(USB_1208FS::DigitalPortID::PortA, USB_1208FS::PortDirection::DigitalInput);
    this->m_digitalPortMap.emplace(USB_1208FS::DigitalPortID::PortB, USB_1208FS::PortDirection::DigitalInput);

    this->m_digitalOutputTracker.emplace(USB_1208FS::DigitalPortID::PortA, 0);
    this->m_digitalOutputTracker.emplace(USB_1208FS::DigitalPortID::PortB, 0);

    this->resetCounter();
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

USB_1208FS& USB_1208FS::operator=(USB_1208FS &&rhs) noexcept {
    this->m_usbDeviceHandle = rhs.m_usbDeviceHandle;
    this->m_serialNumber = std::move(rhs.m_serialNumber);
    this->m_digitalPortMap = std::move(rhs.m_digitalPortMap);
    this->m_digitalOutputTracker = std::move(rhs.m_digitalOutputTracker);
    this->m_analogInputMode = rhs.m_analogInputMode;
    return *this;
}

void USB_1208FS::setDigitalPortDirection(DigitalPortID portID, PortDirection direction) {
    auto currentPortDirection = this->m_digitalPortMap.find(portID)->second;
    if (currentPortDirection == direction) {
        return;
    }
    usbDConfigPort_USB1208FS(this->m_usbDeviceHandle, digitalPortIDToUInt8(portID), digitalPortDirectionToUInt8(direction));
    if (direction == PortDirection::DigitalOutput) {
        usbDOut_USB1208FS(this->m_usbDeviceHandle, digitalPortIDToUInt8(portID), 0x00); //0b00000000
    }
    this->m_digitalPortMap.find(portID)->second = direction;
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
    usbDOut_USB1208FS(this->m_usbDeviceHandle, digitalPortIDToUInt8(portID), *targetPortCurrentState);
    return true;
}

bool USB_1208FS::digitalRead(DigitalPortID portID, uint8_t pinNumber) {
    int upperPinNumber{BITS_PER_PORT_1208FS};
    if (pinNumber >= upperPinNumber) {
        throw std::runtime_error("USB_1208FS::digitalWrite(DigitalPortID, uint8_t, bool): pinNumber for ports A and B must be between 0 and " + toStdString(upperPinNumber) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(upperPinNumber));
    }
    if (this->m_digitalPortMap.find(portID)->second != USB_1208FS::PortDirection::DigitalInput) {
        return false;
    }
    uint8_t allValues{0};
    usbDIn_USB1208FS(this->m_usbDeviceHandle, digitalPortIDToUInt8(portID), &allValues);
    return static_cast<bool>(CHECK_BIT(allValues, pinNumber));
}

bool USB_1208FS::digitalRead(uint8_t pinNumber) {
    DigitalPortID portID{};
    uint8_t adjustedPinNumber{};
    auto result = getDigitalPortIDAndPinNumber(pinNumber, &portID, &adjustedPinNumber);
    if (!result) {
        throw std::runtime_error("USB_1024LS::digitalRead(bool): pinNumber must be between 0 and " + toStdString(PORT_B_MAX_PIN_NUMBER) + "(" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(PORT_B_MAX_PIN_NUMBER));
    }
    return this->digitalRead(portID, adjustedPinNumber);
}

bool USB_1208FS::digitalWrite(uint8_t pinNumber, bool state) {
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

USB_1208FS::~USB_1208FS() {
    libusb_clear_halt(this->m_usbDeviceHandle, LIBUSB_ENDPOINT_IN | 1);
    libusb_clear_halt(this->m_usbDeviceHandle, LIBUSB_ENDPOINT_OUT| 2);
    libusb_clear_halt(this->m_usbDeviceHandle, LIBUSB_ENDPOINT_IN | 3);
    libusb_clear_halt(this->m_usbDeviceHandle, LIBUSB_ENDPOINT_IN | 4);
    libusb_clear_halt(this->m_usbDeviceHandle, LIBUSB_ENDPOINT_IN | 5);
    for (uint8_t i = 0; i < 4; i++) {
        libusb_release_interface(this->m_usbDeviceHandle, i);
    }
    libusb_close(this->m_usbDeviceHandle);
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
        return usbAIn_USB1208FS(this->m_usbDeviceHandle, pinNumber, SE_10_00V);
    } else {
        return usbAIn_USB1208FS(this->m_usbDeviceHandle, pinNumber, this->voltageRangeToDifferentialGain(voltageRange));
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

void USB_1208FS::setAnalogInputMode(USB_1208FS::AnalogInputMode analogInputMode) {
    this->m_analogInputMode = analogInputMode;
}

USB_1208FS::AnalogInputMode USB_1208FS::analogInputMode() const {
    return this->m_analogInputMode;
}

void USB_1208FS::analogWrite(uint8_t pinNumber, uint16_t state) {
    if (pinNumber > (NUMBER_OF_ANALOG_OUTPUT_PINS - 1)) {
        throw std::runtime_error("SB_1208FS::analogWrite(uint8_t, uint16_t): analogWrite pin number exceeds maximum pin number (" + toStdString(static_cast<int>(pinNumber)) + " > " + toStdString(NUMBER_OF_ANALOG_OUTPUT_PINS - 1));
    }
    usbAOut_USB1208FS(this->m_usbDeviceHandle, pinNumber, state);
}

std::string USB_1208FS::serialNumber() const {
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

void USB_1208FS::resetDevice() {
    usbReset_USB1208FS(this->m_usbDeviceHandle);
}

void USB_1208FS::resetCounter() {
    usbInitCounter_USB1208FS(this->m_usbDeviceHandle);
}

uint32_t USB_1208FS::readCounter() {
    return usbReadCounter_USB1208FS(this->m_usbDeviceHandle);
}


} //namespace MeasurementComputingCpp
