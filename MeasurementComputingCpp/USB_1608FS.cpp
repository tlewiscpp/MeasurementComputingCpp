#include <cstring>

#include <mcc-libusb/pmd.h>
#include <mcc-libusb/usb-1608FS.h>

#include "USB_1608FS.hpp"

#define CHECK_BIT(value,bit) ((value) & (1 << (bit)))
#define SET_BIT(value,bit) ((value) |= (1 << (bit)))
#define CLEAR_BIT(value,bit) ((value) &= ~(1 << (bit)))
#define TOGGLE_BIT(value,bit) ((value) ^= (1 << (bit)))

namespace MeasurementComputingCpp {

USB_1608FS::USB_1608FS() :
    m_usbDeviceHandle{nullptr},
    m_digitalPortMap{},
    m_serialNumber{""},
    m_analogInputCalibrationTable{nullptr}
{
    
    int initResult{libusb_init(nullptr)};
    if (initResult != 0) {
        throw std::runtime_error("USB_1608FS::USB_1608FS(): libusb_init failed with return code " + toStdString(initResult));
    }

    this->m_usbDeviceHandle = usb_device_find_USB_MCC(USB1608FS_PID, nullptr);
    if (!this->m_usbDeviceHandle) {
        throw std::runtime_error("USB_1608FS::USB_1608FS(): USB1608FS device not found");
    }
    this->m_analogInputCalibrationTable = new Calibration_AIN[4];
    for (int i = 0; i < 8; i++) {
        this->m_analogInputCalibrationTable[i] = new Calibration_AIN;
    }
    usbBuildCalTable_USB1608FS(this->m_usbDeviceHandle, this->m_analogInputCalibrationTable);

    usbDConfigPort_USB1608FS(this->m_usbDeviceHandle, DIO_DIR_IN);

    this->m_digitalPortMap.emplace(USB_1608FS::DigitalPinNumber::Pin0, USB_1608FS::PortDirection::DigitalInput);
    this->m_digitalPortMap.emplace(USB_1608FS::DigitalPinNumber::Pin1, USB_1608FS::PortDirection::DigitalInput);
    this->m_digitalPortMap.emplace(USB_1608FS::DigitalPinNumber::Pin2, USB_1608FS::PortDirection::DigitalInput);
    this->m_digitalPortMap.emplace(USB_1608FS::DigitalPinNumber::Pin3, USB_1608FS::PortDirection::DigitalInput);
    this->m_digitalPortMap.emplace(USB_1608FS::DigitalPinNumber::Pin4, USB_1608FS::PortDirection::DigitalInput);
    this->m_digitalPortMap.emplace(USB_1608FS::DigitalPinNumber::Pin5, USB_1608FS::PortDirection::DigitalInput);
    this->m_digitalPortMap.emplace(USB_1608FS::DigitalPinNumber::Pin6, USB_1608FS::PortDirection::DigitalInput);
    this->m_digitalPortMap.emplace(USB_1608FS::DigitalPinNumber::Pin7, USB_1608FS::PortDirection::DigitalInput);

    this->resetCounter();
}



USB_1608FS::USB_1608FS(USB_1608FS &&rhs) noexcept :
    m_usbDeviceHandle{rhs.m_usbDeviceHandle},
    m_digitalPortMap{std::move(rhs.m_digitalPortMap)},
    m_serialNumber{std::move(rhs.m_serialNumber)},
    m_analogInputCalibrationTable{}
{
    for (uint8_t i = 0; i < NGAINS_USB1608FS; i++) {
        for (uint8_t j = 0; j < NCHAN_USB1608FS; j++) {
            this->m_analogInputCalibrationTable[i][j] = rhs.m_analogInputCalibrationTable[i][j];
        }
    }
}

USB_1608FS& USB_1608FS::operator=(USB_1608FS &&rhs) noexcept
{
    this->m_usbDeviceHandle = rhs.m_usbDeviceHandle;
    this->m_digitalPortMap = std::move(rhs.m_digitalPortMap);
    this->m_serialNumber = std::move(rhs.m_serialNumber);

    for (uint8_t i = 0; i < NGAINS_USB1608FS; i++) {
        for (uint8_t j = 0; j < NCHAN_USB1608FS; j++) {
            this->m_analogInputCalibrationTable[i][j] = rhs.m_analogInputCalibrationTable[i][j];
        }
    }
    return *this;
}

void USB_1608FS::setDigitalPortDirection(PortDirection portDirection)
{
    usbDConfigPort_USB1608FS(this->m_usbDeviceHandle, this->digitalPortDirectionToUInt8(portDirection));
    if (portDirection == PortDirection::DigitalOutput) {
        usbDOut_USB1608FS(this->m_usbDeviceHandle, 0x00); //0b00000000
    }
    for (auto &it : this->m_digitalPortMap) {
        it.second = portDirection;
    }
}

void USB_1608FS::setDigitalPortDirection(DigitalPinNumber pinNumber, PortDirection direction)
{
    usbDConfigBit_USB1608FS(this->m_usbDeviceHandle, static_cast<uint8_t>(pinNumber), digitalPortDirectionToUInt8(direction));
    if (direction == PortDirection::DigitalOutput) {
        usbDOutBit_USB1608FS(this->m_usbDeviceHandle, static_cast<uint8_t>(pinNumber), 0x00); //0b00000000
    }
    this->m_digitalPortMap.find(pinNumber)->second = direction;
}

uint8_t USB_1608FS::digitalPortDirectionToUInt8(PortDirection direction)
{
    if (direction == USB_1608FS::PortDirection::DigitalInput) {
        return DIO_DIR_IN;
    } else if (direction == USB_1608FS::PortDirection::DigitalOutput) {
        return DIO_DIR_OUT;
    } else {
        throw std::runtime_error("USB_1608FS::digitalPortDirectionToUInt8(PortDirection): Invalid PortDirection");
    }
}

USB_1608FS::PortDirection USB_1608FS::digitalPortDirection(USB_1608FS::DigitalPinNumber pinNumber) const
{
    return this->m_digitalPortMap.find(pinNumber)->second;
}

bool USB_1608FS::digitalWrite(DigitalPinNumber pinNumber, bool state)
{
    if (this->m_digitalPortMap.find(pinNumber)->second != USB_1608FS::PortDirection::DigitalOutput) {
        return false;
    }
    usbDOutBit_USB1608FS(this->m_usbDeviceHandle, static_cast<uint8_t>(pinNumber), static_cast<uint8_t>(state));
    return true;
}

bool USB_1608FS::digitalRead(DigitalPinNumber pinNumber)
{
    if (this->m_digitalPortMap.find(pinNumber)->second != USB_1608FS::PortDirection::DigitalInput) {
        return false;
    }
    uint8_t bitValue{0};
    usbDInBit_USB1608FS(this->m_usbDeviceHandle, static_cast<uint8_t>(pinNumber), &bitValue);
    return static_cast<bool>(bitValue);
}

USB_1608FS::~USB_1608FS()
{
    for (int i = 0; i < 8; i++) {
        delete[] this->m_analogCalibrationTable[i];
    }
    delete[] this->m_analogCalibrationTable[i];
    usbAInStop_USB1608FS(this->m_usbDeviceHandle);
    usbReset_USB1608FS(this->m_usbDeviceHandle);
    libusb_clear_halt(this->m_usbDeviceHandle, LIBUSB_ENDPOINT_IN | 2);
    libusb_clear_halt(this->m_usbDeviceHandle, LIBUSB_ENDPOINT_IN | 3);
    libusb_clear_halt(this->m_usbDeviceHandle, LIBUSB_ENDPOINT_IN | 4);
    libusb_clear_halt(this->m_usbDeviceHandle, LIBUSB_ENDPOINT_IN | 5);
    libusb_clear_halt(this->m_usbDeviceHandle, LIBUSB_ENDPOINT_IN | 6);
    for (uint8_t i = 0; i < 7; i++) {
        libusb_release_interface(this->m_usbDeviceHandle, i);
    }
    libusb_close(this->m_usbDeviceHandle);
}

uint8_t USB_1608FS::voltageRangeToAnalogGain(USB_1608FS::VoltageRange voltageRange)
{
    if (voltageRange == USB_1608FS::VoltageRange::V_10) {
        return BP_10_00V;
    } else if (voltageRange == USB_1608FS::VoltageRange::V_5) {
        return BP_5_00V;
    } else if (voltageRange == USB_1608FS::VoltageRange::V_2_5) {
        return BP_2_50V;
    } else if (voltageRange == USB_1608FS::VoltageRange::V_2) {
        return BP_2_00V;
    } else if (voltageRange == USB_1608FS::VoltageRange::V_1_2_5) {
        return BP_1_25V;
    } else if (voltageRange == USB_1608FS::VoltageRange::V_1) {
        return BP_1_00V;
    } else if (voltageRange == USB_1608FS::VoltageRange::V_0_625) {
        return BP_0_625V;
    } else if (voltageRange == USB_1608FS::VoltageRange::V_0_3125) {
        return BP_0_3125V;
    }
    return 0;
}


short USB_1608FS::analogRead(AnalogPinNumber pinNumber, USB_1608FS::VoltageRange voltageRange)
{
    return usbAIn_USB1608FS(this->m_usbDeviceHandle, static_cast<uint8_t>(pinNumber), this->voltageRangeToAnalogGain(voltageRange), this->m_analogInputCalibrationTable);
}

float USB_1608FS::voltageRead(AnalogPinNumber pinNumber, USB_1608FS::VoltageRange voltageRange)
{
    return analogToVoltage(this->analogRead(pinNumber, voltageRange), voltageRange);
}

std::string USB_1608FS::serialNumber() const
{
    if (!this->m_serialNumber.empty()) {
        return this->m_serialNumber;
    }
    unsigned char tempSerialNumber[SERIAL_NUMBER_BUFFER_1608FS];
    memset(tempSerialNumber, '\0', SERIAL_NUMBER_BUFFER_1608FS);
    getUsbSerialNumber(this->m_usbDeviceHandle, tempSerialNumber);

    char tempSerialNumberChar[SERIAL_NUMBER_BUFFER_1608FS];
    memset(tempSerialNumberChar, '\0', SERIAL_NUMBER_BUFFER_1608FS);

    for (int i = 0; i < SERIAL_NUMBER_BUFFER_1608FS; i++) {
        tempSerialNumberChar[i] = static_cast<char>(tempSerialNumber[i]);
    }
    this->m_serialNumber = std::string{tempSerialNumberChar};
    return this->m_serialNumber;
}

float USB_1608FS::analogToVoltage(short analogReading, VoltageRange voltageRange)
{
    return volts_USB1608FS(voltageRangeToAnalogGain(voltageRange), analogReading);
}

void USB_1608FS::resetDevice()
{
    usbReset_USB1608FS(this->m_usbDeviceHandle);
}

void USB_1608FS::resetCounter() {
    usbInitCounter_USB1608FS(this->m_usbDeviceHandle);
}

uint32_t USB_1608FS::readCounter() {
    return usbReadCounter_USB1608FS(this->m_usbDeviceHandle);
}


} //namespace MeasurementComputingCpp
