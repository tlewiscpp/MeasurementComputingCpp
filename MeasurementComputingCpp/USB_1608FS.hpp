#ifndef MEASUREMENTCOMPUTINGCPP_1608FS_H
#define MEASUREMENTCOMPUTINGCPP_1608FS_H

#include <map>
#include <sstream>
#include "USB_IO_Base.hpp"

#if !defined(_WIN32)
struct Calibration_AIN_t;
struct libusb_device_handle;
#endif //!defined(_WIN32)

namespace MeasurementComputingCpp {

#define BITS_PER_PORT_1608FS 8
#define MAXIMUM_ANALOG_OUTPUT_VALUE_1608FS 65535
#define SERIAL_NUMBER_BUFFER_1608FS 255

class USB_1608FS : public USB_IO_Base
{
public:
    enum class DigitalPinNumber {
        Pin0 = 0,
        Pin1 = 1,
        Pin2 = 2,
        Pin3 = 3,
        Pin4 = 4,
        Pin5 = 5,
        Pin6 = 6,
        Pin7 = 7
    };
    enum class AnalogPinNumber {
        Pin0 = 0,
        Pin1 = 1,
        Pin2 = 2,
        Pin3 = 3,
        Pin4 = 4,
        Pin5 = 5,
        Pin6 = 6,
        Pin7 = 7
    };
    enum class PortDirection {
        DigitalInput,
        DigitalOutput
    };
    enum class VoltageRange {
        V_10,
        V_5,
        V_2_5,
        V_2,
        V_1_2_5,
        V_1,
        V_0_625,
        V_0_3125
    };

    USB_1608FS();
    USB_1608FS(const USB_1608FS &rhs) = delete;
    USB_1608FS(USB_1608FS &&rhs) noexcept;
    ~USB_1608FS() override;

    USB_1608FS &operator=(const USB_1608FS &rhs) = delete;
    USB_1608FS &operator=(USB_1608FS &&rhs) noexcept;

    void setDigitalPortDirection(DigitalPinNumber pinNumber, PortDirection direction);
    void setDigitalPortDirection(PortDirection portDirection);
    PortDirection digitalPortDirection(DigitalPinNumber pinNumber) const;

    bool digitalWrite(DigitalPinNumber pinNumber, bool state);
    bool digitalRead(DigitalPinNumber pinNumber);

    short analogRead(AnalogPinNumber pinNumber, VoltageRange voltageRange = VoltageRange::V_10);
    float voltageRead(AnalogPinNumber pinNumber, VoltageRange voltageRange = VoltageRange::V_10);

    void resetCounter();
    uint32_t readCounter();
    void resetDevice();

    std::string serialNumber() const;
    static float analogToVoltage(short analogReading, VoltageRange voltageRange = VoltageRange::V_10);

private:
#if !defined(_WIN32)
        libusb_device_handle *m_usbDeviceHandle;
        Calibration_AIN_t **m_analogInputCalibrationTable;
#endif //!defined(_WIN32)
        std::map<DigitalPinNumber, PortDirection> m_digitalPortMap;
        mutable std::string m_serialNumber;

    static uint8_t digitalPortDirectionToUInt8(PortDirection direction);
    static uint8_t voltageRangeToAnalogGain(VoltageRange voltageRange);

    template <typename T> static inline std::string toStdString(const T &t) { return dynamic_cast<std::ostringstream &>(std::ostringstream{} << t).str(); }

};

} //namespace MeasurementComputingCpp

#endif //MEASUREMENTCOMPUTINGIO_1608FS_H
