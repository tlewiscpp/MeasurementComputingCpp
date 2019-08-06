#ifndef MEASUREMENTCOMPUTINGCPP_1208FS_HPP
#define MEASUREMENTCOMPUTINGCPP_1208FS_HPP

#include <map>
#include <sstream>
#include "USB_IO_Base.hpp"

struct libusb_device_handle;

namespace MeasurementComputingCpp {

#define BITS_PER_PORT_1208FS 8
#define MAXIMUM_ANALOG_OUTPUT_VALUE_1208FS 65535
#define NUMBER_OF_ANALOG_OUTPUT_PINS 2
#define SERIAL_NUMBER_BUFFER_1208FS 255

class USB_1208FS : public USB_IO_Base
{
public:
    enum class DigitalPortID {
        PortA,
        PortB
    };
    enum class AnalogChannelID {
        Channel0
    };
    enum class PortDirection {
        DigitalInput,
        DigitalOutput
    };
    enum class AnalogInputMode {
        SingleEnded,
        Differential
    };
    enum class VoltageRange {
        V_20,
        V_10,
        V_5,
        V_4,
        V_2_5,
        V_2,
        V_1_2_5,
        V_1
    };

#if defined(_WIN32)
    explicit USB_1208FS(unsigned int m_boardNumber);
#else
	USB_1208FS();
#endif //defined(_WIN32)
    USB_1208FS(const USB_1208FS &rhs) = delete;
    USB_1208FS(USB_1208FS &&rhs) noexcept;
    ~USB_1208FS() override;

    USB_1208FS &operator=(const USB_1208FS &rhs) = delete;
    USB_1208FS &operator=(USB_1208FS &&rhs) noexcept;

    USB_1208FS &setDigitalPortDirection(DigitalPortID portID, PortDirection direction);
    PortDirection digitalPortDirection(DigitalPortID portID) const;

    USB_1208FS &setAnalogInputMode(AnalogInputMode analogInputMode);
    AnalogInputMode analogInputMode() const;

    bool digitalWrite(uint8_t pinNumber, bool state);
    bool digitalRead(uint8_t pinNumber);
    bool digitalWrite(DigitalPortID portID, uint8_t pinNumber, bool state);
    bool digitalRead(DigitalPortID portID, uint8_t pinNumber);
    short analogRead(uint8_t pinNumber, VoltageRange voltageRange = VoltageRange::V_10);
    float voltageRead(uint8_t pinNumber, VoltageRange voltageRange = VoltageRange::V_10);
    USB_1208FS &analogWrite(uint8_t pinNumber, uint16_t state);

    USB_1208FS &resetCounter();
    uint32_t readCounter();
    USB_1208FS &resetDevice();

    std::string serialNumber() const;
    float analogToVoltage(short analogReading, AnalogInputMode inputMode = AnalogInputMode::SingleEnded, VoltageRange voltageRange = VoltageRange::V_10);

protected:
    USB_IO_Base &initialize() override;
    USB_IO_Base &deinitialize() override;


    private:
#if defined(_WIN32)
	unsigned int m_boardNumber;
#else
    libusb_device_handle *m_usbDeviceHandle;
    mutable std::string m_serialNumber;
#endif //defined(_WIN32)
    std::map<DigitalPortID, PortDirection> m_digitalPortMap;
    std::map<DigitalPortID, uint8_t> m_digitalOutputTracker;
    std::map<uint8_t, uint16_t> m_analogOutputTracker;
    AnalogInputMode m_analogInputMode;

    static uint8_t digitalPortIDToUInt8(DigitalPortID portID);
    static uint8_t digitalPortDirectionToUInt8(PortDirection direction);
    static uint8_t voltageRangeToDifferentialGain(VoltageRange voltageRange);
    bool getDigitalPortIDAndPinNumber(uint8_t pinNumber, DigitalPortID *outPortID, uint8_t *outAdjustedPinNumber);
    template <typename T> static inline std::string toStdString(const T &t) { return dynamic_cast<std::ostringstream &>(std::ostringstream{} << t).str(); }

};

} //namespace MeasurementComputingCpp

#endif //MEASUREMENTCOMPUTINGIO_1208FS_HPP
