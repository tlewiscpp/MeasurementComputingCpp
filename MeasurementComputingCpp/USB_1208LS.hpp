#ifndef MEASUREMENTCOMPUTINGCPP_1208LS_HPP
#define MEASUREMENTCOMPUTINGCPP_1208LS_HPP

#include <map>
#include <sstream>
#include "USB_IO_Base.hpp"

#if !defined(_WIN32)
struct hid_device_;
#endif //!defined(_WIN32)

namespace MeasurementComputingCpp {

class USB_1208LS : public USB_IO_Base
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
	explicit USB_1208LS(unsigned int boardNumber);
#else
	USB_1208LS();
#endif //defined(_WIN32)
    USB_1208LS(const USB_1208LS &rhs) = delete;
    USB_1208LS(USB_1208LS &&rhs) noexcept;
    ~USB_1208LS() override;

    USB_1208LS &operator=(const USB_1208LS &rhs) = delete;
    USB_1208LS &operator=(USB_1208LS &&rhs) noexcept;

    USB_1208LS &setDigitalPortDirection(DigitalPortID portID, PortDirection direction);
    PortDirection digitalPortDirection(DigitalPortID portID) const;

    USB_1208LS &setAnalogInputMode(AnalogInputMode analogInputMode);
    AnalogInputMode analogInputMode() const;

    bool digitalWrite(uint8_t pinNumber, bool state);
    bool digitalRead(uint8_t pinNumber);
    bool digitalWrite(DigitalPortID portID, uint8_t pinNumber, bool state);
    bool digitalRead(DigitalPortID portID, uint8_t pinNumber);
    short analogRead(uint8_t pinNumber, VoltageRange voltageRange = VoltageRange::V_10);
    float voltageRead(uint8_t pinNumber, VoltageRange voltageRange = VoltageRange::V_10);
    USB_1208LS &analogWrite(uint8_t pinNumber, uint16_t state);

    USB_1208LS &resetCounter();
    uint32_t readCounter();
    USB_1208LS &resetDevice();

    std::string serialNumber() const;
    float analogToVoltage(short analogReading, AnalogInputMode inputMode = AnalogInputMode::SingleEnded, VoltageRange voltageRange = VoltageRange::V_10);

protected:
    USB_IO_Base &initialize() override;
    USB_IO_Base &deinitialize() override;


private:
#if defined(_WIN32)
	unsigned int m_boardNumber;
#else
    hid_device_ *m_hidDevice;
    mutable std::string m_serialNumber;
#endif //defined(_WIN32)
    std::map<DigitalPortID, PortDirection> m_digitalPortMap;
    std::map<DigitalPortID, uint8_t> m_digitalOutputTracker;
    std::map<uint8_t, uint16_t> m_analogOutputTracker;
    AnalogInputMode m_analogInputMode;

    static uint8_t digitalPortIDToUInt8(DigitalPortID portID);
    static uint8_t digitalPortDirectionToUInt8(PortDirection direction);
    static uint8_t voltageRangeToDifferentialGain(VoltageRange voltageRange);
    static bool getDigitalPortIDAndPinNumber(uint8_t pinNumber, DigitalPortID *outPortID, uint8_t *outAdjustedPinNumber);

    template <typename T> static inline std::string toStdString(const T &t) { return dynamic_cast<std::ostringstream &>(std::ostringstream{} << t).str(); }

};

} //namespace MeasurementComputingCpp

#endif //MEASUREMENTCOMPUTINGIO_1208LS_HPP
