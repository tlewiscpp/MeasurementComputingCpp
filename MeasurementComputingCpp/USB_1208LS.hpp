#ifndef MEASUREMENTCOMPUTINGCPP_1208LS_H
#define MEASUREMENTCOMPUTINGCPP_1208LS_H

#include <map>
#include <sstream>

struct hid_device_;

namespace MeasurementComputingCpp {

class USB_1208LS
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


    USB_1208LS();
    USB_1208LS(const USB_1208LS &rhs) = delete;
    USB_1208LS(USB_1208LS &&rhs) noexcept;
    ~USB_1208LS();

    USB_1208LS &operator=(const USB_1208LS &rhs) = delete;
    USB_1208LS &operator=(USB_1208LS &&rhs) noexcept;

    void setDigitalPortDirection(DigitalPortID portID, PortDirection direction);
    PortDirection digitalPortDirection(DigitalPortID portID) const;

    void setAnalogInputMode(AnalogInputMode analogInputMode);
    AnalogInputMode analogInputMode() const;

    bool digitalWrite(DigitalPortID portID, uint8_t pinNumber, bool state);
    bool digitalRead(DigitalPortID portID, uint8_t pinNumber);
    short analogRead(uint8_t pinNumber, VoltageRange voltageRange = VoltageRange::V_10);
    float voltageRead(uint8_t pinNumber, VoltageRange voltageRange = VoltageRange::V_10);
    void analogWrite(uint8_t pinNumber, uint16_t state);

    void resetCounter();
    uint32_t readCounter();
    void resetDevice();

    std::string serialNumber() const;
    static float analogToVoltage(short analogReading, AnalogInputMode inputMode = AnalogInputMode::SingleEnded, VoltageRange voltageRange = VoltageRange::V_10);

private:
    hid_device_ *m_hidDevice;
    std::map<DigitalPortID, PortDirection> m_digitalPortMap;
    AnalogInputMode m_analogInputMode;
    mutable std::string m_serialNumber;

    static uint8_t digitalPortIDToUInt8(DigitalPortID portID);
    static uint8_t digitalPortDirectionToUInt8(PortDirection direction);
    static uint8_t voltageRangeToDifferentialGain(VoltageRange voltageRange);

    template <typename T> static inline std::string toStdString(const T &t) { return dynamic_cast<std::ostringstream &>(std::ostringstream{} << t).str(); }
    void doSetDigitalPortDirection(DigitalPortID portID, PortDirection direction);

};

} //namespace MeasurementComputingCpp

#endif //MEASUREMENTCOMPUTINGIO_1208LS_H
