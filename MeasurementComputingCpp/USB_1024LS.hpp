#ifndef MEASUREMENTCOMPUTINGCPP_1024LS_HPP
#define MEASUREMENTCOMPUTINGCPP_1024LS_HPP

#include <map>
#include <sstream>
#include "USB_IO_Base.hpp"
#include <utility>

#if !defined(_WIN32)
struct hid_device_;
#endif //!defined(_WIN32)

namespace MeasurementComputingCpp {

#define BITS_PER_PORT_1024LS 8
#define SERIAL_NUMBER_BUFFER_1024LS 255

class USB_1024LS : public USB_IO_Base {
public:
    enum class DigitalPortID {
        PortA,
        PortB,
        PortCLow,
        PortCHigh
    };

    enum class PortDirection {
        DigitalInput,
        DigitalOutput
    };

#if defined(_WIN32)
        explicit USB_1024LS(unsigned int boardNumber);
#else

        USB_1024LS();

#endif //defined(_WIN32)

        USB_1024LS(const USB_1024LS &rhs) = delete;

        USB_1024LS(USB_1024LS &&rhs) noexcept;

        ~USB_1024LS() override;

        USB_1024LS &operator=(const USB_1024LS &rhs) = delete;

        USB_1024LS &operator=(USB_1024LS &&rhs) noexcept;

        void setDigitalPortDirection(DigitalPortID portID, PortDirection direction);

        PortDirection digitalPortDirection(DigitalPortID portID) const;

        void resetCounter();

        uint32_t readCounter();

        void resetDevice();

        bool digitalWrite(DigitalPortID portID, uint8_t pinNumber, bool state);

        bool digitalRead(DigitalPortID portID, uint8_t pinNumber);

        bool digitalRead(uint8_t pinNumber);

        bool digitalWrite(uint8_t pinNumber, bool state);

        std::string serialNumber() const;

    private:
#if defined(_WIN32)
        unsigned int m_boardNumber;
#else
        hid_device_ *m_hidDevice;
        mutable std::string m_serialNumber;
#endif //!defined(_WIN32)
        std::map<DigitalPortID, PortDirection> m_digitalPortMap;

        static uint8_t digitalPortIDToUInt8(DigitalPortID portID);

        static uint8_t digitalPortDirectionToUInt8(PortDirection direction);

        static std::pair<DigitalPortID, uint8_t> rawPinNumberToPinDefinition(uint8_t pinNumber);

        template<typename T>
        static inline std::string toStdString(const T &t) { return dynamic_cast<std::ostringstream &>(std::ostringstream{} << t).str(); }
    };
} //namespace MeasurementComputingCpp

#endif //MEASUREMENTCOMPUTINGCPP_1024LS_HPP
