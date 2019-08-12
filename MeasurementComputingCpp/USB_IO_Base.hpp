#ifndef MEASUREMENTCOMPUTINGCPP_USB_IO_BASE_HPP
#define MEASUREMENTCOMPUTINGCPP_USB_IO_BASE_HPP

#include <string>
#include <mutex>

namespace MeasurementComputingCpp {

class USB_IO_Base
{
public:
#if defined(_WIN32)
    explicit inline USB_IO_Base(const std::string &name, unsigned int boardNumber) :
            m_name{name},
            m_boardNumber{boardNumber} {}
        static std::string getBoardName(unsigned int boardNumber);
        static std::string getErrorString(int errorNumber);
        std::string getSerialNumber() const;
        inline unsigned int boardNumber() const { return this->m_boardNumber; }
#else
    explicit inline USB_IO_Base(const std::string &name) :
        m_name{name} {}

#endif //defined(_WIN32)
    virtual ~USB_IO_Base() = default;
    inline std::string name() const { return this->m_name; }
#if defined(_WIN32)

#endif //defined(_WIN32)
    virtual USB_IO_Base &reinitialize() = 0;

protected:
        virtual USB_IO_Base &initialize() = 0;
        virtual USB_IO_Base &deinitialize() = 0;
        mutable std::recursive_mutex m_ioMutex;

private:
    std::string m_name;

#if defined(_WIN32)
        unsigned int m_boardNumber;
	mutable std::string m_serialNumber;
#endif //defined(_WIN32)
};

} //namespace MeasurementComputingCpp

#endif //MEASUREMENTCOMPUTINGCPP_USB_IO_BASE_HPP
