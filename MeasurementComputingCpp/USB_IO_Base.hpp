#ifndef MEASUREMENTCOMPUTINGCPP_USB_IO_BASE_H
#define MEASUREMENTCOMPUTINGCPP_USB_IO_BASE_H

#include <string>

namespace MeasurementComputingCpp {

class USB_IO_Base
{
public:
    explicit inline USB_IO_Base(const std::string &name) :
        m_name{name} {}
    virtual ~USB_IO_Base() = default;
    inline std::string name() const { return this->m_name; }
#if defined(_WIN32)
    static std::string getBoardName(unsigned int boardNumber);
        static std::string getErrorString(unsigned int errorNumber);
#endif //defined(_WIN32)

private:
    std::string m_name;
};

} //namespace MeasurementComputingCpp

#endif //MEASUREMENTCOMPUTINGCPP_USB_IO_BASE_H
