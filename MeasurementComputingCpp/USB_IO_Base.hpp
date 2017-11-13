#ifndef MEASUREMENTCOMPUTINGCPP_USB_IO_BASE_H
#define MEASUREMENTCOMPUTINGCPP_USB_IO_BASE_H

namespace MeasurementComputingCpp {

class USB_IO_Base
{
public:
    explicit inline USB_IO_Base(const std::string &name) :
        m_name{name} {}
    inline std::string name() const { return this->m_name; }

private:
    std::string m_name;
};

} //namespace MeasurementComputingCpp

#endif //MEASUREMENTCOMPUTINGCPP_USB_IO_BASE_H
