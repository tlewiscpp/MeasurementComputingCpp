#if defined(_MSC_VER)
#    define "windows-config.h"
#elif defined(__ANDROID__)
#    define "android-config.h"
#else
#    define "linux-config.h"
#endif //defined(_MSC_VER)
