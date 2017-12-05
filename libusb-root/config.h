#if defined(_MSC_VER)
#    include "msvc-config.h"
#elif defined(_WIN32)
#    include "mingw-config.h"
#elif defined(__ANDROID__)
#    include "android-config.h"
#else
#    include "linux-config.h"
#endif //defined(_MSC_VER)
