#include "Utilities.h"

#include <errno.h>

bool ParseInt(const std::string& str, int& val)
{
    char* temp;

    errno = 0;

    long long_val = strtol(str.data(), &temp, 0);

    if (temp == str.data() || *temp != '\0' || ((long_val == LONG_MIN || long_val == LONG_MAX) && errno == ERANGE) ||
        !CanTypeFitValue<int>(long_val))
    {
        val = 0;
        return false;
    }

    val = static_cast<int>(long_val);
    return true;
}