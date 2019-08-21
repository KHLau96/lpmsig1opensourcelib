#include "util.h"
#include <iomanip>


void logd(std::string tag, const char* str, ...)
{
    va_list a_list;
    va_start(a_list, str);
    if (!tag.empty())
        printf("[%s] ", tag.c_str());
    vprintf(str, a_list);
    va_end(a_list);
}

const std::string currentDateTime(const char* format)
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), format, &tstruct);

    return buf;
}


const int currentDateTimeInt()
{
    time_t     now = time(0);
    struct tm  tstruct;
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    int ret = (2000+tstruct.tm_year-100) * 10000 + (tstruct.tm_mon+1) * 100 + tstruct.tm_mday;

    return ret;
}
