#pragma once

// RTC-to-OLED presentation only: no ESP-IDF, I2C, config, or timezone APIs.
// The RTC clock value is already the user's selected wall-clock time.
#include <cstdio>

namespace oled_clock_face {

struct Time {
    unsigned year;
    unsigned month;
    unsigned day;
    unsigned hour;
    unsigned minute;
    unsigned second;
};

struct Face {
    char clock[9]{};    // " 1:02:03" / "12:34:56" (fixed 8-character width)
    char period[3]{};   // "AM" / "PM"
    char date[24]{};   // "FRI  OCT 23 2026"
};

inline unsigned hour_12(unsigned hour)
{
    const unsigned h = hour % 12u;
    return h == 0u ? 12u : h;
}

inline unsigned weekday(unsigned year, unsigned month, unsigned day)
{
    // Sakamoto, 0 = Sunday. The caller supplies validated RTC date fields.
    static constexpr unsigned offsets[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    if (month < 1u || month > 12u) return 0u;
    if (month < 3u) --year;
    return (year + year/4u - year/100u + year/400u + offsets[month-1u] + day) % 7u;
}

inline Face format(const Time& time)
{
    static constexpr const char* kWeekdays[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
    static constexpr const char* kMonths[] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN",
                                              "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};
    Face face{};
    const unsigned hour = time.hour <= 23u ? time.hour : 0u;
    const unsigned minute = time.minute <= 59u ? time.minute : 0u;
    const unsigned second = time.second <= 59u ? time.second : 0u;
    std::snprintf(face.clock, sizeof(face.clock), "%2u:%02u:%02u", hour_12(hour), minute, second);
    std::snprintf(face.period, sizeof(face.period), "%s", hour < 12u ? "AM" : "PM");
    if (time.month >= 1u && time.month <= 12u && time.day >= 1u && time.day <= 31u) {
        std::snprintf(face.date, sizeof(face.date), "%s  %s %02u %04u",
                      kWeekdays[weekday(time.year, time.month, time.day)],
                      kMonths[time.month - 1u], time.day, time.year);
    } else {
        std::snprintf(face.date, sizeof(face.date), "RTC DATE UNAVAILABLE");
    }
    return face;
}

} // namespace oled_clock_face
