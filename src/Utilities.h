#pragma once

#include <Arduino.h>

#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <forward_list>
#include <limits>
#include <memory>
#include <utility>

using std::numeric_limits;

template <typename T, typename U>
bool CanTypeFitValue(const U value)
{
    const intmax_t botT = intmax_t(numeric_limits<T>::min());
    const intmax_t botU = intmax_t(numeric_limits<U>::min());
    const uintmax_t topT = uintmax_t(numeric_limits<T>::max());
    const uintmax_t topU = uintmax_t(numeric_limits<U>::max());
    return !((botT > botU && value < static_cast<U>(botT)) || (topT < topU && value > static_cast<U>(topT)));
}

bool ParseInt(const std::string&, int&);
