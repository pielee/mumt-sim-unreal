/****************************************************************************
 *
 * Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DAMAGES ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE.
 *
 * PX4-Autopilot v1.17.0, commit
 * d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 ****************************************************************************/
// MUMT modification: deterministic monotonic clock injection for host tests.
#pragma once
#include <cstdint>
namespace MumtPx4 {
using hrt_abstime=uint64_t;
class Px4MonotonicClock {
public: void setMicroseconds(uint64_t us){now_us_=us;} uint64_t nowMicroseconds()const{return now_us_;}
private:uint64_t now_us_{};
};
}
