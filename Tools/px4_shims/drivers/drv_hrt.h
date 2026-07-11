#pragma once
#include <cstdint>
using hrt_abstime=uint64_t;
extern uint64_t px4_equivalence_time_us;
inline hrt_abstime hrt_absolute_time(){return px4_equivalence_time_us;}
namespace time_literals { constexpr uint64_t operator"" _s(unsigned long long s){return s*1000000ULL;} }
