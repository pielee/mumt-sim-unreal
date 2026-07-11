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
#pragma once
#include "Px4MathAdapter.h"
namespace MumtPx4 {
template<typename T> class AlphaFilter {
public:
 AlphaFilter()=default;
 explicit AlphaFilter(float dt,float tc){setParameters(dt,tc);}
 explicit AlphaFilter(float tc):time_constant_(tc){}
 void setParameters(float dt,float tc){const float d=tc+dt;if(d>FLT_EPSILON)setAlpha(dt/d);time_constant_=tc;}
 void setAlpha(float a){alpha_=a;}
 void reset(const T&s){state_=s;}
 const T& update(const T&s){state_=state_+alpha_*(s-state_);return state_;}
 T update(const T&s,float dt){setParameters(dt,time_constant_);return update(s);}
 const T& getState()const{return state_;}
private: float time_constant_{},alpha_{};T state_{};
};
}
