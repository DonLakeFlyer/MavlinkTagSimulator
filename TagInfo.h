#pragma once

#include <mavsdk/mavsdk.h>

typedef struct {
	uint32_t    tagId                 { 0 };
    uint32_t    frequency;
    uint32_t    pulseDuration;
    uint32_t    intraPulse1;
    uint32_t    intraPulse2;
    uint32_t    intraPulseUncertainty;
    uint32_t    intraPulseJitter;
    float 		maxPulse;
 } TagInfo;