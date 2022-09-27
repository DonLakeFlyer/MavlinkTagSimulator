#pragma once

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include "PulseSimulator.h"
#include "TagInfo.h"

using namespace mavsdk;

class CommandHandler {
public:
    CommandHandler(System& system, MavlinkPassthrough& mavlinkPassthrough, PulseSimulator& PulseSimulator);

private:
    void _sendCommandAck        (uint32_t commandId, uint32_t result);
    void _handleTagCommand      (const mavlink_debug_float_array_t& debugFloatArray);
    void _handleStartDetection  (const mavlink_debug_float_array_t& debugFloatArray);
    void _handleStopDetection   (void);
    bool _handleDebugFloatArray (mavlink_message_t& message);

private:
    System&             _system;
    MavlinkPassthrough& _mavlinkPassthrough;
    PulseSimulator&     _pulseSimulator;
    TagInfo             _tagInfo;
};
