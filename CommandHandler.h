#pragma once

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include "uavrt_interfaces/include/uavrt_interfaces/qgc_enum_class_definitions.hpp"
using namespace uavrt_interfaces;

#include "PulseSimulator.h"
#include "TagInfo.h"

using namespace mavsdk;

class CommandHandler {
public:
    CommandHandler(System& system, MavlinkPassthrough& mavlinkPassthrough, PulseSimulator& PulseSimulator);

private:
    void _sendCommandAck        (CommandID commandId, uint32_t result);
    void _handleTagCommand      (const mavlink_debug_float_array_t& debugFloatArray);
    void _handleStartDetection  (const mavlink_debug_float_array_t& debugFloatArray);
    void _handleStopDetection   (void);
    void _handleTunnel          (const mavlink_message_t& message);
    void _sendTunnelAck         (mavlink_message_t& commandMessage, uint32_t command, uint32_t result);

private:
    System&             _system;
    MavlinkPassthrough& _mavlinkPassthrough;
    PulseSimulator&     _pulseSimulator;
    TagInfo             _tagInfo;
};
