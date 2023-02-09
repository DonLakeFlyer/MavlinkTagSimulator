#pragma once

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include "TunnelProtocol.h"
#include "PulseSimulator.h"

using namespace mavsdk;
using namespace TunnelProtocol;

class CommandHandler {
public:
    CommandHandler(System& system, MavlinkPassthrough& mavlinkPassthrough, PulseSimulator& PulseSimulator);

private:
    void _handleTunnelMessage   (const mavlink_message_t& message);
    bool _handleStartTags       (void);
    bool _handleEndTags         (void);
    bool _handleTagInfo         (const mavlink_tunnel_t& tunnel);
    bool _handleStartDetection  (void);
    bool _handleStopDetection   (void);
    void _sendTunnelAck         (uint32_t command, uint32_t result);

private:
    System&             _system;
    MavlinkPassthrough& _mavlinkPassthrough;
    PulseSimulator&     _pulseSimulator;
    TagInfo_t           _tagInfo;
    bool                _haveTags           = false;
    bool                _receivingTags      = false;
    bool                _detectionRunning   = false;
};
