#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <memory>
#include <thread>

#include "CommandHandler.h"

using namespace mavsdk;

CommandHandler::CommandHandler(System& system, MavlinkPassthrough& mavlinkPassthrough, PulseSimulator& pulseSimulator)
    : _system               (system)
    , _mavlinkPassthrough   (mavlinkPassthrough)
    , _pulseSimulator       (pulseSimulator)
{
    using namespace std::placeholders;

    _mavlinkPassthrough.subscribe_message_async(MAVLINK_MSG_ID_TUNNEL, std::bind(&CommandHandler::_handleTunnelMessage, this, _1));
}

void CommandHandler::_sendTunnelAck(uint32_t command, uint32_t result)
{
    mavlink_message_t   message;
    mavlink_tunnel_t    tunnel;
    AckInfo_t           ackInfo;

    memset(&tunnel, 0, sizeof(tunnel));

    ackInfo.header.command  = COMMAND_ID_ACK;
    ackInfo.command         = command;
    ackInfo.result          = result;

    tunnel.target_system    = _mavlinkPassthrough.get_target_sysid();
    tunnel.target_component = _mavlinkPassthrough.get_target_compid();
    tunnel.payload_type     = MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN;
    tunnel.payload_length   = sizeof(ackInfo);

    memcpy(tunnel.payload, &ackInfo, sizeof(ackInfo));

    std::cerr << "_sendTunnelAck command:result " << command << " " << result << std::endl;

    mavlink_msg_tunnel_encode(
        _mavlinkPassthrough.get_our_sysid(),
        _mavlinkPassthrough.get_our_compid(),
        &message,
        &tunnel);
    _mavlinkPassthrough.send_message(message);        
}

bool CommandHandler::_handleStartTags(void)
{
    std::cout << "COMMAND_ID_START_TAGS"<< std::endl;

    if (_detectionRunning) {
        std::cout << "CommandHandler::_handleStartTags Error: Detection is current running" << std::endl;
        return false;
    } else if (_receivingTags) {
        std::cout << "CommandHandler::_handleStartTags Error: Duplicate start received" << std::endl;
        return false;
    } else {
        // We are in the correct state to start receiving tags
        _receivingTags  = true;
        _haveTags       = false;
        return true;
    }
}

bool CommandHandler::_handleEndTags(void)
{
    std::cout << "COMMAND_ID_END_TAGS"<< std::endl;

    if (!_receivingTags) {
        std::cout << "CommandHandler::_handleEndTags Error: No start tags received" << std::endl;
        return false;
    } else {
        // We are in the correct state to end receiving tags
        _receivingTags = false;
        return true;
    }
}

bool CommandHandler::_handleTagInfo(const mavlink_tunnel_t& tunnel)
{
    memcpy(&_tagInfo, tunnel.payload, sizeof(_tagInfo));

    std::cout << "COMMAND_ID_TAG:"                      << std::endl;
    std::cout << "\tid: "                               << _tagInfo.id << std::endl;
    std::cout << "\tfrequency_hz: "                     << _tagInfo.frequency_hz << std::endl;
    std::cout << "\tpulse_width_msecs: "                << _tagInfo.pulse_width_msecs << std::endl;
    std::cout << "\tintra_pulse1_msecs: "               << _tagInfo.intra_pulse1_msecs << std::endl;
    std::cout << "\tintra_pulse_uncertainty_msecs: "    << _tagInfo.intra_pulse_uncertainty_msecs << std::endl;
    std::cout << "\tintra_pulse_jitter_msecs: "         << _tagInfo.intra_pulse_jitter_msecs << std::endl;
    std::cout << "\tk: "                                << _tagInfo.k << std::endl;
    std::cout << "\tfalse_alarm_probability: "           << _tagInfo.false_alarm_probability << std::endl;
    std::cout << std::endl;

    _haveTags = true;

    return true;
}

bool CommandHandler::_handleStartDetection(void)
{
    std::cout << "COMMAND_ID_START_DETECTION" << std::endl;

    uint32_t commandResult = 1;

    if (_tagInfo.id) {
        std::cout << "handleStartDetection: Detection started for freq " << _tagInfo.frequency_hz << std::endl; 
    } else {
        std::cout << "handleStartDetection: Detection started with no tag send" << std::endl;
        commandResult  = 0;
    }

    _pulseSimulator.startPulses(_tagInfo);

    return true;
}

bool CommandHandler::_handleStopDetection(void)
{
    std::cout << "COMMAND_ID_STOP_DETECTION" << std::endl;

    std::cout << "_handleStopDetection: Detection stopped\n"; 

    _pulseSimulator.stopPulses();

    return true;
}

void CommandHandler::_handleTunnelMessage(const mavlink_message_t& message)
{
    std::cout << "Got TUNNEL mavlink message" << std::endl;
    
    mavlink_message_t   outgoingMessage;
    mavlink_tunnel_t    tunnel;
    bool                success = false;

    mavlink_msg_tunnel_decode(&message, &tunnel);

    HeaderInfo_t header;
    memcpy(&header, tunnel.payload, sizeof(header));

    switch (header.command) {
    case COMMAND_ID_START_TAGS:
        success = _handleStartTags();
        break;
    case COMMAND_ID_TAG:
        success = _handleTagInfo(tunnel);
        break;
    case COMMAND_ID_END_TAGS:
        success = _handleEndTags();
        break;
    case COMMAND_ID_START_DETECTION:
        success = _handleStartDetection();
        break;
    case COMMAND_ID_STOP_DETECTION:
        success = _handleStopDetection();
        break;
    }

    _sendTunnelAck(header.command, success ? COMMAND_RESULT_SUCCESS : COMMAND_RESULT_FAILURE);
}
