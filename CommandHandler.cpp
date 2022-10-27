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

    _mavlinkPassthrough.intercept_incoming_messages_async(std::bind(&CommandHandler::_handleDebugFloatArray, this, _1));
}

void CommandHandler::_sendCommandAck(CommandID commandId, uint32_t result)
{
    mavlink_message_t           message;
    mavlink_debug_float_array_t outgoingDebugFloatArray;

    memset(&outgoingDebugFloatArray, 0, sizeof(outgoingDebugFloatArray));

    outgoingDebugFloatArray.array_id                                                = static_cast<float>(CommandID::CommandIDAck);
    outgoingDebugFloatArray.data[static_cast<uint32_t>(AckIndex::AckIndexCommand)]  = static_cast<float>(commandId);
    outgoingDebugFloatArray.data[static_cast<uint32_t>(AckIndex::AckIndexResult)]   = result;

    std::cerr << "_sendCommandAck" << static_cast<uint32_t>(commandId) << " " << result << std::endl;

    mavlink_msg_debug_float_array_encode(
        _mavlinkPassthrough.get_our_sysid(),
        _mavlinkPassthrough.get_our_compid(),
        &message,
        &outgoingDebugFloatArray);
    _mavlinkPassthrough.send_message(message);        
}

void CommandHandler::_handleTagCommand(const mavlink_debug_float_array_t& debugFloatArray)
{
    _tagInfo.tagId                  = static_cast<uint32_t>(debugFloatArray.data[static_cast<uint32_t>(TagIndex::TagIndexID)]);
    _tagInfo.frequency              = static_cast<uint32_t>(debugFloatArray.data[static_cast<uint32_t>(TagIndex::TagIndexFrequency)]);
    _tagInfo.pulseDuration          = static_cast<uint32_t>(debugFloatArray.data[static_cast<uint32_t>(TagIndex::TagIndexDurationMSecs)]);
    _tagInfo.intraPulse1            = static_cast<uint32_t>(debugFloatArray.data[static_cast<uint32_t>(TagIndex::TagIndexIntraPulse1MSecs)]);
    _tagInfo.intraPulse2            = static_cast<uint32_t>(debugFloatArray.data[static_cast<uint32_t>(TagIndex::TagIndexIntraPulse2MSecs)]);
    _tagInfo.intraPulseUncertainty  = static_cast<uint32_t>(debugFloatArray.data[static_cast<uint32_t>(TagIndex::TagIndexIntraPulseUncertainty)]);
    _tagInfo.intraPulseUncertainty  = static_cast<uint32_t>(debugFloatArray.data[static_cast<uint32_t>(TagIndex::TagIndexIntraPulseJitter)]);
    _tagInfo.maxPulse               = debugFloatArray.data[static_cast<uint32_t>(TagIndex::TagIndexMaxPulse)];

    std::cout << "handleTagCommand: tagId:freq" << _tagInfo.tagId << " " << _tagInfo.frequency << std::endl;

    uint32_t commandResult = 1;

    if (_tagInfo.tagId == 0) {
        std::cout << "handleTagCommand: invalid tag id of 0\n";
        commandResult  = 0;
    } else {
        _pulseSimulator.startPulses(_tagInfo);
    }

    _sendCommandAck(CommandID::CommandIDTag, commandResult);
}

void CommandHandler::_handleStartDetection(const mavlink_debug_float_array_t& debugFloatArray)
{
    uint32_t commandResult = 1;

    if (_tagInfo.tagId) {
        std::cout << "handleStartDetection: Detection started for freq " << _tagInfo.frequency << std::endl; 
    } else {
        std::cout << "handleStartDetection: Detection started with no tag send" << std::endl;
        commandResult  = 0;
    }

    _sendCommandAck(CommandID::CommandIDStart, commandResult);

    _pulseSimulator.startPulses(_tagInfo);
}

void CommandHandler::_handleStopDetection(void)
{
    std::cout << "_handleStopDetection: Detection stopped\n"; 
    _sendCommandAck(CommandID::CommandIDStop, 1);

    _pulseSimulator.stopPulses();
}

bool CommandHandler::_handleDebugFloatArray(mavlink_message_t& message)
{
    if (message.msgid == MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY) {
        mavlink_debug_float_array_t debugFloatArray;

        mavlink_msg_debug_float_array_decode(&message, &debugFloatArray);
        switch (static_cast<CommandID>(debugFloatArray.array_id)) {
        case CommandID::CommandIDTag:
            _handleTagCommand(debugFloatArray);
            break;
        case CommandID::CommandIDStart:
            _handleStartDetection(debugFloatArray);
            break;
        case CommandID::CommandIDStop:
            _handleStopDetection();
            break;
        }
    }

    return true;
}
