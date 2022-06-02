#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <memory>
#include <thread>

using namespace mavsdk;

// DEBUG_FLOAT_ARRAY
//  time_usec - send index (used to detect lost telemetry)
//  array_id - command id

#define COMMAND_ID_ACK                          1   // Ack response to command
#define COMMAND_ID_TAG                          2   // Tag info
#define COMMAND_ID_START_DETECTION              3   // Start pulse detection
#define COMMAND_ID_STOP_DETECTION               4   // Stop pulse detection
#define COMMAND_ID_PULSE                        5   // Detected pulse value

#define ACK_IDX_COMMAND                         0   // Command being acked
#define ACK_IDX_RESULT                          1   // Command result - 1 success, 0 failure

#define PULSE_IDX_DETECTION_STATUS              0   // Detection status (uint 32)
#define PULSE_IDX_STRENGTH                      1   // Pulse strength [0-100] (float)
#define PULSE_IDX_GROUP_INDEX                   2   // Group index 0/1/2 (uint 32)

#define PULSE_DETECTION_STATUS_SUPER_THRESHOLD  1
#define PULSE_DETECTION_STATUS_CONFIRMED        2

#define TAG_IDX_ID                              0   // Tag id (uint 32)
#define TAG_IDX_FREQUENCY                       1   // Frequency - 6 digits shifted by three decimals, NNNNNN means NNN.NNN000 Mhz (uint 32)
#define TAG_IDX_DURATION_MSECS                  2   // Pulse duration
#define TAG_IDX_INTRA_PULSE1_MSECS              3   // Intra-pulse duration 1
#define TAG_IDX_INTRA_PULSE2_MSECS              4   // Intra-pulse duration 2
#define TAG_IDX_INTRA_PULSE_UNCERTAINTY         5   // Intra-pulse uncertainty
#define TAG_IDX_INTRA_PULSE_JITTER              6   // Intra-pulse jitter
#define TAG_IDX_MAX_PULSE                       7   // Max pulse value

#define START_DETECTION_IDX_TAG_ID              0   // Tag to start detection on

// Simulator values
uint32_t    simulatorTagId                 = 0;
uint32_t    simulatorFrequency;
uint32_t    simulatorPulseDuration;
uint32_t    simulatorIntraPulse1;
uint32_t    simulatorIntraPulse2;
uint32_t    simulatorIntraPulseUncertainty;
uint32_t    simulatorIntraPulseJitter;
float       simulatorMaxPulse;


static void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

class CommandHandler {
public:
    CommandHandler(System& system) :
        _system             (system),
        _mavlinkPassthrough (system),
        _telemetry          (system)
    {
        using namespace std::placeholders;

        _mavlinkPassthrough.intercept_incoming_messages_async(std::bind(&CommandHandler::_handleDebugFloatArray, this, _1));
        _telemetry.subscribe_position(std::bind(&CommandHandler::_positionCallback, this, _1));
        _telemetry.subscribe_attitude_euler(std::bind(&CommandHandler::_attitudeEulerCallback, this, _1));
    }

private:
    void _sendCommandAck(uint32_t commandId, uint32_t result)
    {
        mavlink_message_t           message;
        mavlink_debug_float_array_t outgoingDebugFloatArray;

        memset(&outgoingDebugFloatArray, 0, sizeof(outgoingDebugFloatArray));

        outgoingDebugFloatArray.array_id              = COMMAND_ID_ACK;
        outgoingDebugFloatArray.data[ACK_IDX_COMMAND] = commandId;
        outgoingDebugFloatArray.data[ACK_IDX_RESULT]  = result;

        mavlink_msg_debug_float_array_encode(
            _mavlinkPassthrough.get_our_sysid(),
            _mavlinkPassthrough.get_our_compid(),
            &message,
            &outgoingDebugFloatArray);
        _mavlinkPassthrough.send_message(message);        
    }

    void _handleTagCommand(const mavlink_debug_float_array_t& debugFloatArray)
    {
        _simulatorTagId                 = static_cast<uint32_t>(debugFloatArray.data[TAG_IDX_ID]);
        _simulatorFrequency             = static_cast<uint32_t>(debugFloatArray.data[TAG_IDX_FREQUENCY]);
        _simulatorPulseDuration         = static_cast<uint32_t>(debugFloatArray.data[TAG_IDX_DURATION_MSECS]);
        _simulatorIntraPulse1           = static_cast<uint32_t>(debugFloatArray.data[TAG_IDX_INTRA_PULSE1_MSECS]);
        _simulatorIntraPulse2           = static_cast<uint32_t>(debugFloatArray.data[TAG_IDX_INTRA_PULSE2_MSECS]);
        _simulatorIntraPulseUncertainty = static_cast<uint32_t>(debugFloatArray.data[TAG_IDX_INTRA_PULSE_UNCERTAINTY]);
        _simulatorIntraPulseJitter      = static_cast<uint32_t>(debugFloatArray.data[TAG_IDX_INTRA_PULSE_JITTER]);
        _simulatorMaxPulse              = debugFloatArray.data[TAG_IDX_MAX_PULSE];

        std::cout << "handleTagCommand: tagId:freq" << _simulatorTagId << " " << _simulatorFrequency << "\n";

        uint32_t commandResult = 1;

        if (_simulatorTagId == 0) {
            std::cout << "handleTagCommand: invalid tag id of 0\n";
            commandResult  = 0;
        }

        _sendCommandAck(COMMAND_ID_TAG, commandResult);
    }

    void _handleStartDetection(const mavlink_debug_float_array_t& debugFloatArray)
    {
        uint32_t requestedTagId = static_cast<uint32_t>(debugFloatArray.data[START_DETECTION_IDX_TAG_ID]);

        uint32_t commandResult = 1;

        if (requestedTagId == _simulatorTagId) {
            _sendPulses = true;
            std::cout << "handleStartDetection: Detection started for freq " << _simulatorFrequency << "\n"; 
        } else {
            std::cout << "handleStartDetection: requested start tag id != known tag id - requested:known " << 
                requestedTagId << " " <<
                _simulatorTagId << "\n";
            commandResult  = 0;
        }

        _sendCommandAck(COMMAND_ID_START_DETECTION, commandResult);
    }

    void _handleStopDetection(void)
    {
        std::cout << "_handleStopDetection: Detection stopped\n"; 
        _sendPulses = false;
        _sendCommandAck(COMMAND_ID_STOP_DETECTION, 1);
    }

    bool _handleDebugFloatArray(mavlink_message_t& message)
    {
        if (message.msgid == MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY) {
            mavlink_debug_float_array_t debugFloatArray;

            mavlink_msg_debug_float_array_decode(&message, &debugFloatArray);
            switch (debugFloatArray.array_id) {
            case COMMAND_ID_TAG:
                _handleTagCommand(debugFloatArray);
                break;
            case COMMAND_ID_START_DETECTION:
                _handleStartDetection(debugFloatArray);
                break;
            case COMMAND_ID_STOP_DETECTION:
                _handleStopDetection();
                break;
            }
        }

        return true;
    }

    void _positionCallback(Telemetry::Position position) 
    {
        _vehiclePositionKnown   = true;
        _vehiclePosition        = position;
    }

    void _attitudeEulerCallback(Telemetry::EulerAngle eulerAngle) 
    {
        _vehicleEulerAngleKnown = true;
        _vehicleEulerAngle      = eulerAngle;
    }

public:
    uint32_t simulatePulse(void)
    {
        if (_sendPulses && _vehiclePositionKnown && _vehicleEulerAngleKnown) {
            double heading = _vehicleEulerAngle.yaw_deg;

            // Strongest pulse is at 90 degrees
            heading -= 90;
            if (heading < 0) {
                heading += 360;
            }

            double pulseRatio;
            if (heading <= 180) {
                pulseRatio = (180.0 - heading) / 180.0;
            } else {
                heading -= 180;
                pulseRatio = heading / 180.0;
            }
            double pulse = 100.0 * pulseRatio;

            double currentAltRel = _vehiclePosition.relative_altitude_m;
            double maxAlt = 121.92; // 400 feet max alt
            double altRatio = currentAltRel / maxAlt;
            pulse *= altRatio;

            std::cout << "simulatePulse: " << heading << " " << pulseRatio << " " << pulse << "\n";

            mavlink_message_t           message;
            mavlink_debug_float_array_t debugFloatArray;

            debugFloatArray.array_id                    = COMMAND_ID_PULSE;
            debugFloatArray.data[PULSE_IDX_STRENGTH]    = pulse;

            mavlink_msg_debug_float_array_encode(
                _mavlinkPassthrough.get_our_sysid(),
                _mavlinkPassthrough.get_our_compid(),
                &message,
                &debugFloatArray);
            _mavlinkPassthrough.send_message(message);        

            return _simulatorIntraPulse1;
        } else {
            return 1000;
        }
    }

private:
    System&                 _system;
    MavlinkPassthrough      _mavlinkPassthrough;
    Telemetry               _telemetry;
    bool                    _sendPulses                     { false };
    uint32_t                _simulatorTagId                 { 0 };
    uint32_t                _simulatorFrequency;
    uint32_t                _simulatorPulseDuration;
    uint32_t                _simulatorIntraPulse1;
    uint32_t                _simulatorIntraPulse2;
    uint32_t                _simulatorIntraPulseUncertainty;
    uint32_t                _simulatorIntraPulseJitter;
    float                   _simulatorMaxPulse;
    bool                    _vehiclePositionKnown           { false };
    bool                    _vehicleEulerAngleKnown         { false };
    Telemetry::Position     _vehiclePosition;
    Telemetry::EulerAngle   _vehicleEulerAngle;
};


int main(int argc, char* argv[])
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    const std::string connection_url = argv[1];

    Mavsdk mavsdk;

    // We start with sysid 1 but adapt to the one of the autopilot once
    // we have discoverd it.
    uint8_t our_sysid = 1;
    mavsdk.set_configuration(Mavsdk::Configuration{our_sysid, MAV_COMP_ID_USER1, false});

    const ConnectionResult connection_result = mavsdk.add_any_connection(connection_url);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 3 seconds max, surely.
    if (fut.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) {
        std::cerr << "No autopilot found, exiting.\n";
        return 1;
    }

    // Get discovered system now.
    auto system = fut.get();

    // Update system ID if required.
    if (system->get_system_id() != our_sysid) {
        our_sysid = system->get_system_id();
        mavsdk.set_configuration(Mavsdk::Configuration{our_sysid, MAV_COMP_ID_USER1, false});
    }

    auto commandHandler     = CommandHandler{*system};

    std::cout << "Waiting for commands\n";
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(commandHandler.simulatePulse()));
    }

    return 0;
}
