#include "PulseSimulator.h"
#include "CommandDefs.h"

#include <iostream>

PulseSimulator::PulseSimulator(System& system, MavlinkPassthrough& mavlinkPassthrough)
    : _system             (system)
    , _mavlinkPassthrough (mavlinkPassthrough)
    , _telemetry          (system)
{
    using namespace std::placeholders;

    _telemetry.subscribe_position(std::bind(&PulseSimulator::_positionCallback, this, _1));
    _telemetry.subscribe_attitude_euler(std::bind(&PulseSimulator::_attitudeEulerCallback, this, _1));
}


void PulseSimulator::_positionCallback(Telemetry::Position position) 
{
    _vehiclePositionKnown   = true;
    _vehiclePosition        = position;
}

void PulseSimulator::_attitudeEulerCallback(Telemetry::EulerAngle eulerAngle) 
{
    _vehicleEulerAngleKnown = true;
    _vehicleEulerAngle      = eulerAngle;
}

uint32_t PulseSimulator::simulatePulse(void)
{
    if (readyRunning()) {
        double heading = _vehicleEulerAngle.yaw_deg;

        heading -= _strongestPulseAngle;
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

        memset(&debugFloatArray, 0, sizeof(debugFloatArray));
        debugFloatArray.array_id                    = COMMAND_ID_PULSE;
        debugFloatArray.data[PULSE_IDX_STRENGTH]    = pulse;

        mavlink_msg_debug_float_array_encode(
            _mavlinkPassthrough.get_our_sysid(),
            _mavlinkPassthrough.get_our_compid(),
            &message,
            &debugFloatArray);
        _mavlinkPassthrough.send_message(message);        

        return _tagInfo.intraPulse1;
    } else {
        return 1000;
    }
}

void PulseSimulator::startPulses(TagInfo& tagInfo)
{
    _sendPulses = true;
    _tagInfo    = tagInfo;
}

void PulseSimulator::stopPulses(void)
{
    _sendPulses     = false;
    _tagInfo.tagId  = 0;
}

bool PulseSimulator::readyRunning(void)
{
    return _sendPulses && _vehiclePositionKnown && _vehicleEulerAngleKnown && _tagInfo.tagId != 0;
}

void PulseSimulator::sendStatus(void)
{
    if (readyRunning()) {
        std::cout << "Sending COMMAND_ID_DETECTION_STATUS\n";

        mavlink_message_t           message;
        mavlink_debug_float_array_t debugFloatArray;

        debugFloatArray.array_id                            = COMMAND_ID_DETECTION_STATUS;
        debugFloatArray.data[DETECTION_STATUS_IDX_STATUS]   = 0;

        mavlink_msg_debug_float_array_encode(
            _mavlinkPassthrough.get_our_sysid(),
            _mavlinkPassthrough.get_our_compid(),
            &message,
            &debugFloatArray);
        _mavlinkPassthrough.send_message(message);         
    }
}
