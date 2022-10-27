#include "PulseSimulator.h"

#include "uavrt_interfaces/include/uavrt_interfaces/qgc_enum_class_definitions.hpp"
using namespace uavrt_interfaces;

#include <iostream>
#include <chrono>

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
    static bool confirmed = true;

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

        mavlink_message_t           message;
        mavlink_debug_float_array_t debugFloatArray;

        memset(&debugFloatArray, 0, sizeof(debugFloatArray));
        debugFloatArray.time_usec                                                           = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        debugFloatArray.array_id                                                            = static_cast<uint16_t>(CommandID::CommandIDPulse);
        debugFloatArray.data[static_cast<uint32_t>(PulseIndex::PulseIndexStrengthLEGACY)]   = pulse;
        debugFloatArray.data[static_cast<uint32_t>(PulseIndex::PulseIndexSNR)]              = pulse;
        debugFloatArray.data[static_cast<uint32_t>(PulseIndex::PulseIndexConfirmedStatus)]  = confirmed;

        std::cout << "simulatePulse: timeSecs:snr:confirmed" << debugFloatArray.time_usec / 1000 << pulse << confirmed << std::endl;

        confirmed = !confirmed;

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
