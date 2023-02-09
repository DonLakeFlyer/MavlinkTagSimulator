#include "PulseSimulator.h"

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

        mavlink_message_t   message;
        mavlink_tunnel_t    tunnel;
        PulseInfo_t         pulseInfo;

        memset(&tunnel,     0, sizeof(tunnel));
        memset(&pulseInfo,  0, sizeof(pulseInfo));

        pulseInfo.header.command        = COMMAND_ID_PULSE;
        pulseInfo.tag_id                = _tagInfo.id;
        pulseInfo.frequency_hz          = _tagInfo.frequency_hz;
        pulseInfo.start_time_seconds    = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        pulseInfo.snr                   = pulse;
        pulseInfo.confirmed_status      = confirmed;

        memcpy(tunnel.payload, &pulseInfo, sizeof(pulseInfo));

        tunnel.target_system    = _mavlinkPassthrough.get_target_sysid();
        tunnel.target_component = _mavlinkPassthrough.get_target_compid();
        tunnel.payload_type     = MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN;
        tunnel.payload_length   = sizeof(pulseInfo);

        std::cout << "simulatePulse: timeSecs:snr:confirmed " << pulseInfo.start_time_seconds << pulse << confirmed << std::endl;

        confirmed = !confirmed;

        mavlink_msg_tunnel_encode(
            _mavlinkPassthrough.get_our_sysid(),
            _mavlinkPassthrough.get_our_compid(),
            &message,
            &tunnel);
        _mavlinkPassthrough.send_message(message);        

        return _tagInfo.intra_pulse1_msecs;
    } else {
        return 1000;
    }
}

void PulseSimulator::startPulses(TagInfo_t& tagInfo)
{
    _sendPulses = true;
    _tagInfo    = tagInfo;
}

void PulseSimulator::stopPulses(void)
{
    _sendPulses = false;
    _tagInfo.id = 0;
}

bool PulseSimulator::readyRunning(void)
{
    return _sendPulses && _vehiclePositionKnown && _vehicleEulerAngleKnown && _tagInfo.id != 0;
}
