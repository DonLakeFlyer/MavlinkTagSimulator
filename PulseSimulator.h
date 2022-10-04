#pragma once

#include "TagInfo.h"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;

class PulseSimulator {
public:
    PulseSimulator(System& system, MavlinkPassthrough& mavlinkPassthrough);

    void 		startPulses		(TagInfo& tagInfo);
    void 		stopPulses		(void);
    uint32_t 	simulatePulse	(void);
    bool        readyRunning    (void);
    void        sendStatus      (void);

private:
    void _positionCallback		(Telemetry::Position position);
    void _attitudeEulerCallback	(Telemetry::EulerAngle eulerAngle);

    System&                 _system;
    MavlinkPassthrough&     _mavlinkPassthrough;
    Telemetry               _telemetry;
    bool                    _sendPulses             { false };
    TagInfo					_tagInfo;
    bool                    _vehiclePositionKnown   { false };
    bool                    _vehicleEulerAngleKnown	{ false };
    Telemetry::Position     _vehiclePosition;
    Telemetry::EulerAngle   _vehicleEulerAngle;

    const double 			_strongestPulseAngle 	{ 45 };
};
