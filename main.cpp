#include "PulseSimulator.h"
#include "CommandHandler.h"
#include "UDPPulseReceiver.h"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <memory>
#include <thread>

using namespace mavsdk;

static void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " [simulate-pulse|udp-pulse]\n"
              << "Defaults to simulate-pulse\n";
}

int main(int argc, char* argv[])
{
    bool udpPulse;

    if (argc == 2) {
        if (strcmp(argv[1], "udp-pulse") == 0) {
            udpPulse = true;
        } else if (strcmp(argv[1], "simulate-pulse") == 0) {
            udpPulse = false;
        } else {
            usage(argv[0]);
            return 1;                    
        }
    } else if (argc == 1) {
        udpPulse = false;
    } else {
        usage(argv[0]);
        return 1;        
    }

    Mavsdk mavsdk;
    mavsdk.set_configuration(Mavsdk::Configuration(1, MAV_COMP_ID_ONBOARD_COMPUTER, true));

    const ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14540");
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    // We need to wait for both the Autopilot and QGC Systems to become availaable

    bool foundAutopilot = false;
    bool foundQGC       = false;

    std::shared_ptr<System> autopilotSystem;
    std::shared_ptr<System> qgcSystem;

    std::cout << "Waiting to discover Autopilot and QGC" << std::endl;
    while (!foundAutopilot || !foundQGC) {
        std::vector< std::shared_ptr< System > > systems = mavsdk.systems();
        for (size_t i=0; i<systems.size(); i++) {
            std::shared_ptr< System > system = systems[i];
            std::vector< uint8_t > compIds = system->component_ids();
            for (size_t i=0; i < compIds.size(); i++) {
                auto compId = compIds[i];
                if (!foundAutopilot && compId == MAV_COMP_ID_AUTOPILOT1) {
                    std::cout << "Discovered Autopilot" << std::endl;
                    autopilotSystem = system;
                    foundAutopilot  = true;
                } else if (!foundQGC && compId == MAV_COMP_ID_MISSIONPLANNER && system->get_system_id() == 255) {
                    std::cout << "Discovered QGC" << std::endl;
                    qgcSystem = system;
                    foundQGC = true;
                } 
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    auto udpPulseReceiver   = UDPPulseReceiver("127.0.0.1", 30001);
    auto mavlinkPassthrough = MavlinkPassthrough{qgcSystem};
    auto pulseSimulator     = PulseSimulator{*qgcSystem, mavlinkPassthrough};
    auto commandHandler     = CommandHandler{*qgcSystem, mavlinkPassthrough, pulseSimulator};

    std::cout << "targets " << int(mavlinkPassthrough.get_target_sysid()) << " " << int(mavlinkPassthrough.get_target_compid()) << std::endl;

    if (udpPulse) {
        udpPulseReceiver.start();
    }

    if (udpPulse) {
        std::cout << "UDP Pulses\n";
    } else {
        std::cout << "Simulating Pulses\n";
    }

    while (true) {
        if (udpPulse) {
            udpPulseReceiver.receive();
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(pulseSimulator.simulatePulse()));
        }
    }

    return 0;
}
