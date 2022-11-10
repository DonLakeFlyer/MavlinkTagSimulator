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

    // We start with sysid 1 but adapt to the one of the autopilot once
    // we have discoverd it.
    uint8_t our_sysid = 1;
    mavsdk.set_configuration(Mavsdk::Configuration{our_sysid, MAV_COMP_ID_ONBOARD_COMPUTER, false});

    const ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14540");

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    std::cout << "Waiting to discover Autopilot and QGC systems...\n";

    // Wait 3 seconds for the Autopilot System to show up
    auto autopilotPromise   = std::promise<std::shared_ptr<System>>{};
    auto autopilotFuture    = autopilotPromise.get_future();
    mavsdk.subscribe_on_new_system([&mavsdk, &autopilotPromise]() {
        auto system = mavsdk.systems().back();
        if (system->has_autopilot()) {
            std::cout << "Discovered Autopilot" << std::endl;
            autopilotPromise.set_value(system);
            mavsdk.subscribe_on_new_system(nullptr);            
        }
    });
    if (autopilotFuture.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
        std::cerr << "No autopilot found, exiting.\n";
        return 1;
    }

    // Since we can't do anything without the QGC connection we wait for it indefinitely
    auto qgcPromise = std::promise<std::shared_ptr<System>>{};
    auto qgcFuture  = qgcPromise.get_future();
    mavsdk.subscribe_on_new_system([&mavsdk, &qgcPromise]() {
        auto system = mavsdk.systems().back();
        std::vector< uint8_t > compIds = system->component_ids();
        if (std::find(compIds.begin(), compIds.end(), MAV_COMP_ID_MISSIONPLANNER) != compIds.end()) {
            std::cout << "Discovered QGC" << std::endl;
            qgcPromise.set_value(system);
            mavsdk.subscribe_on_new_system(nullptr);            
        }
    });
    qgcFuture.wait();

    // We have both systems ready for use now
    auto autopilotSystem    = autopilotFuture.get();
    auto qgcSystem          = qgcFuture.get();

    // Update system ID if required.
    if (autopilotSystem->get_system_id() != our_sysid) {
        our_sysid = autopilotSystem->get_system_id();
        mavsdk.set_configuration(Mavsdk::Configuration{our_sysid, MAV_COMP_ID_ONBOARD_COMPUTER, false});
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
