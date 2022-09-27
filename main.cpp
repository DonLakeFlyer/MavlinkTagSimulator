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
    mavsdk.set_configuration(Mavsdk::Configuration{our_sysid, MAV_COMP_ID_USER1, false});

    const ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14540");

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
    if (fut.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
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

    auto udpPulseReceiver   = UDPPulseReceiver("127.0.0.1", 30001);
    auto mavlinkPassthrough = MavlinkPassthrough{*system};
    auto pulseSimulator     = PulseSimulator{*system, mavlinkPassthrough};
    auto commandHandler     = CommandHandler{*system, mavlinkPassthrough, pulseSimulator};

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
