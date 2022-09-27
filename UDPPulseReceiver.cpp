#include "UDPPulseReceiver.h"

#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h> // for close()

#include <algorithm>
#include <utility>
#include <iostream>
#include <string.h>
#include <cstddef>

UDPPulseReceiver::UDPPulseReceiver(std::string localIp, int localPort)
    : _localIp	(std::move(localIp))
    , _localPort(localPort)
{

}

UDPPulseReceiver::~UDPPulseReceiver()
{
    // If no one explicitly called stop before, we should at least do it.
    stop();
}

void UDPPulseReceiver::start()
{
    if (!_setupPort()) {
        return;
    }
}

bool UDPPulseReceiver::_setupPort(void)
{
    _fdSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (_fdSocket < 0) {
        std::cout << "socket error" << strerror(errno) << "\n";
        return false;
    }

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
//    inet_pton(AF_INET, _localIp.c_str(), &(addr.sin_addr));
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(_localPort);

    if (bind(_fdSocket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        std::cout << "bind error: " << strerror(errno) << "\n";
        return false;
    }

    return true;
}

void UDPPulseReceiver::stop()
{
    // This should interrupt a recv/recvfrom call.
    shutdown(_fdSocket, SHUT_RDWR);

    // But on Mac, closing is also needed to stop blocking recv/recvfrom.
    close(_fdSocket);
}

void UDPPulseReceiver::receive()
{
    // Enough for MTU 1500 bytes.
    char buffer[2048];

    struct sockaddr_in src_addr = {};
    socklen_t src_addr_len = sizeof(src_addr);
    auto recv_len = recvfrom(
        _fdSocket,
        buffer,
        sizeof(buffer),
        0,
        reinterpret_cast<struct sockaddr*>(&src_addr),
        &src_addr_len);

    if (recv_len < 0) {
        // This happens on destruction when close(_fdSocket) is called,
        // therefore be quiet.
        std::cout << "recvfrom error: " << strerror(errno);
        return;
    }

    int pulseCount = 0;
    while (recv_len--) {
    	std::cout << std::dec << "Pulse " << static_cast<unsigned int>(buffer[pulseCount++]) << "\n";
    }
}
