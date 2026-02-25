#include "mavlink_hooks.hpp"
#include "tlv_cmd_router.hpp"
#include "tlv_pack.hpp"
#include "tlv_parser.hpp"
#include "udp_server.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
#include <thread>

int main(int argc, char** argv)
{
    const uint16_t port = (argc >= 2) ? static_cast<uint16_t>(std::stoi(argv[1])) : 14550;

    UdpServer server;
    if (!server.Open(port)) {
        std::cerr << "Failed to open UDP port " << port << "\n";
        return 1;
    }
    std::cout << "CM5 UDP TLV server listening on 0.0.0.0:" << port << "\n";

    DummyHooks hooks;
    TlvCmdRouter router(hooks);
    router.RegisterDefaults();

    TlvParser parser;
    UdpPeer lastPeer{};
    uint8_t rxBuffer[2048]{};

    while (true) {
        UdpPeer peer{};
        const int recvLen = server.Recv(rxBuffer, sizeof(rxBuffer), &peer);
        if (recvLen > 0) {
            lastPeer = peer;

            parser.Push(rxBuffer, static_cast<size_t>(recvLen));
            while (auto frame = parser.TryPop()) {
                const RouteResult routeResult = router.Handle(*frame);

                std::vector<uint8_t> ackFrame = MakeAckFrame(
                    frame->seq,
                    frame->tMs,
                    frame->cmd,
                    frame->seq,
                    routeResult.status);

                server.SendTo(lastPeer, ackFrame.data(), ackFrame.size());

                if (!routeResult.msg.empty()) {
                    std::cout << "[CMD " << static_cast<int>(frame->cmd) << " seq=" << frame->seq
                              << "] " << routeResult.msg << "\n";
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    return 0;
}
