#include "udp_server.hpp"
#include "tlv_parser.hpp"
#include "tlv_cmd_router.hpp"
#include "tlv_pack.hpp"

#include "mavlink_hooks.hpp"

#include <chrono>
#include <iostream>
#include <thread>

static uint32_t nowMs32()
{
    using namespace std::chrono;
    auto ms = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    return (uint32_t)(ms & 0xFFFFFFFFu);
}

int main(int argc, char** argv)
{
    const uint16_t port = (argc >= 2) ? (uint16_t)std::stoi(argv[1]) : 14550;

    UdpServer srv;
    if (!srv.open(port))
    {
        std::cerr << "Failed to open UDP port " << port << "\n";
        return 1;
    }
    std::cout << "CM5 UDP TLV server listening on 0.0.0.0:" << port << "\n";

    DummyHooks hooks; // TODO: replace with Px4Hooks using your MavlinkSerial
    TlvCmdRouter router(hooks);
    router.registerDefaults();

    TlvParser parser;

    UdpPeer lastPeer{};
    uint8_t rx[2048];

    while (true)
    {
        UdpPeer peer{};
        const int n = srv.recv(rx, sizeof(rx), &peer);
        if (n > 0)
        {
            lastPeer = peer;

            parser.push(rx, (size_t)n);
            while (auto f = parser.tryPop())
            {
                // route
                const auto rr = router.handle(*f);

                // ack back
                auto ack = makeAckFrame(
                    /*req_seq*/ f->seq,
                    /*t_ms*/   f->t_ms,
                    /*ack_cmd*/ f->cmd,
                    /*ack_seq*/ f->seq,
                    /*status*/  rr.status);

                srv.sendTo(lastPeer, ack.data(), ack.size());

                if (!rr.msg.empty())
                    std::cout << "[CMD " << (int)f->cmd << " seq=" << f->seq << "] " << rr.msg << "\n";
            }
        }

        // TODO: your 50Hz offboard loop should live in its own thread.
        // Here we just sleep a bit to reduce CPU in this demo.
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    return 0;
}