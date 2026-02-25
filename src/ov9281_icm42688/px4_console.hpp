#pragma once

#include <atomic>
#include <cctype>
#include <chrono>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "mavlink_pose_sender.hpp"

class Px4Console {
  public:
    explicit Px4Console(MavlinkSerial &mav) : m_mavlink(mav) {}

    ~Px4Console() { Stop(); }

    void Start()
    {
        m_running.store(true);
        if (m_thread.joinable())
            m_thread.join();
        m_thread = std::thread([this]() { this->Loop(); });
        printf("[px4console] started!\n");
    }

    void Stop()
    {
        m_running.store(false);
        if (m_thread.joinable())
            m_thread.join();
    }

    bool HandleLine(const std::string &line)
    {
        auto tokens = Tokenize(line);
        if (tokens.empty())
            return true;

        const std::string &cmd = tokens[0];
        printf("[px4console] cmd:");
        for (const auto &token: tokens) {
            printf(" %s", token.c_str());
        }
        printf("\n");

        if (cmd.find("help", 0) == 0 || cmd.find("?", 0) == 0) {
            PrintHelp();
            return true;
        }

        if (cmd.find("exit", 0) == 0 || cmd.find("quit", 0) == 0) {
            m_running.store(false);
            return true;
        }

        if (cmd.find("sp", 0) == 0) {
            // sp <x> <y> <z> [yaw]
            if (tokens.size() < 4) {
                printf("usage: sp <x> <y> <z> [yaw]\n");
                return true;
            }
            float x = (float)ToDouble(tokens[1], 0);
            float y = (float)ToDouble(tokens[2], 0);
            float z = (float)ToDouble(tokens[3], -0.8);
            float yaw = NAN;
            if (tokens.size() >= 5)
                yaw = (float)ToDouble(tokens[4], NAN);

            m_mavlink.UpdateStreamPosition(x, y, z, yaw);
            printf("[px4] setpoint NED x=%f, y=%f, z=%f, yaw=%s\n", x, y, z,
                (std::isfinite(yaw) ? std::to_string(yaw).c_str() : "NAN"));
            return true;
        }

        if (cmd.find("spenu", 0) == 0) {
            if (tokens.size() < 4) {
                printf("usage: spenu <e> <n> <u> [yaw]\n");
                return true;
            }
            float e = (float)ToDouble(tokens[1], 0);
            float n = (float)ToDouble(tokens[2], 0);
            float u = (float)ToDouble(tokens[3], 0);
            float yaw = NAN;
            if (tokens.size() >= 5)
                yaw = (float)ToDouble(tokens[4], NAN);

            float xN = n;
            float yE = e;
            float zD = -u;
            m_mavlink.UpdateStreamPosition(xN, yE, zD, yaw);
            printf("[px4] setpoint ENU->NED x=%f, y=%f, z=%f, yaw=%s\n", xN, yE, zD,
                (std::isfinite(yaw) ? std::to_string(yaw) : "NAN"));
            return true;
        }

        if (cmd.find("stream", 0) == 0) {
            // stream <hz>
            if (tokens.size() < 2) {
                printf("usage: stream <hz>\n");
                return true;
            }
            double hz = ToDouble(tokens[1], 20.0);
            if (hz <= 0) {
                m_mavlink.StopSetpointStream();
                printf("[px4] setpoint stream stopped\n");
            } else {
                m_mavlink.StartSetpointStreamHz(hz);
                printf("[px4] setpoint stream started hz=%f\n", hz);
            }
            return true;
        }

        if (cmd.find("offboard", 0) == 0) {
            // offboard [setpointHz] [warmupMs]
            double setpointHz = (tokens.size() >= 2) ? ToDouble(tokens[1], 20.0) : 20.0;
            int warmupMs = (tokens.size() >= 3) ? (int)ToDouble(tokens[2], 800) : 800;

            m_mavlink.SetModeOffboard();
            printf("[px4] sent DO_SET_MODE OFFBOARD (NOTE: PX4 still requires streaming setpoints)\n");
            return true;
        }

        if (cmd.find("arm", 0) == 0) {
            m_mavlink.Arm(true);
            printf("[px4] arm sent\n");
            return true;
        }

        if (cmd.find("disarm", 0) == 0) {
            m_mavlink.Arm(false);
            printf("[px4] disarm sent\n");
            return true;
        }

        if (cmd.find("go", 0) == 0) {
            // go [setpointHz] [hbHz] [warmupMs]
            double spHz = (tokens.size() >= 2) ? ToDouble(tokens[1], 20.0) : 20.0;
            double hbHz = (tokens.size() >= 3) ? ToDouble(tokens[2], 1.0) : 1.0;
            int warmupMs = (tokens.size() >= 4) ? (int)ToDouble(tokens[3], 800) : 800;

            m_mavlink.StartOffboardAndArm(spHz, hbHz, warmupMs);
            printf("[px4] go: heartbeat+stream+offboard+arm started (spHz=%f, hbHz=%f, warmupMs=%d",
                spHz, hbHz, warmupMs);
            return true;
        }

        if (cmd.find("stop", 0) == 0) {
            m_mavlink.StopSetpointStream();
            printf("[px4] stopped setpoint stream\n");
            return true;
        }

        if (cmd.find("land", 0) == 0) {
            m_mavlink.StopSetpointStream();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            m_mavlink.SendLand();
            printf("[px4] land sent\n");
            return true;
        }

        if (cmd.find("cmd", 0) == 0) {
            // cmd <command> [p1..p7]
            if (tokens.size() < 2) {
                printf("usage: cmd <command> [p1 p2 p3 p4 p5 p6 p7]\n");
                return true;
            }
            uint16_t command = (uint16_t)ToDouble(tokens[1], 0);

            float p[7] = {0, 0, 0, 0, 0, 0, 0};
            for (int i = 0; i < 7; ++i) {
                if ((size_t)(i + 2) < tokens.size())
                    p[i] = (float)ToDouble(tokens[i + 2], 0.0);
            }
            m_mavlink.SendCommandLong(command, p[0], p[1], p[2], p[3], p[4], p[5], p[6]);
            printf("[px4] cmdLong sent cmd=%u\n", command);
            return true;
        }

        printf("[px4] unknown command: %s (type 'help')\n", cmd.c_str());
        return true;
    }

  private:
    MavlinkSerial &m_mavlink;
    std::atomic<bool> m_running{false};
    std::thread m_thread;

    void Loop()
    {
        std::string line{};
        while (m_running.load() && std::getline(std::cin, line)) {
            printf("Input cmd:\n");
            HandleLine(line);
            line.clear();
        }
    }

    void PrintHelp()
    {
        printf("PX4 Console commands:\n"
            "  help | ?                         Show this help\n"
            "  exit | quit                      Exit console thread\n"
            "  sp <x> <y> <z> [yaw]             Update NED position setpoint (meters, yaw rad). e.g. sp 0 0 -0.8\n"
            "  spenu <e> <n> <u> [yaw]          Update ENU setpoint (converted to NED). e.g. spenu 0 0 0.8\n"
            "  stream <hz>                      Start/stop setpoint stream (<=0 stops). e.g. stream 20\n"
            "  go [spHz] [hbHz] [warmupMs]   One-shot: heartbeat+stream+offboard+arm. e.g. go 20 1 800\n"
            "  offboard                          Send OFFBOARD mode command only (PX4 still needs stream)\n"
            "  arm | disarm                      Arm/disarm\n"
            "  land                              Stop stream then send LAND\n"
            "  stop                              Stop setpoint stream + heartbeat\n"
            "  cmd <command> [p1..p7]           Raw COMMAND_LONG. e.g. cmd 21   (LAND)\n"
            "                                   e.g. cmd 400 1 (ARM)\n"
            "                                   e.g. cmd 176 1 393216 (OFFBOARD via DO_SET_MODE)\n"
            "Notes:\n"
            "  - Offboard requires continuous setpoints (recommend 20Hz or higher).\n"
            "  - For NED: z is Down. So hover at 0.8m above origin -> z = -0.8\n");
    }

    static std::vector<std::string> Tokenize(const std::string &s)
    {
        std::istringstream iss(s);
        std::vector<std::string> out;
        std::string tok;
        while (iss >> tok) {
            // lower-case command token only; keep params as-is
            if (out.empty()) {
                for (auto &c : tok)
                    c = (char)std::tolower((unsigned char)c);
            }
            out.push_back(tok);
        }
        return out;
    }

    static double ToDouble(const std::string &s, double def)
    {
        try {
            size_t idx = 0;
            double v = std::stod(s, &idx);
            if (idx != s.size())
                return def;
            return v;
        } catch (...) {
            return def;
        }
    }
};
