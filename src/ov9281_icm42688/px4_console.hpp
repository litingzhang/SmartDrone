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
    explicit Px4Console(MavlinkSerial &mav) : mav_(mav) {}

    ~Px4Console() { stop(); }

    void start()
    {
        running_.store(true);
        if (thread_.joinable())
            thread_.join();
        thread_ = std::thread([this]() { this->loop(); });
        printf("[px4console] started!\n");
    }

    void stop()
    {
        running_.store(false);
        if (thread_.joinable())
            thread_.join();
    }

    bool handleLine(const std::string &line)
    {
        auto tokens = tokenize(line);
        if (tokens.empty())
            return true;

        const std::string &cmd = tokens[0];
        printf("[px4console] cmd:");
        for (const auto &token: tokens) {
            printf(" %s", token.c_str());
        }
        printf("\n");

        if (cmd.find("help", 0) == 0 || cmd.find("?", 0) == 0) {
            printHelp();
            return true;
        }

        if (cmd.find("exit", 0) == 0 || cmd.find("quit", 0) == 0) {
            running_.store(false);
            return true;
        }

        if (cmd.find("sp", 0) == 0) {
            // sp <x> <y> <z> [yaw]
            if (tokens.size() < 4) {
                printf("usage: sp <x> <y> <z> [yaw]\n");
                return true;
            }
            float x = (float)toDouble(tokens[1], 0);
            float y = (float)toDouble(tokens[2], 0);
            float z = (float)toDouble(tokens[3], -0.8);
            float yaw = NAN;
            if (tokens.size() >= 5)
                yaw = (float)toDouble(tokens[4], NAN);

            mav_.updateStreamPosition(x, y, z, yaw);
            printf("[px4] setpoint NED x=%f, y=%f, z=%f, yaw=%s\n", x, y, z,
                (std::isfinite(yaw) ? std::to_string(yaw).c_str() : "NAN"));
            return true;
        }

        if (cmd.find("spenu", 0) == 0) {
            if (tokens.size() < 4) {
                printf("usage: spenu <e> <n> <u> [yaw]\n");
                return true;
            }
            float e = (float)toDouble(tokens[1], 0);
            float n = (float)toDouble(tokens[2], 0);
            float u = (float)toDouble(tokens[3], 0);
            float yaw = NAN;
            if (tokens.size() >= 5)
                yaw = (float)toDouble(tokens[4], NAN);

            float x_n = n;
            float y_e = e;
            float z_d = -u;
            mav_.updateStreamPosition(x_n, y_e, z_d, yaw);
            printf("[px4] setpoint ENU->NED x=%f, y=%f, z=%f, yaw=%s\n", x_n, y_e, z_d,
                (std::isfinite(yaw) ? std::to_string(yaw) : "NAN"));
            return true;
        }

        if (cmd.find("stream", 0) == 0) {
            // stream <hz>
            if (tokens.size() < 2) {
                printf("usage: stream <hz>\n");
                return true;
            }
            double hz = toDouble(tokens[1], 20.0);
            if (hz <= 0) {
                mav_.stopSetpointStream();
                printf("[px4] setpoint stream stopped\n");
            } else {
                mav_.startSetpointStreamHz(hz);
                printf("[px4] setpoint stream started hz=%f\n", hz);
            }
            return true;
        }

        if (cmd.find("offboard", 0) == 0) {
            // offboard [setpoint_hz] [warmup_ms]
            double setpoint_hz = (tokens.size() >= 2) ? toDouble(tokens[1], 20.0) : 20.0;
            int warmup_ms = (tokens.size() >= 3) ? (int)toDouble(tokens[2], 800) : 800;

            mav_.setModeOffboard();
            printf("[px4] sent DO_SET_MODE OFFBOARD (NOTE: PX4 still requires streaming setpoints)\n");
            return true;
        }

        if (cmd.find("arm", 0) == 0) {
            mav_.arm(true);
            printf("[px4] arm sent\n");
            return true;
        }

        if (cmd.find("disarm", 0) == 0) {
            mav_.arm(false);
            printf("[px4] disarm sent\n");
            return true;
        }

        if (cmd.find("go", 0) == 0) {
            // go [setpoint_hz] [hb_hz] [warmup_ms]
            double sp_hz = (tokens.size() >= 2) ? toDouble(tokens[1], 20.0) : 20.0;
            double hb_hz = (tokens.size() >= 3) ? toDouble(tokens[2], 1.0) : 1.0;
            int warmup_ms = (tokens.size() >= 4) ? (int)toDouble(tokens[3], 800) : 800;

            mav_.startOffboardAndArm(sp_hz, hb_hz, warmup_ms);
            printf("[px4] go: heartbeat+stream+offboard+arm started (sp_hz=%f, hb_hz=%f, warmup_ms=%d",
                sp_hz, hb_hz, warmup_ms);
            return true;
        }

        if (cmd.find("stop", 0) == 0) {
            mav_.stopSetpointStream();
            printf("[px4] stopped setpoint stream\n");
            return true;
        }

        if (cmd.find("land", 0) == 0) {
            mav_.stopSetpointStream();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            mav_.sendLand();
            printf("[px4] land sent\n");
            return true;
        }

        if (cmd.find("cmd", 0) == 0) {
            // cmd <command> [p1..p7]
            if (tokens.size() < 2) {
                printf("usage: cmd <command> [p1 p2 p3 p4 p5 p6 p7]\n");
                return true;
            }
            uint16_t command = (uint16_t)toDouble(tokens[1], 0);

            float p[7] = {0, 0, 0, 0, 0, 0, 0};
            for (int i = 0; i < 7; ++i) {
                if ((size_t)(i + 2) < tokens.size())
                    p[i] = (float)toDouble(tokens[i + 2], 0.0);
            }
            mav_.sendCommandLong(command, p[0], p[1], p[2], p[3], p[4], p[5], p[6]);
            printf("[px4] cmd_long sent cmd=%u\n", command);
            return true;
        }

        printf("[px4] unknown command: %s (type 'help')\n", cmd.c_str());
        return true;
    }

  private:
    MavlinkSerial &mav_;
    std::atomic<bool> running_{false};
    std::thread thread_;

    void loop()
    {
        std::string line{};
        while (running_.load() && std::getline(std::cin, line)) {
            printf("Input cmd:\n");
            handleLine(line);
            line.clear();
        }
    }

    void printHelp()
    {
        printf("PX4 Console commands:\n"
            "  help | ?                         Show this help\n"
            "  exit | quit                      Exit console thread\n"
            "  sp <x> <y> <z> [yaw]             Update NED position setpoint (meters, yaw rad). e.g. sp 0 0 -0.8\n"
            "  spenu <e> <n> <u> [yaw]          Update ENU setpoint (converted to NED). e.g. spenu 0 0 0.8\n"
            "  stream <hz>                      Start/stop setpoint stream (<=0 stops). e.g. stream 20\n"
            "  go [sp_hz] [hb_hz] [warmup_ms]   One-shot: heartbeat+stream+offboard+arm. e.g. go 20 1 800\n"
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

    static std::vector<std::string> tokenize(const std::string &s)
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

    static double toDouble(const std::string &s, double def)
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
