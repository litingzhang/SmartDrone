#pragma once

#include <atomic>
#include <chrono>
#include <cctype>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "mavlink_pose_sender.hpp"

class Px4Console {
public:
    explicit Px4Console(MavlinkSerial& mav)
        : mav_(mav)
    {}

    ~Px4Console() { stop(); }

    void start() {
        running_.store(true);
        if (thread_.joinable()) thread_.join();
        thread_ = std::thread([this]() { this->loop(); });
    }

    void stop() {
        running_.store(false);
        if (thread_.joinable()) thread_.join();
    }

    bool handleLine(const std::string& line) {
        auto tokens = tokenize(line);
        if (tokens.empty()) return true;

        const std::string& cmd = tokens[0];

        if (cmd == "help" || cmd == "?") {
            printHelp();
            return true;
        }

        if (cmd == "exit" || cmd == "quit") {
            running_.store(false);
            return true;
        }

        if (cmd == "sp") {
            // sp <x> <y> <z> [yaw]
            if (tokens.size() < 4) {
                std::cout << "usage: sp <x> <y> <z> [yaw]\n";
                return true;
            }
            float x = (float)toDouble(tokens[1], 0);
            float y = (float)toDouble(tokens[2], 0);
            float z = (float)toDouble(tokens[3], -0.8);
            float yaw = NAN;
            if (tokens.size() >= 5) yaw = (float)toDouble(tokens[4], NAN);

            mav_.updateStreamPosition(x, y, z, yaw);
            std::cout << "[px4] setpoint NED x=" << x << " y=" << y << " z=" << z
                      << " yaw=" << (std::isfinite(yaw) ? std::to_string(yaw) : "NAN") << "\n";
            return true;
        }

        if (cmd == "spenu") {
            if (tokens.size() < 4) {
                std::cout << "usage: spenu <e> <n> <u> [yaw]\n";
                return true;
            }
            float e = (float)toDouble(tokens[1], 0);
            float n = (float)toDouble(tokens[2], 0);
            float u = (float)toDouble(tokens[3], 0);
            float yaw = NAN;
            if (tokens.size() >= 5) yaw = (float)toDouble(tokens[4], NAN);

            float x_n = n;
            float y_e = e;
            float z_d = -u;
            mav_.updateStreamPosition(x_n, y_e, z_d, yaw);
            std::cout << "[px4] setpoint ENU->NED x=" << x_n << " y=" << y_e << " z=" << z_d
                      << " yaw=" << (std::isfinite(yaw) ? std::to_string(yaw) : "NAN") << "\n";
            return true;
        }

        if (cmd == "stream") {
            // stream <hz>
            if (tokens.size() < 2) {
                std::cout << "usage: stream <hz>\n";
                return true;
            }
            double hz = toDouble(tokens[1], 20.0);
            if (hz <= 0) {
                mav_.stopSetpointStream();
                std::cout << "[px4] setpoint stream stopped\n";
            } else {
                mav_.startSetpointStreamHz(hz);
                std::cout << "[px4] setpoint stream started hz=" << hz << "\n";
            }
            return true;
        }

        if (cmd == "offboard") {
            // offboard [setpoint_hz] [warmup_ms]
            double setpoint_hz = (tokens.size() >= 2) ? toDouble(tokens[1], 20.0) : 20.0;
            int warmup_ms = (tokens.size() >= 3) ? (int)toDouble(tokens[2], 800) : 800;

            mav_.setModeOffboard();
            std::cout << "[px4] sent DO_SET_MODE OFFBOARD (NOTE: PX4 still requires streaming setpoints)\n";
            return true;
        }

        if (cmd == "arm") {
            mav_.arm(true);
            std::cout << "[px4] arm sent\n";
            return true;
        }

        if (cmd == "disarm") {
            mav_.arm(false);
            std::cout << "[px4] disarm sent\n";
            return true;
        }

        if (cmd == "go") {
            // go [setpoint_hz] [hb_hz] [warmup_ms]
            double sp_hz = (tokens.size() >= 2) ? toDouble(tokens[1], 20.0) : 20.0;
            double hb_hz = (tokens.size() >= 3) ? toDouble(tokens[2], 1.0) : 1.0;
            int warmup_ms = (tokens.size() >= 4) ? (int)toDouble(tokens[3], 800) : 800;

            mav_.startOffboardAndArm(sp_hz, hb_hz, warmup_ms);
            std::cout << "[px4] go: heartbeat+stream+offboard+arm started (sp_hz=" << sp_hz
                      << ", hb_hz=" << hb_hz << ", warmup_ms=" << warmup_ms << ")\n";
            return true;
        }

        if (cmd == "stop") {
            mav_.stopSetpointStream();
            std::cout << "[px4] stopped setpoint stream\n";
            return true;
        }

        if (cmd == "land") {
            mav_.stopSetpointStream();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            mav_.sendLand();
            std::cout << "[px4] land sent\n";
            return true;
        }

        if (cmd == "cmd") {
            // cmd <command> [p1..p7]
            if (tokens.size() < 2) {
                std::cout << "usage: cmd <command> [p1 p2 p3 p4 p5 p6 p7]\n";
                return true;
            }
            uint16_t command = (uint16_t)toDouble(tokens[1], 0);

            float p[7] = {0,0,0,0,0,0,0};
            for (int i = 0; i < 7; ++i) {
                if ((size_t)(i + 2) < tokens.size()) p[i] = (float)toDouble(tokens[i + 2], 0.0);
            }
            mav_.sendCommandLong(command, p[0],p[1],p[2],p[3],p[4],p[5],p[6]);
            std::cout << "[px4] cmd_long sent cmd=" << command << "\n";
            return true;
        }

        std::cout << "[px4] unknown command: " << cmd << " (type 'help')\n";
        return true;
    }

private:
    MavlinkSerial& mav_;
    std::atomic<bool> running_{false};
    std::thread thread_;

    void loop() {
        printHelp();
        std::string line;
        while (running_.load() && std::getline(std::cin, line)) {
            handleLine(line);
        }
    }

    static void printHelp() {
        std::cout <<
R"HELP(
PX4 Console commands:
  help | ?                         Show this help
  exit | quit                      Exit console thread

  hb [hz]                          Start heartbeat at hz (<=0 stops). e.g. hb 1
  sp <x> <y> <z> [yaw]             Update NED position setpoint (meters, yaw rad). e.g. sp 0 0 -0.8
  spenu <e> <n> <u> [yaw]          Update ENU setpoint (converted to NED). e.g. spenu 0 0 0.8
  stream <hz>                      Start/stop setpoint stream (<=0 stops). e.g. stream 20

  go [sp_hz] [hb_hz] [warmup_ms]   One-shot: heartbeat+stream+offboard+arm. e.g. go 20 1 800
  offboard                          Send OFFBOARD mode command only (PX4 still needs stream)
  arm | disarm                      Arm/disarm
  land                              Stop stream then send LAND
  stop                              Stop setpoint stream + heartbeat

  cmd <command> [p1..p7]           Raw COMMAND_LONG. e.g. cmd 21   (LAND)
                                   e.g. cmd 400 1 (ARM)
                                   e.g. cmd 176 1 393216 (OFFBOARD via DO_SET_MODE)
Notes:
  - Offboard requires continuous setpoints (recommend 20Hz or higher).
  - For NED: z is Down. So hover at 0.8m above origin -> z = -0.8
)HELP";
    }

    static std::vector<std::string> tokenize(const std::string& s) {
        std::istringstream iss(s);
        std::vector<std::string> out;
        std::string tok;
        while (iss >> tok) {
            // lower-case command token only; keep params as-is
            if (out.empty()) {
                for (auto& c : tok) c = (char)std::tolower((unsigned char)c);
            }
            out.push_back(tok);
        }
        return out;
    }

    static double toDouble(const std::string& s, double def) {
        try {
            size_t idx = 0;
            double v = std::stod(s, &idx);
            if (idx != s.size()) return def;
            return v;
        } catch (...) {
            return def;
        }
    }
};
