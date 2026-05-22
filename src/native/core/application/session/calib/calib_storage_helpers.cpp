#include "core/application/session/calib/calib_storage_helpers.h"

#include <algorithm>
#include <system_error>

namespace SmartDrone::Core::Application {

void EnsureDir(const fs::path &p)
{
    std::error_code ec;
    fs::create_directories(p, ec);
}

std::string TsToName(int64_t tNs)
{
    return std::to_string(tNs) + ".png";
}

void SetupFileBuffer(FILE *f, size_t bytes)
{
    if (f) {
        setvbuf(f, nullptr, _IOFBF, bytes);
    }
}

bool TryParseCalibIndex(const std::string &name, int &indexOut)
{
    static const std::string prefix = "calib_data_";
    if (name.rfind(prefix, 0) != 0) {
        return false;
    }
    const std::string suffix = name.substr(prefix.size());
    if (suffix.empty()) {
        return false;
    }
    for (char c : suffix) {
        if (c < '0' || c > '9') {
            return false;
        }
    }
    try {
        indexOut = std::stoi(suffix);
        return indexOut >= 0;
    } catch (...) {
        return false;
    }
}

std::string MakeCalibSessionDir(const std::string &root)
{
    EnsureDir(fs::path(root));
    int maxIndex = -1;
    std::error_code ec;
    for (const auto &entry : fs::directory_iterator(root, ec)) {
        if (ec || !entry.is_directory()) {
            continue;
        }
        int index = -1;
        if (TryParseCalibIndex(entry.path().filename().string(), index)) {
            maxIndex = std::max(maxIndex, index);
        }
    }
    return (fs::path(root) / ("calib_data_" + std::to_string(maxIndex + 1))).string();
}

int CleanupCalibDataDirs(const std::string &root)
{
    int removed = 0;
    std::error_code ec;
    if (!fs::exists(root, ec)) {
        return 0;
    }
    for (const auto &entry : fs::directory_iterator(root, ec)) {
        if (ec || !entry.is_directory()) {
            continue;
        }
        int index = -1;
        if (!TryParseCalibIndex(entry.path().filename().string(), index)) {
            continue;
        }
        std::error_code rmEc;
        const auto n = fs::remove_all(entry.path(), rmEc);
        if (!rmEc && n > 0) {
            removed++;
        }
    }
    return removed;
}

} // namespace SmartDrone::Core::Application
