#pragma once

#include <cstdio>
#include <filesystem>
#include <string>

namespace smartdrone::core::application {

namespace fs = std::filesystem;

void EnsureDir(const fs::path &p);
std::string TsToName(int64_t tNs);
void SetupFileBuffer(FILE *f, size_t bytes);
bool TryParseCalibIndex(const std::string &name, int &indexOut);
std::string MakeCalibSessionDir(const std::string &root);
int CleanupCalibDataDirs(const std::string &root);

} // namespace smartdrone::core::application
