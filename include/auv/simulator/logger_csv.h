#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace auv::simulator {

class CsvLogger {
public:
    CsvLogger() = default;
    CsvLogger(const std::filesystem::path& path, std::vector<std::string> header);

    void open(const std::filesystem::path& path, std::vector<std::string> header);
    void writeRow(const std::vector<double>& values);
    void close();

    bool isOpen() const;
    const std::vector<std::string>& header() const;

private:
    void writeHeader();

    std::ofstream stream_;
    std::vector<std::string> header_;
};

}  // namespace auv::simulator
