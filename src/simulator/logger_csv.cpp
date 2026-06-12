#include "auv/simulator/logger_csv.h"

#include <iomanip>
#include <stdexcept>

namespace auv::simulator {

CsvLogger::CsvLogger(
    const std::filesystem::path& path,
    std::vector<std::string> header
) {
    open(path, std::move(header));
}

void CsvLogger::open(
    const std::filesystem::path& path,
    std::vector<std::string> header
) {
    close();

    if (header.empty()) {
        throw std::invalid_argument("CSV header must not be empty");
    }

    const auto parent = path.parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }

    stream_.open(path, std::ios::out | std::ios::trunc);
    if (!stream_.is_open()) {
        throw std::runtime_error("Failed to open CSV file: " + path.string());
    }

    stream_ << std::setprecision(9);
    header_ = std::move(header);
    writeHeader();
}

void CsvLogger::writeRow(const std::vector<double>& values) {
    if (!stream_.is_open()) {
        throw std::runtime_error("CSV logger is not open");
    }

    if (values.size() != header_.size()) {
        throw std::invalid_argument("CSV row size does not match header size");
    }

    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) {
            stream_ << ',';
        }
        stream_ << values[i];
    }
    stream_ << '\n';
}

void CsvLogger::close() {
    if (stream_.is_open()) {
        stream_.close();
    }
}

bool CsvLogger::isOpen() const {
    return stream_.is_open();
}

const std::vector<std::string>& CsvLogger::header() const {
    return header_;
}

void CsvLogger::writeHeader() {
    for (std::size_t i = 0; i < header_.size(); ++i) {
        if (i != 0) {
            stream_ << ',';
        }
        stream_ << header_[i];
    }
    stream_ << '\n';
}

}  // namespace auv::simulator
