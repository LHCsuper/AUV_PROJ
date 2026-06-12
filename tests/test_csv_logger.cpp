#include "auv/simulator/logger_csv.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

int expect(bool condition, const char* label) {
    if (!condition) {
        std::cerr << label << " failed\n";
        return 1;
    }
    return 0;
}

}  // namespace

int main() {
    using auv::simulator::CsvLogger;

    const std::filesystem::path outputPath =
        std::filesystem::current_path() / "test_outputs" / "logger_test.csv";

    const std::vector<std::string> header{
        "time",
        "x",
        "y",
        "psi",
        "u",
        "v",
        "r"
    };

    {
        CsvLogger logger(outputPath, header);
        logger.writeRow({0.0, 1.0, 2.0, 0.1, 0.5, 0.0, 0.01});
        logger.writeRow({0.1, 1.1, 2.0, 0.12, 0.5, 0.0, 0.01});
        logger.close();
    }

    int failures = 0;
    failures += expect(std::filesystem::exists(outputPath), "CSV file exists");

    std::ifstream input(outputPath);
    failures += expect(input.is_open(), "CSV file opens");

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
        lines.push_back(line);
    }

    failures += expect(lines.size() == 3, "CSV line count");
    if (lines.size() == 3) {
        failures += expect(lines[0] == "time,x,y,psi,u,v,r", "CSV header");
        failures += expect(splitCsvLine(lines[1]).size() == header.size(), "row 1 field count");
        failures += expect(splitCsvLine(lines[2]).size() == header.size(), "row 2 field count");
        failures += expect(lines[1].find("0.5") != std::string::npos, "numeric value written");
    }

    const std::filesystem::path invalidOutputPath =
        std::filesystem::current_path() / "test_outputs" / "logger_invalid_row_test.csv";

    bool rowSizeRejected = false;
    try {
        CsvLogger logger(invalidOutputPath, header);
        logger.writeRow({1.0, 2.0});
    } catch (const std::invalid_argument&) {
        rowSizeRejected = true;
    }
    failures += expect(rowSizeRejected, "row size rejection");

    if (failures != 0) {
        return 1;
    }

    std::cout << "test_csv_logger passed\n";
    return 0;
}
