#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <vector>
#include <set>
#include <algorithm>
#include <iterator>
#include <ctime>
#include <iomanip>
#include <cstdlib>

// Placeholder for your timestamp parsing function
time_t parse_timestamp(const std::string &timestamp) {
    std::tm t = {};
    std::istringstream ss(timestamp);
    ss >> std::get_time(&t, "%Y-%m-%d %H:%M:%S"); // Adjust the format as needed
    return std::mktime(&t);
}

int main() {
    std::string filename_info = "/home/chuongnl1/project/EKF/record_data/stable_files/files/accelerometer.csv";
    std::string line;

    // Data structures to hold timestamped accelerometer and gyroscope data
    std::map<time_t, std::vector<float>> dataA;
    std::map<time_t, std::vector<float>> dataG;

    // Reading accelerometer data
    std::ifstream fileA(filename_info);
    if (fileA.is_open()) {
        while (std::getline(fileA, line)) {
            std::stringstream ss(line);
            std::string token;
            std::vector<float> values;
            time_t ts;
            
            bool is_first = true;
            while (std::getline(ss, token, ',')) {
                if (is_first) {
                    ts = parse_timestamp(token); // Parse timestamp
                    is_first = false;
                } else {
                    values.push_back(std::stof(token)); // Convert string to float
                }
            }
            dataA[ts] = values;
        }
        fileA.close();
    } else {
        std::cerr << "Unable to open accelerometer file\n";
    }

    // Reading gyroscope data
    std::string filename = "/home/chuongnl1/project/EKF/record_data/stable_files/files/gyroscope.csv";
    std::ifstream fileG(filename);
    if (fileG.is_open()) {
        while (std::getline(fileG, line)) {
            std::stringstream ss(line);
            std::string token;
            std::vector<float> values;
            time_t ts;

            bool is_first = true;
            while (std::getline(ss, token, ',')) {
                if (is_first) {
                    ts = parse_timestamp(token); // Parse timestamp
                    is_first = false;
                } else {
                    values.push_back(std::stof(token)); // Convert string to float
                }
            }
            dataG[ts] = values;
        }
        fileG.close();
    } else {
        std::cerr << "Unable to open gyroscope file\n";
    }



    std::string front_filename = "";
    std::ifstream fileG(front_filename);
    if (fileG.is_open()){
        while (std::getline(fileG, line)) {
            std::stringstream ss(line);
            std::string token;
            std::vector<float> values;
            time_t ts;

            bool is_first = true;
            while (std::getline(ss, token, ',')){
                if (is_first) {
                    ts = parse_timestamp(token);
                    is_first = false;
                } else {
                    values.push_back(std::stof(token));
                }
            }
        }
        data[ts] = values;
    }
    // Extract keys (timestamps) from dataA and dataG
    std::set<time_t> keysA, keysG;
    for (const auto &entry : dataA) {
        keysA.insert(entry.first);
    }
    for (const auto &entry : dataG) {
        keysG.insert(entry.first);
    }

    // Find common timestamps
    std::set<time_t> common_timestamps;
    std::set_intersection(keysA.begin(), keysA.end(), keysG.begin(), keysG.end(),
                          std::inserter(common_timestamps, common_timestamps.begin()));

    // Merge data based on common timestamps
    std::vector<std::tuple<time_t, std::vector<float>, std::vector<float>>> data;
    for (const auto &ts : common_timestamps) {
        data.push_back(std::make_tuple(ts, dataA[ts], dataG[ts]));
    }

    // Example output
    for (const auto &entry : data) {
        time_t ts;
        std::vector<float> acc, gyro;
        std::tie(ts, acc, gyro) = entry;

        std::cout << "Timestamp: " << ts << "\n";
        std::cout << "Accelerometer: ";
        for (const auto &val : acc) std::cout << val << " ";
        std::cout << "\nGyroscope: ";
        for (const auto &val : gyro) std::cout << val << " ";
        std::cout << "\n";
    }

    return 0;
}
