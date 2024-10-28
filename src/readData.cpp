#include "readData.h"

// Placeholder for your timestamp parsing function
time_t parse_timestamp(const std::string &timestamp) {
    std::tm t = {};
    std::istringstream ss(timestamp);
    ss >> std::get_time(&t, "%Y-%m-%d %H:%M:%S"); // Adjust the format as needed
    return std::mktime(&t);
}
Eigen::VectorXd to_eigen_vector(const std::vector<double> &vec) {
    return Eigen::Map<const Eigen::VectorXd>(vec.data(), vec.size());
}

// Function to read data from a CSV file
std::map<time_t, std::vector<double>> read_data(const std::string &filename) {
    std::map<time_t, std::vector<double>> data;
    std::ifstream file(filename);
    std::string line;

    if (file.is_open()) {
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            std::vector<double> values;
            time_t ts;

            bool is_first = true;
            while (std::getline(ss, token, ',')) {
                if (is_first) {
                    ts = parse_timestamp(token); // Parse timestamp
                    is_first = false;
                } else {
                    values.push_back(std::stof(token)); // Convert string to double
                }
            }
            data[ts] = values;
        }
        file.close();
    } else {
        std::cerr << "Unable to open file: " << filename << "\n";
    }

    return data;
}

// Function to extract timestamps from a map of time_t to vector of doubles
std::set<time_t> extract_keys(const std::map<time_t, std::vector<double>> &data) {
    std::set<time_t> keys;
    for (const auto &entry : data) {
        keys.insert(entry.first);
    }
    return keys;
}

// Function to merge data based on common timestamps
std::vector<std::tuple<time_t, Eigen::VectorXd, Eigen::VectorXd>> merge_data(
    const std::map<time_t, std::vector<double>> &dataA,
    const std::map<time_t, std::vector<double>> &dataG) {
    
    std::set<time_t> keysA = extract_keys(dataA);
    std::set<time_t> keysG = extract_keys(dataG);

    // Find common timestamps
    std::set<time_t> common_timestamps;
    std::set_intersection(keysA.begin(), keysA.end(), keysG.begin(), keysG.end(),
                          std::inserter(common_timestamps, common_timestamps.begin()));

    // Merge data based on common timestamps
    std::vector<std::tuple<time_t, Eigen::VectorXd, Eigen::VectorXd>> merged_data;
    for (const auto &ts : common_timestamps) {
        Eigen::VectorXd acc = to_eigen_vector(dataA.at(ts));
        Eigen::VectorXd gyro = to_eigen_vector(dataG.at(ts));
        merged_data.push_back(std::make_tuple(ts, acc, gyro));
    }

    return merged_data;
}
