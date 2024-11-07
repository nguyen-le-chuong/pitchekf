// System Includes
#include <string>
#include <iostream>


#include "simulation.h"
#include "car.h"
#include "readData.h"
// #include "matplotlibcpp.h"

// Screen dimension constants
const int SCREEN_WIDTH = 1024;
const int SCREEN_HEIGHT = 768;
const double GRID_SIZE = 500;
const double GRID_SPACEING = 25;

// Function Prototypes
SimulationParams loadSimulation4Parameters();

// Main Loop
int main( int argc, char* args[] )
{
    // Display mDisplay;
    Simulation mSimulation;
    time_t ts;
    Eigen::VectorXd acc, gyro;
    double odo;
    double h_rear, h_front;
    // Read Data
    std::string accelerometer_file = "/home/chuongnl1/project/EKF/record_data/stable_files/files/accelerometer.csv";
    std::string gyroscope_file = "/home/chuongnl1/project/EKF/record_data/stable_files/files/gyroscope.csv";

    std::map<time_t, std::vector<double>> dataA = read_data(accelerometer_file);
    std::map<time_t, std::vector<double>> dataG = read_data(gyroscope_file);
    
    // Merge the data based on common timestamps
    std::vector<std::tuple<time_t, Eigen::VectorXd, Eigen::VectorXd>> merged_data = merge_data(dataA, dataG);

    mSimulation.reset(loadSimulation4Parameters());
    time_t prev_ts = 0;
    double delta_t = 0;
    for (const auto &entry : merged_data) {
        time_t ts;
        std::tie(ts, acc, gyro) = entry;
        if (prev_ts != 0) {
            delta_t = difftime(ts, prev_ts)/1000000.0;
        }
        mSimulation.update(acc, gyro, 5, h_rear, h_front, ts, delta_t);
        mSimulation.updateRoadSlope(acc, gyro, 5, delta_t);

        prev_ts = ts;

    }
    mSimulation.writeVectorsToCSV("output.csv");

    return 0;
}

SimulationParams loadSimulation4Parameters()
{    
    SimulationParams sim_params;
    sim_params.profile_name = "4 - Variable Speed Profile + GPS + GYRO";
    sim_params.end_time = 200;
    sim_params.car_initial_velocity = 0;
    sim_params.odo_enabled = true;
    sim_params.gyro_enabled = true;
    sim_params.accel_enabled = true;
    // sim_params.car_initial_psi = M_PI/180.0 * 45.0;

    return sim_params;
}
