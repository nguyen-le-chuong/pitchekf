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
    std::string odo_file = "/home/chuongnl1/project/EKF/record_data/stable_files/files/gyroscope.csv";

    std::string front_file = "";
    std::string rear_file = "";

    std::map<time_t, std::vector<double>> dataA = read_data(accelerometer_file);
    std::map<time_t, std::vector<double>> dataG = read_data(gyroscope_file);

    std::map<time_t, std::vector<double>> dataF = read_data(front_file);
    std::map<time_t, std::vector<double>> dataR = read_data(rear_file);
    // std::map<time_t, std::vector<double>> dataO = read_data(odo_file);
    
    
    // Merge the data based on common timestamps
    std::vector<std::tuple<time_t, Eigen::VectorXd, Eigen::VectorXd>> merged_data = merge_data(dataA, dataG);
    // std::vector<std::tuple<time_t, std::vector<double>, std::vector<double>>> merged_data = merge_data(merged_data, dataO);
    // std::vector<std::tuple<time_t, std::vector<double>, std::vector<double>>> merged_data_high = merge_data(dataA, dataG);

    // if (!merged_data.empty()) {
    //     time_t first_ts;
    //     std::vector<float> first_acc, first_gyro;
    //     std::tie(first_ts, first_acc, first_gyro) = merged_data[0];  

    mSimulation.reset(loadSimulation4Parameters());
    time_t prev_ts = 0;
    double delta_t = 0;
    for (const auto &entry : merged_data) {
        time_t ts;
        std::tie(ts, acc, gyro) = entry;

        if (prev_ts != 0) {
            delta_t = difftime(ts, prev_ts);
        }
        mSimulation.update(acc, gyro, odo, h_rear, h_front, ts, delta_t);

        prev_ts = ts;

    }
    // std::vector<time_t> x = mSimulation.m_time_history;
    // std::vector<double> y1 = mSimulation.m_filter_pitch_history;   // First y dataset
    // std::vector<double> y2 = mSimulation.m_wheel_pitch_history;    // Second y dataset

    // Create the plot
    // plt::plot(x, y1, "r-");  // Plot y1 in red
    // plt::plot(x, y2, "b-");  // Plot y2 in blue
    // plt::title("EKF");
    // plt::xlabel("time");
    // plt::ylabel("pitch");
    
    // // Save the plot as a PNG image
    // plt::save("plot.png");

    // return 0;
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
