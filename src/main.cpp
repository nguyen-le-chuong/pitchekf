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
    double ts;
    Eigen::VectorXd acc, gyro;
    double odo;
    double h_rear, h_front;
    // Read Data
    std::string accelerometer_file = "/home/chuongnl1/project/pitchekf/data/accel.csv";
    std::string gyroscope_file = "/home/chuongnl1/project/pitchekf/data/gyro.csv";

    std::map<double, std::vector<double>> dataA = read_data(accelerometer_file);
    std::map<double, std::vector<double>> dataG = read_data(gyroscope_file);
    
    // Merge the data based on common timestamps
    std::vector<std::tuple<double, Eigen::VectorXd, Eigen::VectorXd>> merged_data = merge_data(dataA, dataG);

    VectorXd RotationState(3);
    VectorXd SlopeState(3);

    RotationState << 0, 0, 1;
    SlopeState << 0, 0, 9.8;
    MatrixXd cov = MatrixXd::Identity(3, 3) * 0.01;
    

    mSimulation.reset(loadSimulation4Parameters(), RotationState, SlopeState, cov);
    double prev_ts = 0.0;
    double dt = 0.0;
    Eigen::Vector2d alpha;
    alpha << 0.01, 0.01;
    bool reset = false;
    double flag_ts = 0.0;
    for (const auto &entry : merged_data) {
        double ts;
        std::tie(ts, acc, gyro) = entry;
        // if (reset) {
        //     RotationState = mSimulation.returnVehicleState();
        //     SlopeState = mSimulation.returnSlopeState();
        //     // cov = mSimulation.returnVehicleCovarience();
        //     mSimulation.reset(loadSimulation4Parameters(), RotationState, SlopeState, cov);
        //     reset = false;
        // }
        if (prev_ts != 0) {
        // std::cout << "prev" << prev_ts << std::endl;
        // std::cout << "ts" << ts << std::endl;
            dt = (ts - prev_ts);
            // std::cout << dt << std::endl;
        }
        // std::cout << alpha << std::endl;
        mSimulation.update(acc, gyro, 0, h_rear, h_front, ts, dt, alpha);
        mSimulation.updateRoadSlope(acc, gyro, 0, dt);
        // if (ts - flag_ts >= 5) {
        //     reset = true;
        //     // std::cout << "al";
        //     flag_ts = ts;
        // }
        prev_ts = ts;
        // if (ts > 5) {
        //     break;
        // }

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
