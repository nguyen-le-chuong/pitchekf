
#include "simulation.h"
#include "utils.h"
#include "math.h"

Simulation::Simulation()
: m_sim_parameters(SimulationParams()),
    m_is_paused(false),
    m_is_running(false),
    m_time_multiplier(1),
    m_view_size(100),
    m_time(0.0),
    m_time_till_gyro_measurement(0.0),
    m_time_till_accel_measurement(0.0),
    m_time_till_odo_measurement(0.0)
{}

void Simulation::reset()
{
    // Reset Simulation
    m_time = 0.0;
    m_time_till_gyro_measurement = 0.0;
    m_time_till_accel_measurement = 0.0;
    m_time_till_odo_measurement = 0.0;
  

    m_is_running = true;
    m_is_paused = false;
    
    m_kalman_filter.reset();


    m_gyro_sensor.reset();
    m_gyro_sensor.setGyroNoiseStd(m_sim_parameters.gyro_noise_std);
    m_gyro_sensor.setGyroBias(m_sim_parameters.gyro_bias);

    m_accel_sensor.reset();
    m_accel_sensor.setAccelNoiseStd(m_sim_parameters.accel_noise_std);
    m_accel_sensor.setAccelBias(m_sim_parameters.accel_bias);

    m_odo_sensor.reset();
    m_odo_sensor.setOdoNoiseStd(m_sim_parameters.odo_noise_std);
    m_odo_sensor.setOdoBias(m_sim_parameters.odo_bias);

    m_front_sensor.reset();
    m_rear_sensor.reset();


    m_car.reset(0, 0, 0);
    


    // Stats Variables
    m_filter_error_x_position_history.clear();
    m_filter_error_y_position_history.clear();
    m_filter_error_heading_history.clear();
    m_filter_error_velocity_history.clear();

    std::cout << "Simulation: Reset" << std::endl;
}



void Simulation::writeVectorsToCSV(const std::string& filename) {
    
    std::vector<time_t> vec1 = m_time_history;
    std::vector<double> vec2 = m_filter_pitch_history;
    std::vector<double> vec3 = m_wheel_pitch_history;
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    size_t maxSize = std::max({vec1.size(), vec2.size(), vec3.size()});

    for (size_t i = 0; i < maxSize; ++i) {
        if (i < vec1.size()) file << vec1[i];
        file << ","; // Column separator

        if (i < vec2.size()) file << vec2[i];
        file << ",";

        if (i < vec3.size()) file << vec3[i];
        file << "\n"; // Newline at the end of each row
    }

    file.close();
    std::cout << "Data successfully written to " << filename << std::endl;
}



void Simulation::update(Eigen::VectorXd acc, Eigen::VectorXd gyro, double odo, double h_rear, double h_front, time_t& m_time, double& delta_t)
{
    m_sim_parameters.time_step = delta_t;
    if (m_is_running && !m_is_paused)
    {
            // Update Motion (for Front and Rear ) 
            m_car.update(h_rear, h_front);
            m_vehicle_pitch_history.push_back(m_car.getWheelState().pitch);

            // Gyro Measurement / Prediction Step
            GyroMeasurement gyro_meas;  // Khai báo biến gyro_meas
            AccelMeasurement accel_meas; // Khai báo biến accel_meas
            OdoMeasurement v_meas;  // Khai báo biến v_meas

            if (m_sim_parameters.gyro_enabled)
            {
                    gyro_meas = m_gyro_sensor.generateGyroMeasurement(gyro);
                    m_kalman_filter.predictionStep(gyro_meas, m_sim_parameters.time_step);
                    m_time_till_gyro_measurement += 1.0/m_sim_parameters.gyro_update_rate;
     
            }

            if (m_sim_parameters.accel_enabled)
            {
     
                    accel_meas = m_accel_sensor.generateAccelMeasurement(acc);
                    m_time_till_accel_measurement += 1.0/m_sim_parameters.accel_update_rate;
    
            }

            if (m_sim_parameters.odo_enabled)
            {
         
                    v_meas = m_odo_sensor.generateOdoMeasurement(odo);
                    m_time_till_odo_measurement += 1.0/m_sim_parameters.odo_update_rate;

            }
            m_kalman_filter.measurementStep1(accel_meas, gyro_meas, v_meas.v);
            m_kalman_filter.measurementStep2();

            // Save Filter History and Calculate Stats
            if (m_kalman_filter.isInitialised())
            {
                WheelState wheel_state = m_car.getWheelState();
                VehicleState filter_state = m_kalman_filter.getVehicleState();
                m_filter_error_pitch_history.push_back(filter_state.pitch - wheel_state.pitch);
                m_filter_pitch_history.push_back(filter_state.pitch);
                m_wheel_pitch_history.push_back(wheel_state.pitch);
                m_time_history.push_back(m_time);
            }

            // Update Time
            m_time += m_sim_parameters.time_step;
    }
}
        

void Simulation::reset(SimulationParams sim_params){m_sim_parameters = sim_params; reset();}
void Simulation::increaseTimeMultiplier()
{
    m_time_multiplier++;
    std::cout << "Simulation: Time Multiplier Increased (x" << m_time_multiplier << ")" << std::endl;
}
void Simulation::decreaseTimeMultiplier()
{
    if (m_time_multiplier > 1)
    {
        m_time_multiplier--;
        std::cout << "Simulation: Time Multiplier Decreased (x" << m_time_multiplier << ")" << std::endl;
    }
}
void Simulation::setTimeMultiplier(unsigned int multiplier){m_time_multiplier = static_cast<int>(multiplier);}
void Simulation::increaseZoom()
{
    if (m_view_size > 25){m_view_size -= 25;}
    std::cout << "Simulation: Zoom Increased (" << m_view_size << "m)" << std::endl;
}
void Simulation::decreaseZoom()
{
    if (m_view_size < 400){m_view_size += 25;}
    std::cout << "Simulation: Zoom Decreased (" << m_view_size << "m)" << std::endl;
}
void Simulation::togglePauseSimulation()
{
    m_is_paused = !m_is_paused;
    std::cout << "Simulation: Paused (" << (m_is_paused?"True":"False") << ")" << std::endl;
}
bool Simulation::isPaused(){return m_is_paused;}
bool Simulation::isRunning(){return m_is_running;}
