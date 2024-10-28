#ifndef INCLUDE_AKFSFSIM_CAR_H
#define INCLUDE_AKFSFSIM_CAR_H

#include <queue>
#include <cmath>
#include "utils.h"

struct VehicleState
{
    double R31, R32, R33;
    double pitch, roll;
    VehicleState():R31(0.0), R32(0.0), R33(1.0), pitch(0.0), roll(0.0) {}
    VehicleState(double setR31, double setR32, double setR33, double setPitch, double setRoll):R31(setR31), R32(setR32), R33(setR33), pitch(setPitch), roll(setRoll) {}
    // VehicleState(double setX, double setY, double setPsi, double setV, double setPsiDot, double setSteering):x(setX), y(setY), psi(setPsi), V(setV), yaw_rate(setPsiDot),steering(setSteering) {}
};

struct WheelState {
    double h_rear, h_front;
    double pitch;
    WheelState(): h_rear(0.0), h_front(0.0), pitch(0.0) {}
    WheelState(double setRear, double setFront, double setPitch): h_rear(setRear), h_front(setFront), pitch(setPitch) {}
};

class BicycleMotion
{
    public:

        BicycleMotion():m_initial_state(WheelState(0,0,0)),m_wheel_base(4.0),m_max_velocity(28.0),m_max_acceleration(2.0),m_max_steering(0.8) {reset();}
        BicycleMotion(double h_rear, double h_front, double pitch):m_initial_state(WheelState(h_rear, h_front, pitch)),m_wheel_base(4.0),m_max_velocity(28.0),m_max_acceleration(2.0),m_max_steering(0.8){reset();}

        void reset()
        {
            m_current_state = m_initial_state;
            // m_steering_command = m_initial_state.steering;
            // m_velocity_command = m_initial_state.V;
        }
        void reset(WheelState state)
        {
            m_initial_state = state;
            reset();
        }

        void update(double h_rear, double h_front)
        {
            // double cosPsi = cos(m_current_state.psi);
            // double sinPsi = sin(m_current_state.psi);
            // double x = m_current_state.x + m_current_state.V * cosPsi * dt;
            // double y = m_current_state.y + m_current_state.V * sinPsi * dt;

            // double accel = m_velocity_command - m_current_state.V;
            // if (accel > m_max_acceleration) {accel = m_max_acceleration;}
            // if (accel < -m_max_acceleration) {accel = -m_max_acceleration;}

            // double steer = m_steering_command;
            // if (steer > m_max_steering) {steer = m_max_steering;}
            // if (steer < -m_max_steering) {steer = -m_max_steering;}

            // double vel = m_current_state.V + accel * dt;
            // if (vel > m_max_velocity) {vel = m_max_velocity;}
            // if (vel < -m_max_velocity) {vel = -m_max_velocity;}

            // double psi_dot = m_current_state.V*steer/m_wheel_base;
            // double psi = wrapAngle(m_current_state.psi + psi_dot* dt);
            // m_current_state = VehicleState(x,y,psi,vel,psi_dot,steer);
            double delta_h = abs(h_front - h_rear);
            double pitch = atan2(delta_h, m_wheel_base);
            m_current_state = WheelState(h_rear, h_front, pitch);
        }

        // void setSteeringCmd(double steer){m_steering_command = steer;}
        // void setVelocityCmd(double accel){m_velocity_command = accel;}
        WheelState getWheelState() const {return m_current_state;}

    private:

        WheelState m_current_state;
        WheelState m_initial_state;

        double m_steering_command;
        double m_velocity_command;

        double m_wheel_base;

        double m_max_velocity;
        double m_max_acceleration;
        double m_max_steering;

};

class Car
{
    public:

        Car():m_vehicle_model() //,m_current_command(nullptr)
        {
            // Create Display Geometry
            m_car_lines_body = {{2,-1},{2,1},{-2,1},{-2,-1},{2,-1}};
            m_marker_lines = {{{0.5,0.5},{-0.5,-0.5}}, {{0.5,-0.5},{-0.5,0.5}}, {{0,0},{3.5,0}}};
            m_wheel_lines = {{-0.6,0.3},{0.6, 0.3},{0.6, -0.3},{-0.6, -0.3},{-0.6, 0.3}};
            m_wheel_fl_offset = Eigen::Vector2d(2, -1.6);
            m_wheel_fr_offset = Eigen::Vector2d(2, 1.6);
            m_wheel_rl_offset = Eigen::Vector2d(-2, -1.6);
            m_wheel_rr_offset = Eigen::Vector2d(-2, 1.6);
        }

        void reset(double h_front, double h_rear, double pitch)
        {
            m_vehicle_model.reset(WheelState(h_front, h_rear, pitch));
            // while (!m_vehicle_commands.empty()){m_vehicle_commands.pop();}
            // m_current_command = nullptr;
        }

        // void addVehicleCommand(MotionCommandBase* cmd)
        // {
        //     if (cmd != nullptr){m_vehicle_commands.push(cmd);}
        // }

        WheelState getWheelState() const {return m_vehicle_model.getWheelState();}

        bool update(double h_front, double h_rear)
        {
            // Update Command
            // if(m_current_command == nullptr && !m_vehicle_commands.empty())
            // {
            //     m_current_command = m_vehicle_commands.front();
            //     m_vehicle_commands.pop();
            //     m_current_command->startCommand(time, m_vehicle_model.getVehicleState());
            // }

            // Run Command
            // if (m_current_command != nullptr)
            // {
            //     bool cmd_complete = m_current_command->update(time, dt, m_vehicle_model.getVehicleState());
            //     m_vehicle_model.setSteeringCmd(m_current_command->getSteeringCommand());
            //     m_vehicle_model.setVelocityCmd(m_current_command->getVelocityCommand());
            //     if(cmd_complete){m_current_command = nullptr;}
            // }
            // else
            // {
            //     m_vehicle_model.setSteeringCmd(0.0);
            //     m_vehicle_model.setVelocityCmd(0.0);
            // }

            // Update Vehicle
            m_vehicle_model.update(h_rear, h_front);

            return true;
        }

    private:

        BicycleMotion m_vehicle_model;

        std::vector<Eigen::Vector2d> m_car_lines_body;
        std::vector<Eigen::Vector2d> m_wheel_lines;
        std::vector<std::vector<Eigen::Vector2d>> m_marker_lines;
        Eigen::Vector2d m_wheel_fl_offset, m_wheel_fr_offset, m_wheel_rl_offset, m_wheel_rr_offset;
};


#endif  // INCLUDE_AKFSFSIM_CAR_H