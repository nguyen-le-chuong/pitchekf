// ------------------------------------------------------------------------------- //
// Advanced Kalman Filtering and Sensor Fusion Course - Extended Kalman Filter
//
// ####### ANSWER FILE #######
//
// Usage:
// -Rename this file to "kalmanfilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "utils.h"

// -------------------------------------------------- //
// YOU CAN USE AND MODIFY THESE CONSTANTS HERE
constexpr double ACCEL_STD = 1.0;
constexpr double GYRO_STD = 0.01/180.0 * M_PI;
constexpr double INIT_VEL_STD = 10.0;
constexpr double INIT_PSI_STD = 45.0/180.0 * M_PI;
constexpr double GPS_POS_STD = 3.0;
constexpr double LIDAR_RANGE_STD = 3.0;
constexpr double LIDAR_THETA_STD = 0.02;
// -------------------------------------------------- //
MatrixXd R1 = Matrix2d::Identity() * GYRO_STD * INIT_VEL_STD + Matrix2d::Identity() * ACCEL_STD;
MatrixXd R2 = MatrixXd::Constant(1, 1, 0.01);

void KalmanFilter::predictionStep(GyroMeasurement gyro, double dt)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Extract the current state variables
        double R31 = state(0);
        double R32 = state(1);
        double R33 = state(2);


        double wx = gyro.wx;
        double wy = gyro.wy;
        double wz = gyro.wz;

        Matrix3d omega_skew;
        Matrix3d x_skew;

        omega_skew <<  0,   -wz,   wy,
                       wz,   0,   -wx,
                      -wy,  wx,    0;
        x_skew << 0,    -R33,   R32,
                  R33,  0,      -R31,
                  -R32, R31,    0;  
        Matrix3d F = Matrix3d::Identity() + omega_skew * dt;  // F matrix

        // Predict the new state
        state = F * state;

        Matrix3d sigma_G = GYRO_STD * Matrix3d::Identity();
        Matrix3d Q = dt * dt * x_skew * sigma_G * x_skew;


        cov = F * cov * F.transpose() + Q;

        setState(state);
        setCovariance(cov);
    }
}





void KalmanFilter::measurementStep1(AccelMeasurement accel, GyroMeasurement gyro, double vt)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        double R31 = state(0);
        double R32 = state(1);
        double R33 = state(2);

        double ay = accel.ay;          
        double az = accel.az;  

        double wx = gyro.wx;   
        double wy = gyro.wy;   
        double wz = gyro.wz;    

 
        double ay_centripetal = vt * wz;       
        double az_centripetal = -vt * wx;      

        double ay_corrected = ay - ay_centripetal;
        double az_corrected = az - az_centripetal;

        MatrixXd H1(2, 3);
        H1 << 0, 9.81, 0,
              0, 0, 9.81;

        Vector2d z1;
        z1 << ay_corrected, az_corrected;

        VectorXd y = z1 - H1 * state;

        MatrixXd S1 = H1 * cov * H1.transpose() + R1;
        MatrixXd K1 = cov * H1.transpose() * S1.inverse();

        state = state + K1 * y;

        MatrixXd I = MatrixXd::Identity(cov.rows(), cov.cols());
        cov = (I - K1 * H1) * cov;

        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilter::measurementStep2()
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        double R31 = state(0);
        double R32 = state(1);
        double R33 = state(2);

        MatrixXd H2(1, 3);
        H2 << R31, 0, 0;
        double z2 = sqrt(1 - R32 * R32 - R33 * R33);
        
        VectorXd result = H2 * state; // Result will be a vector
        double y = z2 - result(0); // Use the first element (or any other as needed)

        //double y = z2 - H2 * state;

        MatrixXd S2 = H2 * cov * H2.transpose() + R2;
        MatrixXd K2 = cov * H2.transpose() * S2.inverse();

        state = state + K2 * y;

        MatrixXd I = MatrixXd::Identity(cov.rows(), cov.cols());
        cov = (I - K2 * H2) * cov;

        setState(state);
        setCovariance(cov);
    }
}



Matrix2d KalmanFilter::getVehicleStatePositionCovariance()
{
    Matrix2d pos_cov = Matrix2d::Zero();
    MatrixXd cov = getCovariance();
    if (isInitialised() && cov.size() != 0){pos_cov << cov(0,0), cov(0,1), cov(1,0), cov(1,1);}
    return pos_cov;
}

VehicleState KalmanFilter::getVehicleState()
{
    if (isInitialised())
    {
        VectorXd state = getState(); // STATE VECTOR [R31,R32,R33]
        double roll = std::atan2(state[1], state[2]);
        double pitch = std::atan2(-state[0], (state[1]/std::sin(roll)));
        // pitch_degree = pitch*180.0 / 3.14;
        return VehicleState(state[0], state[1], state[2], pitch, roll);
    }
    return VehicleState();
}

void KalmanFilter::predictionStep(double dt){}
