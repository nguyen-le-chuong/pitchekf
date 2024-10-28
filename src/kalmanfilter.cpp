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
// void KalmanFilter::handleLidarMeasurements(const std::vector<LidarMeasurement>& dataset, const BeaconMap& map)
// {
//     // Assume No Correlation between the Measurements and Update Sequentially
//     for(const auto& meas : dataset) {handleLidarMeasurement(meas, map);}
// }

// void KalmanFilter::handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map)
// {
//     if (isInitialised())
//     {
//         VectorXd state = getState();
//         MatrixXd cov = getCovariance();

//         // Implement The Kalman Filter Update Step for the Lidar Measurements in the 
//         // section below.
//         // HINT: use the wrapAngle() function on angular values to always keep angle
//         // values within correct range, otherwise strange angle effects might be seen.
//         // HINT: You can use the constants: LIDAR_RANGE_STD, LIDAR_THETA_STD
//         // HINT: The mapped-matched beacon position can be accessed by the variables
//         // map_beacon.x and map_beacon.y
//         // ----------------------------------------------------------------------- //
//         // ENTER YOUR CODE HERE

//         BeaconData map_beacon = map.getBeaconWithId(meas.id); // Match Beacon with built in Data Association Id
//         if (meas.id != -1 && map_beacon.id != -1)
//         {           
//             // Measurement Vector
//             VectorXd z = Vector2d::Zero();
//             z << meas.range, meas.theta;

//             // Predicted Measurement Vector (Measurement Model)
//             VectorXd z_hat = Vector2d::Zero();
//             double delta_x = map_beacon.x - state[0];
//             double delta_y = map_beacon.y - state[1];
//             double zhat_range = sqrt(delta_x*delta_x + delta_y*delta_y);
//             double zhat_theta = wrapAngle(atan2(delta_y,delta_x) - state[2]);
//             z_hat << zhat_range, zhat_theta;

//             // Measurement Model Sensitivity Matrix
//             MatrixXd H = MatrixXd(2,4);
//             H << -delta_x/zhat_range,-delta_y/zhat_range,0,0,delta_y/zhat_range/zhat_range,-delta_x/zhat_range/zhat_range,-1,0;

//             // Generate Measurement Model Noise Covariance Matrix
//             MatrixXd R = Matrix2d::Zero();
//             R(0,0) = LIDAR_RANGE_STD*LIDAR_RANGE_STD;
//             R(1,1) = LIDAR_THETA_STD*LIDAR_THETA_STD;

//             VectorXd y = z - z_hat;
//             MatrixXd S = H * cov * H.transpose() + R;
//             MatrixXd K = cov*H.transpose()*S.inverse();

//             y(1) = wrapAngle(y(1)); // Wrap the Heading Innovation

//             state = state + K*y;
//             cov = (Matrix4d::Identity() - K*H) * cov;            
//         }
//         // ----------------------------------------------------------------------- //

//         setState(state);
//         setCovariance(cov);
//     }
// }
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

        // Wrap the angular state (R31, R32, R33) using wrapAngle if needed (similar to psi in the example)
        // Wrap state(2) for angle correction (if necessary based on your system's angle representation)

        // Set the updated state and covariance back
        setState(state);
        setCovariance(cov);
    }
}

// void KalmanFilter::predictionStep(GyroMeasurement gyro, double dt)
// {
//     if (isInitialised())
//     {
//         VectorXd state = getState();
//         MatrixXd cov = getCovariance();

//         // Implement The Kalman Filter Prediction Step for the system in the  
//         // section below.
//         // HINT: Assume the state vector has the form [PX, PY, PSI, V].
//         // HINT: Use the Gyroscope measurement as an input into the prediction step.
//         // HINT: You can use the constants: ACCEL_STD, GYRO_STD
//         // HINT: use the wrapAngle() function on angular values to always keep angle
//         // values within correct range, otherwise strange angle effects might be seen.
//         // ----------------------------------------------------------------------- //
//         // ENTER YOUR CODE HERE

//         double x = state(0);
//         double y = state(1);
//         double psi = state(2);
//         double V = state(3);

//         // Update State
//         double x_new = x + dt * V * cos(psi);
//         double y_new = y + dt * V * sin(psi);
//         double psi_new = wrapAngle(psi + dt * gyro.psi_dot);
//         double V_new = V;
//         state << x_new,y_new,psi_new,V_new;

//         // Generate F Matrix
//         MatrixXd F = Matrix4d::Zero();
//         F << 1,0,-dt*V*sin(psi),dt*cos(psi),0,1,dt*V*cos(psi),dt*sin(psi),0,0,1,0,0,0,0,1;

//         // Generate Q Matrix
//         MatrixXd Q = Matrix4d::Zero();
//         Q(2,2) = dt*dt*GYRO_STD*GYRO_STD;
//         Q(3,3) = dt*dt*ACCEL_STD*ACCEL_STD;

//         cov = F * cov * F.transpose() + Q;

//         // ----------------------------------------------------------------------- //

//         setState(state);
//         setCovariance(cov);
//     } 
// }



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

// void KalmanFilter::handleGPSMeasurement(GPSMeasurement meas)
// {
//     // All this code is the same as the LKF as the measurement model is linear
//     // so the UKF update state would just produce the same result.
//     if(isInitialised())
//     {
//         VectorXd state = getState();
//         MatrixXd cov = getCovariance();

//         VectorXd z = Vector2d::Zero();
//         MatrixXd H = MatrixXd(2,4);
//         MatrixXd R = Matrix2d::Zero();

//         z << meas.x,meas.y;
//         H << 1,0,0,0,0,1,0,0;
//         R(0,0) = GPS_POS_STD*GPS_POS_STD;
//         R(1,1) = GPS_POS_STD*GPS_POS_STD;

//         VectorXd z_hat = H * state;
//         VectorXd y = z - z_hat;
//         MatrixXd S = H * cov * H.transpose() + R;
//         MatrixXd K = cov*H.transpose()*S.inverse();

//         state = state + K*y;
//         cov = (Matrix4d::Identity() - K*H) * cov;

//         setState(state);
//         setCovariance(cov);
//     }
//     else
//     {
//         VectorXd state = Vector4d::Zero();
//         MatrixXd cov = Matrix4d::Zero();

//         state(0) = meas.x;
//         state(1) = meas.y;
//         cov(0,0) = GPS_POS_STD*GPS_POS_STD;
//         cov(1,1) = GPS_POS_STD*GPS_POS_STD;
//         cov(2,2) = INIT_PSI_STD*INIT_PSI_STD;
//         cov(3,3) = INIT_VEL_STD*INIT_VEL_STD;

//         setState(state);
//         setCovariance(cov);
//     }   
// }

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
        return VehicleState(state[0], state[1], state[2], pitch, roll);
    }
    return VehicleState();
}

void KalmanFilter::predictionStep(double dt){}
