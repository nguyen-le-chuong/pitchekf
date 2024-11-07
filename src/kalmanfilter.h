#ifndef INCLUDE_AKFSFSIM_KALMANFILTER_H
#define INCLUDE_AKFSFSIM_KALMANFILTER_H

#include <vector>
#include <Eigen/Dense>

#include "car.h"
#include "sensors.h"

using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector4d;

using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::Matrix3d;

class KalmanFilterBase
{
    public:

        KalmanFilterBase():m_initialised(false){}
        virtual ~KalmanFilterBase(){}
        void reset(){
            VectorXd initialState(3);
            initialState << 0, 0, 1;
            setState(initialState);
            m_initialised = true;
            MatrixXd cov = MatrixXd::Identity(3, 3) * 0.01;
            setCovariance(cov);}
        bool isInitialised() const {return m_initialised;}

    protected:
    
        VectorXd getState() const {return m_state;}
        MatrixXd getCovariance()const {return m_covariance;}
        void setState(const VectorXd& state ) {m_state = state; m_initialised = true;}
        void setCovariance(const MatrixXd& cov ){m_covariance = cov;}

    private:
        bool m_initialised;
        VectorXd m_state;
        MatrixXd m_covariance;
};

class KalmanFilterRoadSlopeBase
{
    public:

        KalmanFilterRoadSlopeBase():m_initialised(false){}
        virtual ~KalmanFilterRoadSlopeBase(){}
        void reset(){
            VectorXd initialState(3);
            initialState << 0, 0, 9.8;
            setState(initialState);
            m_initialised = true;
            MatrixXd cov = MatrixXd::Identity(3, 3) * 0.01;
            setCovariance(cov);}
        bool isInitialised() const {return m_initialised;}

    protected:
    
        VectorXd getState() const {return m_state;}
        MatrixXd getCovariance()const {return m_covariance;}
        void setState(const VectorXd& state ) {m_state = state; m_initialised = true;}
        void setCovariance(const MatrixXd& cov ){m_covariance = cov;}

    private:
        bool m_initialised;
        VectorXd m_state;
        MatrixXd m_covariance;
};

class KalmanFilter : public KalmanFilterBase
{
    public:

        VehicleState getVehicleState();
        Matrix2d getVehicleStatePositionCovariance();

        void predictionStep(double dt);
        void predictionStep(GyroMeasurement gyro, double dt);
        void measurementStep1(AccelMeasurement accel, GyroMeasurement gyro, double v_t);
        void measurementStep2();

};


class KalmanFilterRoadSlope : public KalmanFilterRoadSlopeBase
{
    public:
        SlopeState getVehicleState();
        Matrix2d getVehicleStatePositionCovariance();

        void predictionStep(double dt);
        void measurementStep(AccelMeasurement accel, GyroMeasurement gyro, double v_t, double dt);
};



#endif  // INCLUDE_AKFSFSIM_KALMANFILTER_H