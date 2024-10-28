#include "utils.h"

#include <cmath>
#include <numeric>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Core>

double wrapAngle(double angle)
{
	angle = fmod(angle, (2.0*M_PI));
	if (angle <= -M_PI){angle += (2.0*M_PI);}
	else if (angle > M_PI){angle -= (2.0*M_PI);}
	return angle;
}

double calculateMean(const std::vector<double>& dataset)
{
    if (dataset.empty()){return NAN;}
    double sum = std::accumulate(std::begin(dataset), std::end(dataset), 0.0);
    double mean =  sum / dataset.size();
    return mean;
}

double calculateRMSE(const std::vector<double>& dataset)
{
    double accum = 0.0;
    if (dataset.empty()){return 0.0;}
    std::for_each (std::begin(dataset), std::end(dataset), [&](const double d) {accum += (d * d);});
    double rmse = sqrt(accum / (dataset.size()));
    return rmse;
}

// std::vector<Eigen::Vector2d> generateEllipse(double x, double y, double sigma_xx, double sigma_yy, double sigma_xy, int num_points)
// {
//     Eigen::Matrix2d pos_cov;
//     pos_cov << sigma_xx, sigma_xy, sigma_xy, sigma_yy;
    
//     Eigen::JacobiSVD<Eigen::MatrixXd> svd(pos_cov, Eigen::ComputeThinU | Eigen::ComputeThinV);
//     Eigen::Matrix2d D = svd.matrixU() * Eigen::VectorXd(3.0 * svd.singularValues().array().sqrt()).asDiagonal();
    
//     auto theta = Eigen::ArrayXd::LinSpaced(num_points, 0, 2*M_PI);
//     Eigen::ArrayXXd theta_array(2,num_points);
//     theta_array.row(0) = theta.cos();
//     theta_array.row(1) = theta.sin();
//     Eigen::MatrixXd A = D*Eigen::MatrixXd(theta_array);

//     std::vector<Eigen::Vector2d> shape_body;
//     for (int i = 0; i < A.cols(); ++i){ shape_body.push_back(Eigen::Vector2d(A(0,i),A(1,i)));}
//     std::vector<Eigen::Vector2d> shape_world = offsetPoints(shape_body, Eigen::Vector2d(x,y));
    
//     return shape_world;
// }

// std::vector<Eigen::Vector2d> generateCircle(double x, double y, double radius, int num_points)
// {
//     auto theta = Eigen::ArrayXd::LinSpaced(num_points, 0, 2*M_PI);
//     std::vector<Eigen::Vector2d> circle;
//     for (int i = 0; i < theta.size(); ++i)
//     {circle.push_back(Eigen::Vector2d(x+radius*cos(theta(i)),y+radius*sin(theta(i))));}
//     return circle;
// }
