#include "ukf.h"
#include "Eigen/Dense"

#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  // State dimension
  n_x_ = 5;

  // Dimension of the augmented state
  n_aug_ = 7;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd::Zero(5);

  // initial covariance matrix
  P_ = MatrixXd::Zero(5, 5);
  P_(0, 0) = 1;
  P_(1, 1) = 1;

  std::cout << "P= " << P_ << std::endl;

  // Initializa predicted sigma points
  Xsig_pred_ = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);

  is_initialized_ = false;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 5; 

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 4; 
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  // define spreading parameter
  lambda_ = 3 - n_aug_;
  
  // set vector for weights
  weights_ = VectorXd::Zero(2*n_aug_+1);
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i=1; i<2*n_aug_+1; ++i) {  
    weights_(i) = 0.5/(lambda_+n_aug_);
  }

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  // Check if we wan tto use the sensor and which kind of measurement package we received
  if(meas_package.sensor_type_ == MeasurementPackage::LASER and !use_laser_)
    return;
  
  if(meas_package.sensor_type_ == MeasurementPackage::RADAR and !use_radar_)
    return;

  // If the state x has not been initialized, we can use the first measurement
  if(!is_initialized_)
  {
    x_.fill(0.0);
    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      // meas_package.raw_measurements_ : [p_x, p_y]
      x_(0) = meas_package.raw_measurements_[0]; // p_x
      x_(1) = meas_package.raw_measurements_[1]; // p_y
    }
    else
    {
      // meas_package.raw_measurements_ : [rho, phi, rho_dot]
      x_(0) = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]); // p_x = rho * cos(phi)
      x_(1) = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]); // p_y = rho * sin(phi)
      double vx = meas_package.raw_measurements_[2] * cos(meas_package.raw_measurements_[1]); // vx = rho_dot * cos(phi)
      double vy = meas_package.raw_measurements_[2] * sin(meas_package.raw_measurements_[1]); // vy = rho_dot * sin(phi)
      x_(2) = sqrt(vx * vx + vy * vy); // v
    }

    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
    
    return;
  }

  double delta_t = (static_cast<double>(meas_package.timestamp_) - static_cast<double>(time_us_)) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  // Execute state prediction
  Prediction(delta_t);

  // Select the update procedure according to the sensor
  if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    UpdateLidar(meas_package);
  else
    UpdateRadar(meas_package);
}

void UKF::Prediction(double delta_t) {

  // Create and initialize augmented mean vector
  VectorXd x_aug = VectorXd::Zero(7);
  x_aug.head(5) = x_;

  // Create and initialize augmented state covariance
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;
  // Create square root matrix of P_aug
  MatrixXd L = P_aug.llt().matrixL();

  // Create augmented sigma points matrix
  MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; ++i) {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  // Predict sigma poins
  for(int i=0; i<2*n_aug_+1; ++i)
  {
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    double px_p, py_p;

    // Avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // Add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // Write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  // Compute the predicted process mean 
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // Compute the predicted process covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // Angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  // set measurement dimension, lidar can measure p_x and p_y
  int n_z = 2;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd::Zero(n_z, 2 * n_aug_ + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z,n_z);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // measurement model
    Zsig(0,i) = Xsig_pred_(0,i); // p_x
    Zsig(1,i) = Xsig_pred_(1,i); // p_y
  }

  // mean predicted measurement
  for (int i=0; i < 2*n_aug_+1; ++i) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd::Zero(n_z,n_z);
  R(0,0) = std_laspx_*std_laspx_;
  R(1,1) = std_laspy_*std_laspy_;

  S = S + R;

  // create vector for incoming radar measurement
  VectorXd z = VectorXd::Zero(n_z);
  z << meas_package.raw_measurements_(0), // p_x in m
       meas_package.raw_measurements_(1); // p_y in m

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  // calculate cross correlation matrix
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  
  // Calculate the radar NIS.
  double epsilon = z_diff.transpose() * S.inverse() * z_diff;
  // radar_nis.push_back(epsilon);

  // std::cout << "lidar NIS: " << epsilon << std::endl;

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  
  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd::Zero(n_z, 2 * n_aug_ + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z,n_z);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                       // r
    Zsig(1,i) = atan2(p_y,p_x);                                // phi
    Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   // r_dot
  }

  // mean predicted measurement
  for (int i=0; i < 2*n_aug_+1; ++i) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd::Zero(n_z,n_z);
  R(0,0) = std_radr_*std_radr_;
  R(1,1) = std_radphi_*std_radphi_;
  R(2,2) = std_radrd_*std_radrd_;

  S = S + R;

  // create vector for incoming radar measurement
  VectorXd z = VectorXd::Zero(n_z);
  z << meas_package.raw_measurements_(0), // rho in m
       meas_package.raw_measurements_(1), // phi in rad
       meas_package.raw_measurements_(2); // rho_dot in m/s

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  // calculate cross correlation matrix
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  // Calculate the radar NIS.
  double epsilon = z_diff.transpose() * S.inverse() * z_diff;
  // radar_nis.push_back(epsilon);

  // std::cout << "radar NIS: " << epsilon << std::endl;
}