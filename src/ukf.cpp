#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.25 * M_PI;
  
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

  // state dimensions
  n_x_   = 5;
  n_aug_ = 7;

  // lambda
  lambda_ = 3 - n_aug_;

  // weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_ = 1/(2*(lambda_ + n_aug_)) * weights_.setOnes(2*n_aug_+1);
  weights_(0) = lambda_/(lambda_ + n_aug_);

  // Sigma point prediction
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // NIS data file setup
  if (use_laser_) {
    nis_laser_file.open("NIS_laser.dat");
  }

  if (use_radar_) {
    nis_radar_file.open("NIS_radar.dat");
  }

}

UKF::~UKF() {

  if (use_laser_) {
    nis_laser_file.close();
  }

  if (use_radar_) {
    nis_radar_file.close();
  }

}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    x_ = VectorXd(5);
    x_ << 1, 1, 1, 1, 1;
    P_ = MatrixXd(5,5);
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;
    
    if (use_radar_ && (meas_package.sensor_type_ == MeasurementPackage::RADAR)) {
      float rho   = meas_package.raw_measurements_[0];
      float theta = meas_package.raw_measurements_[1];

      while (theta >  M_PI) { theta -= 2. * M_PI; };
      while (theta < -M_PI) { theta += 2. * M_PI; };
      
      // as we have no time difference, velocity is kept zero
      x_ << rho * cos(theta), rho * sin(theta), 4, 0, 0;
      P_(0,0) = 0.3 * cos(0.03);
      P_(1,1) = 0.3 * sin(0.03);
    }
    else if (use_laser_ && (meas_package.sensor_type_ == MeasurementPackage::LASER)) {
      x_ <<  meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 4, 0, 0;
      P_(0,0) = 0.15;
      P_(1,1) = 0.15;
    }
    
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;

    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds

  time_us_ = meas_package.timestamp_;

  Prediction(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/


  if (use_radar_ && (meas_package.sensor_type_ == MeasurementPackage::RADAR)) {
    UpdateRadar(meas_package);
  } else if (use_laser_ && (meas_package.sensor_type_ == MeasurementPackage::LASER)) {
    UpdateLidar(meas_package);
  }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  /***
   * Create augmented Sigma points
   ***/

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.setZero();
  x_aug.head(n_x_) = x_;
  //create augmented covariance matrix
  P_aug.setZero();
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_ * std_yawdd_;
  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; ++i) {
      Xsig_aug.col(1+i) = x_aug + sqrt(lambda_ + n_aug_)*A.col(i);
      Xsig_aug.col(n_aug_+1+i) = x_aug - sqrt(lambda_ + n_aug_)*A.col(i);
  }

  /***
   * Sigma point prediction
   ***/

  double vk;
  double phik;
  double dphik;
  double nua;
  double nuddphi;
  
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
      vk =      Xsig_aug(2,i);
      phik =    Xsig_aug(3,i);
      dphik =   Xsig_aug(4,i);
      nua =     Xsig_aug(5,i);
      nuddphi = Xsig_aug(6,i);
      
      if (Xsig_aug(4,i) != 0) {
          Xsig_pred_(0,i) = Xsig_aug(0,i) + vk/dphik*(sin(phik+dphik*delta_t) - sin(phik)) + 0.5*delta_t*delta_t*cos(phik)*nua;
          Xsig_pred_(1,i) = Xsig_aug(1,i) + vk/dphik*(-cos(phik+dphik*delta_t) + cos(phik)) + 0.5*delta_t*delta_t*sin(phik)*nua;
      }
      else
      {
          Xsig_pred_(0,i) = Xsig_aug(0,i) + delta_t*vk*cos(phik) + 0.5*delta_t*delta_t*cos(phik)*nua;
          Xsig_pred_(1,i) = Xsig_aug(1,i) + delta_t*vk*sin(phik) + 0.5*delta_t*delta_t*sin(phik)*nua;
      }
      Xsig_pred_(2,i) = Xsig_aug(2,i) + delta_t*nua;
      Xsig_pred_(3,i) = Xsig_aug(3,i) + delta_t*dphik + 0.5*delta_t*delta_t*nuddphi;
      Xsig_pred_(4,i) = Xsig_aug(4,i) + delta_t*nuddphi;
  }

  /***
   * Predict mean and covariance
   ***/

  //predict state mean
  x_ = weights_(0)*Xsig_pred_.col(0);
  for (int i = 1; i < 2*n_aug_ + 1; ++i) {
      x_ += weights_(1)*Xsig_pred_.col(i);
  }
  //predict state covariance matrix
  VectorXd xdiff(n_x_);
  P_ = weights_(0)*(Xsig_pred_.col(0) - x_)*(Xsig_pred_.col(0) - x_).transpose();
  for (int i = 1; i < 2*n_aug_ + 1; ++i) {
      xdiff = Xsig_pred_.col(i) - x_;
      while (xdiff(3) >  M_PI) {xdiff(3) -= 2.*M_PI;}
      while (xdiff(3) < -M_PI) {xdiff(3) += 2.*M_PI;}
      P_ += weights_(i)*(xdiff)*(xdiff).transpose();
  }

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

  //set measurement dimension for lidar (px, py)
  int n_z = 2;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  //set measurement vector
  VectorXd z = VectorXd(n_z);
  z = meas_package.raw_measurements_;
  
  /***
   * Transform Sigma points into measurement space, 
   * calc. mean predicted measurement, 
   * calc. predicted measurement covariance
   ***/
  
  //transform sigma points into measurement space
  double px;
  double py;
  double vk;
  double phi;
  double dphi;
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
      Zsig(0,i) = Xsig_pred_(0,i);
      Zsig(1,i) = Xsig_pred_(1,i);
  }
  //calculate mean predicted measurement
  z_pred = weights_(0)*Zsig.col(0);
  for (int i = 1; i < 2*n_aug_ + 1; ++i) {
      z_pred += weights_(i)*Zsig.col(i);
  }
  //calculate measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2*n_aug_ + 1; ++i) {
      VectorXd zdiff = Zsig.col(i) - z_pred;
      S += weights_(i)*zdiff*zdiff.transpose();
  }
  S(0,0) += std_laspx_ * std_laspx_;
  S(1,1) += std_laspy_ * std_laspy_;


  /***
   * Update state and covariance
   ***/

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();


  /*
   * Calculate NIS
   ***/
  double epsilon;
  epsilon = z_diff.transpose() * S.inverse() * z_diff;

  nis_laser_file << time_us_ << " " << epsilon << endl;

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  //set measurement dimension for radar (r, phi, r_dot)
  int n_z = 3;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  //set measurement vector
  VectorXd z = VectorXd(n_z);
  z = meas_package.raw_measurements_;
  
  /***
   * Transform Sigma points into measurement space, 
   * calc. mean predicted measurement, 
   * calc. predicted measurement covariance
   ***/

  //transform sigma points into measurement space
  double px;
  double py;
  double vk;
  double phi;
  double dphi;
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
      px = Xsig_pred_(0,i);
      py = Xsig_pred_(1,i);
      vk = Xsig_pred_(2,i);
      phi = Xsig_pred_(3,i);
      dphi = Xsig_pred_(4,i);
      Zsig(0,i) = sqrt(px*px + py*py);
      Zsig(1,i) = atan2(py,px);
      Zsig(2,i) = vk*(px*cos(phi) + py*sin(phi))/Zsig(0,i);
  }
  //calculate mean predicted measurement
  z_pred = weights_(0)*Zsig.col(0);
  for (int i = 1; i < 2*n_aug_ + 1; ++i) {
      z_pred += weights_(i)*Zsig.col(i);
  }
  //calculate measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2*n_aug_ + 1; ++i) {
      VectorXd zdiff = Zsig.col(i) - z_pred;
      while (zdiff(1) >  M_PI) {zdiff(1) -= 2.*M_PI;}
      while (zdiff(1) < -M_PI) {zdiff(1) += 2.*M_PI;}
      
      S += weights_(i)*zdiff*zdiff.transpose();
  }
  S(0,0) += std_radr_ * std_radr_;
  S(1,1) += std_radphi_ * std_radphi_;
  S(2,2) += std_radrd_ * std_radrd_;


  /***
   * Update state and covariance
   ***/

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1) >  M_PI) {z_diff(1) -= 2.*M_PI; }
    while (z_diff(1) < -M_PI) {z_diff(1) += 2.*M_PI; }

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3) >  M_PI) {x_diff(3) -= 2.*M_PI; }
    while (x_diff(3) < -M_PI) {x_diff(3) += 2.*M_PI; }

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();


  /*
   * Calculate NIS
   ***/
  double epsilon;
  epsilon = z_diff.transpose() * S.inverse() * z_diff;

  nis_radar_file << time_us_ << " " << epsilon << endl;

}
