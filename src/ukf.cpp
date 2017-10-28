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
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

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

  ///* State dimension
  int n_x_ = 5;

  ///* Augmented state dimension
  int n_aug_ = 7; 

  ///* Sigma point spreading parameter
  double lambda_ = 3 - n_aug;

  ///* Weights of sigma points
  VectorXd weights = VectorXd(2*n_aug+1);
   
  double weight_0 = lambda/(lambda+n_aug);
  
  weights(0) = weight_0;
  
  for (int i=1; i<2*n_aug+1; i++) {  
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;

  //sigma points matrix
  MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);

}



UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    //cout << "EKF: " << endl;
    //ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      x_<<measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]),
          measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]),
          sqrt(((measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]))*
          (measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]))) + 
          ((measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]))*
          measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]))),
          0,0;


    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_<<measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1],0,0,0;
    }

    // Initialize covariance matrix
    ukf_.P_ = MatrixXd(5, 5);
    ukf_.P_ << 	1, 0, 0, 0,0,
         		0, 1, 0, 0, 0,
         		0, 0, 100, 0, 0,
         		0, 0, 0, 1, 0,
         		0, 0, 0, 0, 5;

    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

//STILL NEED TO DO CALCULATIONS TO PREP FOR AND THEN CALL PREDICTION AND UPDATE FUNCTIONS

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

//*********************************************************************
//SIGMA POINTS
//*********************************************************************

  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P;
  P_aug(5,5) = std_a*std_a;
  P_aug(6,6) = std_yawdd*std_yawdd;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda+n_aug) * L.col(i);
    Xsig_aug.col(i+1+n_aug) = x_aug - sqrt(lambda+n_aug) * L.col(i);
  }




}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
