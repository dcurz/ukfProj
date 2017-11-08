#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

//Initializes Unscented Kalman filter
UKF::UKF() {
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = .57;

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
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7; 

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  ///* Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
   
  //sigma points matrix
  Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

  //predicted sigma points matrix
  Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
}

UKF::~UKF() {}

//Master function for executing a passthrough of the ukf
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  //First measurement state initialization
  if (!is_initialized_) {
    
    //case where first measurement is radar
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      
      //Convert radar from polar to cartesian coordinates to initialize state.
      x_<<meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]),
          meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]),
          sqrt(((meas_package.raw_measurements_[2] * cos(meas_package.raw_measurements_[1]))*
          (meas_package.raw_measurements_[2] * cos(meas_package.raw_measurements_[1]))) + 
          ((meas_package.raw_measurements_[2] * sin(meas_package.raw_measurements_[1]))*
          meas_package.raw_measurements_[2] * sin(meas_package.raw_measurements_[1]))),
          0,0;
    }

    //case where first measurement is laser
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      
      //Initialize state directly with laser measurements.
      x_<<meas_package.raw_measurements_[0], meas_package.raw_measurements_[1],0,0,0;
      
      //Correction for special case if near origin
      if (fabs(x_(0)) < .001 and fabs(x_(1)) < .001){
        x_(0) = .001;
        x_(1) = .001;       
      }
    }

    //Initialize covariance matrix
    P_ << 	1, 0, 0, 0, 0,
         		0, 1, 0, 0, 0,
         		0, 0, 1, 0, 0,
         		0, 0, 0, 1, 0,
         		0, 0, 0, 0, 1;

    //Define values of weights for use in determining mean values from sigma points
    double weight_0 = lambda_/(lambda_+n_aug_);
    weights_(0) = weight_0;
    for (int i=1; i<weights_.size(); i++) {  
      double weight = 0.5/(n_aug_+lambda_);
      weights_(i) = weight;
    }

    previous_timestamp_ = meas_package.timestamp_;
    
    is_initialized_ = true;
	}
    
	//Determine elapsed time
  float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
	previous_timestamp_ = meas_package.timestamp_;

	//Call prediction step function
  Prediction(dt);

  //Update step for radar or lidar
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
		UpdateRadar(meas_package);
	}
	if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
	  	UpdateLidar(meas_package);
	}
}

//*********************************************************************
//*********************************************************************
void UKF::Prediction(double dt) {
//*********************************************************************
//*********************************************************************

//*********************************************************************
//GENERATE SIGMA POINTS
//*********************************************************************

  //augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //augmented sigma points matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //augmented state covariance matrix
  MatrixXd P_aug = MatrixXd(7, 7);

  //populate augmented mean state - last 2 values are 0
  x_aug.fill(0.0);
  x_aug.head(5) = x_;

  //populate augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix for future calculations
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

//*********************************************************************
//PREDICT SIGMA POINTS
//*********************************************************************

  //run each sigma point through process model
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*dt) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*dt) );
    }
    else {
        px_p = p_x + v*dt*cos(yaw);
        py_p = p_y + v*dt*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*dt;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*dt*dt * cos(yaw);
    py_p = py_p + 0.5*nu_a*dt*dt * sin(yaw);
    v_p = v_p + nu_a*dt;

    yaw_p = yaw_p + 0.5*nu_yawdd*dt*dt;
    yawd_p = yawd_p + nu_yawdd*dt;

    //write predicted sigma point values into correct column for the given sima point (by iteration 'i')
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }

//*********************************************************************
//Generate predicted mean state and covariance matrices based on predicted sigma points
//*********************************************************************

  const double m_PI = 3.141592;

  //predicted state mean
  x_.fill(0.0);
  x_ = Xsig_pred * weights_;

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;
    //angle normalization
    while (x_diff(3)> m_PI) x_diff(3)-=2.*m_PI;
    while (x_diff(3)<-m_PI) x_diff(3)+=2.*m_PI;
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

//*********************************************************************
//*********************************************************************
void UKF::UpdateLidar(MeasurementPackage meas_package) {
//*********************************************************************
//*********************************************************************

//*********************************************************************
//Project sigma points into measurement space (for lidar)
//*********************************************************************

  //Establish laser measurement vector dimension
  int n_z = 2;

  //create matrix for representing sigma points defined in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);	

  //populate Zsig matrix
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
   //extract values for better readibility
   double p_x = Xsig_pred(0,i);
   double p_y = Xsig_pred(1,i);

   Zsig(0,i) = p_x;                  //px
   Zsig(1,i) = p_y;            			 //py
  }	

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  z_pred = Zsig * weights_;

  //measurement covariance matrix definition
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { 
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_laspx_*std_laspx_, 0,
          0, std_laspy_*std_laspy_;
          
  S = S + R;

//*********************************************************************
//Calculate cross correlation matrix and perform update step
//*********************************************************************

  //Define measurement vector
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
}	

//*********************************************************************
//*********************************************************************
void UKF::UpdateRadar(MeasurementPackage meas_package) {
//*********************************************************************
//*********************************************************************

//*********************************************************************
//Project sigma points into measurement space (for lidar), calculate mean pred measurement (z) and S matrix
//*********************************************************************
	
//Establish radar measurement vector dimension
  int n_z = 3;

  const double m_PI = 3.141592;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);	

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  

    // extract values for better readibility
    double p_x = Xsig_pred(0,i);
    double p_y = Xsig_pred(1,i);
    double v  = Xsig_pred(2,i);
    double yaw = Xsig_pred(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  z_pred = Zsig * weights_;

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> m_PI) z_diff(1)-=2.*m_PI;
    while (z_diff(1)<-m_PI) z_diff(1)+=2.*m_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S = S + R;

//*********************************************************************
//Calculate cross correlation matrix and perform update step
//*********************************************************************

  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> m_PI) z_diff(1)-=2.*m_PI;
    while (z_diff(1)<-m_PI) z_diff(1)+=2.*m_PI;

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;
    //angle normalization
    while (x_diff(3)> m_PI) x_diff(3)-=2.*m_PI;
    while (x_diff(3)<-m_PI) x_diff(3)+=2.*m_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> m_PI) z_diff(1)-=2.*m_PI;
  while (z_diff(1)<-m_PI) z_diff(1)+=2.*m_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
}