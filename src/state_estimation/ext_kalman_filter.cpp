#include "state_estimation/ext_kalman_filter.h"

ExtKalmanFilter::ExtKalmanFilter(const state_t& x_init)
{
  setTuningParamsFromFile(":/config/tuning.json"); // default
  reset(x_init);
}

ExtKalmanFilter::ExtKalmanFilter(const state_t& x_init, const QString& tuning_params_file)
{
  setTuningParamsFromFile(tuning_params_file);
  reset(x_init);
}

//--------- Public functions ---------------------------------------------------------//

void ExtKalmanFilter::getStateEstimate(state_t& state_estimate) const
{
  state_estimate = x_hat_;
}

void ExtKalmanFilter::reset(const state_t& x_init)
{
  x_hat_ = x_init;
  P_     = P_init_;
}

void ExtKalmanFilter::reset(const state_t& x_init, const QString& tuning_params_file)
{
  setTuningParamsFromFile(tuning_params_file);
  reset(x_init);
}

void ExtKalmanFilter::prediction(const vect<ActuatorStatus>& actuators_status)
{
  predictState(actuators_status, clock_.Elapsed());
  clock_.Reset();
}

template <typename T> void ExtKalmanFilter::update(const UpdateType meas_t, const T& meas)
{
  switch (meas_t)
  {
    case UpdateType::IMU:
      imuUpdate(static_cast<int>(meas));
      break;
    case UpdateType::CAMERA:
      cameraUpdate(static_cast<int>(meas));
      break;
    case UpdateType::LOAD_CELLS:
      loadCellsUpdate(static_cast<int>(meas));
      break;
    case UpdateType::NONE:
      break;
  }
}

//--------- Private functions --------------------------------------------------------//

void ExtKalmanFilter::predictState(const vect<ActuatorStatus>& actuators_status,
                                   const double time)
{
  // Non-linear motion model for a-priori state estimation
  x_hat_apriori_ = x_hat_;

  // System Jacobian for prediction of the Covariance Matrix
  cov_t A;

  // A-priori error covariance estimation
  P_apriori_ = A * P_ * A.Transpose() + Q_;
}

void ExtKalmanFilter::imuUpdate(const double meas)
{
  // Non-linear measurement model
  double z_hat_apriori = x_hat_apriori_(1);
  double z_k           = meas;

  // Measurement Jacobian for calculation of kalman gain
  grabnum::MatrixXd<1, STATES_NUM> H;

  // Innovation matrix
  const grabnum::MatrixXd<1, 1> Z = H * P_apriori_ * H.Transpose() + R_imu_;

  // Kalman filter
  const grabnum::MatrixXd<STATES_NUM, 1> K = P_apriori_ * H.Transpose() * Z; // Z.Inv()

  // A priori measurement estimation
  x_hat_ = x_hat_apriori_ + K * (z_k - z_hat_apriori);

  // Update error covariance matrix
  P_ = P_apriori_ - K * Z * K.Transpose();
}

void ExtKalmanFilter::cameraUpdate(const double meas)
{
  // Non-linear measurement model
  double z_hat_apriori = x_hat_apriori_(1);
  double z_k           = meas;

  // Measurement Jacobian for calculation of kalman gain
  grabnum::MatrixXd<1, STATES_NUM> H;

  // Innovation matrix
  const grabnum::MatrixXd<1, 1> Z = H * P_apriori_ * H.Transpose() + R_camera_;

  // Kalman filter
  const grabnum::MatrixXd<STATES_NUM, 1> K = P_apriori_ * H.Transpose() * Z; // Z.Inv()

  // A priori measurement estimation
  x_hat_ = x_hat_apriori_ + K * (z_k - z_hat_apriori);

  // Update error covariance matrix
  P_ = P_apriori_ - K * Z * K.Transpose();
}

void ExtKalmanFilter::loadCellsUpdate(const double meas)
{
  // Non-linear measurement model
  double z_hat_apriori = x_hat_apriori_(1);
  double z_k           = meas;

  // Measurement Jacobian for calculation of kalman gain
  grabnum::MatrixXd<1, STATES_NUM> H;

  // Innovation matrix
  const grabnum::MatrixXd<1, 1> Z = H * P_apriori_ * H.Transpose() + R_load_cells_;

  // Kalman filter
  const grabnum::MatrixXd<STATES_NUM, 1> K = P_apriori_ * H.Transpose() * Z; // Z.Inv()

  // A priori measurement estimation
  x_hat_ = x_hat_apriori_ + K * (z_k - z_hat_apriori);

  // Update error covariance matrix
  P_ = P_apriori_ - K * Z * K.Transpose();
}

void ExtKalmanFilter::setTuningParamsFromFile(const QString& filepath)
{
  QFile tuning_file(filepath);
  tuning_file.open(QIODevice::ReadOnly | QIODevice::Text);
  QString file_string = tuning_file.readAll();
  tuning_file.close();

  QJsonDocument json_doc = QJsonDocument::fromJson(file_string.toUtf8());
  QJsonObject root_obj   = json_doc.object();
  QVariantMap root_map   = root_obj.toVariantMap();

  QVariantList rows = root_map["R_imu"].toList();
  for (int i = 0; i < rows.size(); ++i)
  {
    QVariantList cols = rows[i].toList();
    for (int j = 0; j < cols.size(); ++j)
      R_imu_(i + 1, j + 1) = cols[j].toDouble();
  }

  rows = root_map["R_camera"].toList();
  for (int i = 0; i < rows.size(); ++i)
  {
    QVariantList cols = rows[i].toList();
    for (int j = 0; j < cols.size(); ++j)
      R_camera_(i + 1, j + 1) = cols[j].toDouble();
  }

  rows = root_map["R_load_cells"].toList();
  for (int i = 0; i < rows.size(); ++i)
  {
    QVariantList cols = rows[i].toList();
    for (int j = 0; j < cols.size(); ++j)
      R_load_cells_(i + 1, j + 1) = cols[j].toDouble();
  }

  rows = root_map["Q"].toList();
  for (int i = 0; i < rows.size(); ++i)
  {
    QVariantList cols = rows[i].toList();
    for (int j = 0; j < cols.size(); ++j)
      Q_(i + 1, j + 1) = cols[j].toDouble();
  }

  rows = root_map["P_init"].toList();
  for (int i = 0; i < rows.size(); ++i)
  {
    QVariantList cols = rows[i].toList();
    for (int j = 0; j < cols.size(); ++j)
      P_init_(i + 1, j + 1) = cols[j].toDouble();
  }
}
