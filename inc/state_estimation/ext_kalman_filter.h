#ifndef CABLE_ROBOT_STATE_ESTIMATION_H
#define CABLE_ROBOT_STATE_ESTIMATION_H

#include <QDebug>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>

#include "libcdpr/inc/types.h"
#include "libgrabrt/inc/clocks.h"

#include "utils/types.h"

#define STATES_NUM 6

using state_t = grabnum::VectorXd<STATES_NUM>;
using cov_t   = grabnum::MatrixXd<STATES_NUM, STATES_NUM>;

ENUM_CLASS(UpdateType, IMU, CAMERA, LOAD_CELLS, NONE)

class ExtKalmanFilter
{
 public:
  ExtKalmanFilter(const state_t& x_init);
  ExtKalmanFilter(const state_t& x_init, const QString& tuning_params_file);

  void getStateEstimate(state_t& state_estimate) const;

  void reset(const state_t& x_init);
  void reset(const state_t& x_init, const QString& tuning_params_file);

  void prediction(const vect<ActuatorStatus>& actuators_status);

  template <typename T> void update(const UpdateType meas_t, const T& meas);

 private:
  // Tuning parameters
  cov_t P_init_;
  cov_t Q_;
  grabnum::MatrixXd<1, 1> R_imu_;
  grabnum::MatrixXd<1, 1> R_camera_;
  grabnum::MatrixXd<1, 1> R_load_cells_;

  // Time-variant state distribution variables
  state_t x_hat_;
  state_t x_hat_apriori_;
  cov_t P_;
  cov_t P_apriori_;

  grabrt::Clock clock_;

  void predictState(const vect<ActuatorStatus>& actuators_status, const double time);

  void imuUpdate(const double meas);
  void cameraUpdate(const double meas);
  void loadCellsUpdate(const double meas);

  void setTuningParamsFromFile(const QString& filepath);
};

#endif // CABLE_ROBOT_STATE_ESTIMATION_H
