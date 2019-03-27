#ifndef CABLE_ROBOT_DEBUG_SINGLE_DRIVE_SYSID_H
#define CABLE_ROBOT_DEBUG_SINGLE_DRIVE_SYSID_H

#include <fstream>

#include "ctrl/controller_singledrive.h"
#include "robot/cablerobot.h"

class SingleDriveSysID: public QObject
{
  Q_OBJECT
 public:
  SingleDriveSysID(QObject* parent, CableRobot* robot, ControllerSingleDrive* controller);
  ~SingleDriveSysID();

  void start(const double cable_len);

 private slots:
  void logData();
  void stopLogging();

 private:
  CableRobot* robot_;
  ControllerSingleDrive* controller_;
  id_t motor_id_;

  static const std::string kTrajFilepath_;
  static constexpr size_t kTrajLength_   = 30001;
  static constexpr int kLogIntervalMsec_ = 10;
  QTimer* log_timer_                     = NULL;

  std::vector<double> computeTrajectory(const double cable_len);
};

#endif // CABLE_ROBOT_DEBUG_SINGLE_DRIVE_SYSID_H
