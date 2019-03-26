#ifndef SINGLE_DRIVE_SYSID_H
#define SINGLE_DRIVE_SYSID_H

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

  static constexpr size_t kTrajLength_   = 30001;
  static constexpr int kLogIntervalMsec_ = 10;
  QTimer* log_timer_                     = NULL;

  std::vector<double> ComputeTrajectory(const double cable_len);
};

#endif // SINGLE_DRIVE_SYSID_H
