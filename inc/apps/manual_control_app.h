#ifndef MANUAL_CONTROL_APP_H
#define MANUAL_CONTROL_APP_H

#include <QObject>
#include <QTextStream>

#include "matrix.h"

#include "ctrl/controller_joints_pvt.h"
#include "robot/cablerobot.h"

enum Coordinates: uchar
{
  X = 1,
  Y,
  Z
};

class ManualControlApp: public QObject
{
  Q_OBJECT

 public:
  explicit ManualControlApp(QObject* parent, CableRobot* robot = nullptr);
  ~ManualControlApp();

  const grabnum::Vector3d& getActualPos() const;

  void setTarget(const Coordinates coord, const double value);
  void resetTarget();

 signals:
  /**
   * @brief Signal including a message to any QConsole, for instance a QTextBrowser.
   */
  void printToQConsole(const QString&) const;

 private:
  CableRobot* robot_ptr_ = nullptr;
  grabnum::Vector3d target_pos_;
  grabnum::Vector3d actual_pos_;
};

#endif // MANUAL_CONTROL_APP_H
