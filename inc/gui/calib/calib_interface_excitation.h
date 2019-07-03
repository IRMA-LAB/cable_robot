#ifndef CABLE_ROBOT_CALIB_INTERFACE_EXCITATION_H
#define CABLE_ROBOT_CALIB_INTERFACE_EXCITATION_H

#include <QDialog>

#include "calib/calib_excitation.h"

namespace Ui {
class CalibInterfaceExcitation;
}

class CalibInterfaceExcitation: public QDialog
{
  Q_OBJECT

 public:
  explicit CalibInterfaceExcitation(QWidget* parent, CableRobot* robot, const vect<grabcdpr::ActuatorParams>& params);
  ~CalibInterfaceExcitation();

 private slots:
  void appendText2Browser(const QString& text);
  void handleStateChanged(const quint8& state);

 private slots:
  void on_pushButton_enable_clicked();
  void on_radioButton_torque_clicked();
  void on_radioButton_position_clicked();
  void on_pushButton_return_clicked();

  void on_pushButton_logging_clicked();

 private:
  Ui::CalibInterfaceExcitation* ui;

  CableRobot* robot_ptr_ = nullptr;
  CalibExcitation app_;
};

#endif // CABLE_ROBOT_CALIB_INTERFACE_EXCITATION_H
