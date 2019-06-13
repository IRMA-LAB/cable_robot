#ifndef MANUAL_CONTROL_DIALOG_H
#define MANUAL_CONTROL_DIALOG_H

#include <QDialog>

#include "apps/manual_control_app.h"

namespace Ui {
class ManualControlDialog;
}

class ManualControlDialog: public QDialog
{
  Q_OBJECT

 public:
  explicit ManualControlDialog(QWidget* parent, CableRobot* robot);
  ~ManualControlDialog();

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
  Ui::ManualControlDialog* ui;

  CableRobot* robot_ptr_ = nullptr;
  ManualControlApp app_;

  static constexpr uint kRtCycleMultiplier_ = 10; // logging T = cycle_time * multiplier
  bool logging_;
};

#endif // MANUAL_CONTROL_DIALOG_H
