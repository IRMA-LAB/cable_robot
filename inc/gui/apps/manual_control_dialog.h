#ifndef MANUAL_CONTROL_DIALOG_H
#define MANUAL_CONTROL_DIALOG_H

#include <QDialog>
#include <QTimer>

#include "gui/apps/my3dscatterwidget.h"
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
  void updateActualXYZ();

 private slots:
  void on_pushButton_reset_clicked();

  void on_pushButton_return_clicked();

 private:
  static constexpr uint kTimerPeriodMsec_ = 100;

  Ui::ManualControlDialog* ui;
  My3DScatterWidget traj_display_;

  CableRobot* robot_ptr_ = nullptr;
  ManualControlApp app_;
  QTimer actual_pos_timer_;

  void resetTargetXYZ();
};

#endif // MANUAL_CONTROL_DIALOG_H
