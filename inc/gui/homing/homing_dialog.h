#ifndef HOMING_DIALOG_H
#define HOMING_DIALOG_H

#include <QDialog>

#include "libcdpr/inc/types.h"
#include "easylogging++.h"

#include "gui/homing/homing_interface_proprioceptive.h"

namespace Ui
{
class HomingDialog;
}

class HomingDialog : public QDialog
{
  Q_OBJECT

public:
  explicit HomingDialog(QWidget* parent, CableRobot* robot);
  ~HomingDialog();

signals:
  void enableMainGUI(bool);
  void homingFailed();
  void homingSuccess();

private slots:
  void on_buttonBox_accepted();
  void on_buttonBox_rejected() { HomingFailedCb(); }

  void HomingFailedCb();
  void HomingSuccessCb();

private:
  Ui::HomingDialog* ui;

  enum HomingMethod
  {
    PROPRIOCEPTIVE,
    VISION,
    FUSION
  };

  const grabcdpr::Params* config_ptr_;
  HomingInterface* interface_ = NULL;
  CableRobot* robot_ptr_ = NULL;
};

#endif // HOMING_DIALOG_H
