#ifndef HOMING_DIALOG_H
#define HOMING_DIALOG_H

#include <QDialog>

#include "easylogging++.h"
#include "libcdpr/inc/types.h"

#include "gui/homing/homing_interface_proprioceptive.h"


namespace Ui {
class HomingDialog;
}

class HomingDialog: public QDialog
{
  Q_OBJECT

 public:
  HomingDialog(QWidget* parent, CableRobot* robot);
  ~HomingDialog();

 signals:
  void enableMainGUI(bool);
  void homingFailed();
  void homingSuccess();

 private slots:
  void on_buttonBox_accepted();
  void on_buttonBox_rejected();

 private slots:
  void fwdHomingFailed();
  void fwdHomingSuccess();

 private:
  Ui::HomingDialog* ui;

  enum HomingMethod
  {
    PROPRIOCEPTIVE,
    VISION,
    FUSION,
    NONE
  };

  const grabcdpr::Params* config_ptr_;
  HomingInterface* interface_ = NULL;
  CableRobot* robot_ptr_      = NULL;
  int homing_method_;

  void DeleteInterface();
};

#endif // HOMING_DIALOG_H
