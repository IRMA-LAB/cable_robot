#include "gui/homing/homing_dialog.h"
#include "ui_homing_dialog.h"

HomingDialog::HomingDialog(QWidget* parent, CableRobot* robot)
  : QDialog(parent), ui(new Ui::HomingDialog), robot_ptr_(robot)
{
  ui->setupUi(this);
}

HomingDialog::~HomingDialog()
{
  if (interface_ != NULL)
  {
    disconnect(interface_, SIGNAL(homingSuccess()), this, SLOT(HomingSuccessCb()));
    disconnect(interface_, SIGNAL(homingFailed()), this, SLOT(HomingFailedCb()));
    delete interface_;
  }
  delete ui;
  CLOG(INFO, "event") << "Homing dialog closed";
}

void HomingDialog::on_buttonBox_accepted()
{
  if (interface_ != NULL)
    delete interface_;

  switch (ui->comboBox_homingMethod->currentIndex())
  {
  case PROPRIOCEPTIVE:
    interface_ = new HomingInterfaceProprioceptive(parentWidget(), robot_ptr_);
    break;
  case VISION:
    interface_ = NULL;
    HomingSuccessCb(); // replace with homing vision interface
    return;
  case FUSION:
    interface_ = NULL;
    HomingSuccessCb(); // replace with homing fusion interface
    return;
  }
  connect(interface_, SIGNAL(homingSuccess()), this, SLOT(HomingSuccessCb()));
  connect(interface_, SIGNAL(homingFailed()), this, SLOT(HomingFailedCb()));
  interface_->show();
  CLOG(INFO, "event") << "Prompt homing interface "
                      << ui->comboBox_homingMethod->currentText();
  hide();
  CLOG(INFO, "event") << "Hide homing dialog";
}

void HomingDialog::HomingFailedCb()
{
  CLOG(INFO, "event") << ui->comboBox_homingMethod->currentText() << " homing failed";
  emit homingFailed();
  emit enableMainGUI(false);
  close();
}

void HomingDialog::HomingSuccessCb()
{
  CLOG(INFO, "event") << ui->comboBox_homingMethod->currentText() << " homing success";
  emit homingSuccess();
  emit enableMainGUI(true);
  close();
}
