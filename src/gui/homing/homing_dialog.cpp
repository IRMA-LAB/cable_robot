#include "gui/homing/homing_dialog.h"
#include "ui_homing_dialog.h"

HomingDialog::HomingDialog(QWidget* parent, const grabcdpr::Params* config)
  : QDialog(parent), ui(new Ui::HomingDialog), config_ptr_(config)
{
  ui->setupUi(this);
}

HomingDialog::~HomingDialog()
{
  if (interface_ != NULL)
  {
    disconnect(interface_, SIGNAL(HomingInterface::homingSuccess()), this,
               SLOT(HomingSuccessCb()));
    disconnect(interface_, SIGNAL(HomingInterface::homingFailed()), this,
               SLOT(HomingFailedCb()));
  }
  delete ui;
}

void HomingDialog::on_buttonBox_accepted()
{
  switch (ui->comboBox_homingMethod->currentIndex())
  {
  case PROPRIOCEPTIVE:
    interface_ = new HomingInterfaceProprioceptive(this, config_ptr_);
    break;
  case VISION:
    HomingSuccessCb(); // replace with homing vision interface
    break;
  case FUSION:
    HomingSuccessCb(); // replace with homing fusion interface
    break;
  }
  connect(interface_, SIGNAL(HomingInterface::homingSuccess()), this,
          SLOT(HomingSuccessCb()));
  connect(interface_, SIGNAL(HomingInterface::homingFailed()), this,
          SLOT(HomingFailedCb()));
  interface_->show();
  hide();
}

void HomingDialog::HomingFailedCb()
{
  emit homingFailed();
  emit enableMainGUI(false);
  close();
}

void HomingDialog::HomingSuccessCb()
{
  emit homingSuccess();
  emit enableMainGUI(true);
  close();
}
