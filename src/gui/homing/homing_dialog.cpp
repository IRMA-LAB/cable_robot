/**
 * @file homing_dialog.cpp
 * @author Simone Comari
 * @date 07 Mar 2019
 * @brief This file includes definitions of class present in homing_dialog.h.
 */

#include "gui/homing/homing_dialog.h"
#include "ui_homing_dialog.h"


HomingDialog::HomingDialog(QWidget* parent, CableRobot* robot)
  : QDialog(parent), ui(new Ui::HomingDialog), robot_ptr_(robot), homing_method_(NONE)
{
  ui->setupUi(this);
}

HomingDialog::~HomingDialog()
{
  DeleteInterface();
  delete ui;
  CLOG(INFO, "event") << "Homing dialog closed";
}

//--------- Private GUI slots -------------------------------------------------------//

void HomingDialog::on_buttonBox_accepted()
{
  if (homing_method_ != ui->comboBox_homingMethod->currentIndex())
  {
    DeleteInterface();
    homing_method_ = ui->comboBox_homingMethod->currentIndex();
    switch (homing_method_)
    {
      case PROPRIOCEPTIVE:
        interface_ = new HomingInterfaceProprioceptive(parentWidget(), robot_ptr_);
        break;
      case VISION:
        interface_ = NULL;
        fwdHomingSuccess(); // TODO: replace with homing vision interface
        return;
      case FUSION:
        interface_ = NULL;
        fwdHomingSuccess(); // TODO: replace with homing fusion interface
        return;
    }
    connect(interface_, SIGNAL(homingSuccess()), this, SLOT(fwdHomingSuccess()));
    connect(interface_, SIGNAL(homingFailed()), this, SLOT(fwdHomingFailed()));
  }
  interface_->show();
  CLOG(INFO, "event") << "Prompt " << ui->comboBox_homingMethod->currentText()
                      << " homing interface";
  hide();
  CLOG(INFO, "event") << "Hide homing dialog";
}

void HomingDialog::on_buttonBox_rejected()
{
  fwdHomingFailed();
  hide();
  CLOG(INFO, "event") << "Hide homing dialog";
}

//--------- Private slots -----------------------------------------------------------//

void HomingDialog::fwdHomingFailed()
{
  CLOG(INFO, "event") << ui->comboBox_homingMethod->currentText() << " homing failed";
  emit homingFailed();
  emit enableMainGUI(false);
}

void HomingDialog::fwdHomingSuccess()
{
  CLOG(INFO, "event") << ui->comboBox_homingMethod->currentText() << " homing success";
  emit homingSuccess();
  emit enableMainGUI(true);
}

//--------- Private functions ------------------------------------------------------//

void HomingDialog::DeleteInterface()
{
  if (interface_ != NULL)
  {
    interface_->Close();
    disconnect(interface_, SIGNAL(homingSuccess()), this, SLOT(fwdHomingSuccess()));
    disconnect(interface_, SIGNAL(homingFailed()), this, SLOT(fwdHomingFailed()));
    delete interface_;
    interface_ = NULL;
  }
}
