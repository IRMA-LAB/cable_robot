/**
 * @file homing_dialog.cpp
 * @author Simone Comari
 * @date 17 Jul 2019
 * @brief This file includes definitions of class present in homing_dialog.h.
 */

#include "gui/homing/homing_dialog.h"
#include "ui_homing_dialog.h"

HomingDialog::HomingDialog(QWidget* parent, CableRobot* robot,
                           const SensorsParams sensor_config)
  : QDialog(parent), ui(new Ui::HomingDialog), robot_ptr_(robot), homing_method_(NONE)
{
  ui->setupUi(this);
  for (auto sensor : sensor_config.activeSensors())
    if (sensor == SensorType::VISION)
    {
      ui->comboBox_homingMethod->addItem("Vision-based");
      vision_params_ = sensor_config.vision;
      break;
    }
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
  if (homing_method_ != ui->comboBox_homingMethod->currentIndex() ||
      interface_ == nullptr)
  {
    DeleteInterface();
    homing_method_ = ui->comboBox_homingMethod->currentIndex();
    switch (homing_method_)
    {
      case PROPRIOCEPTIVE:
        interface_ = new HomingInterfaceProprioceptive(parentWidget(), robot_ptr_);
        break;
      case VISION:
        interface_ =
          new HomingInterfaceVision(parentWidget(), robot_ptr_, vision_params_);
        break;
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
  if (interface_ != nullptr)
  {
    interface_->Close();
    disconnect(interface_, SIGNAL(homingSuccess()), this, SLOT(fwdHomingSuccess()));
    disconnect(interface_, SIGNAL(homingFailed()), this, SLOT(fwdHomingFailed()));
    delete interface_;
    interface_ = nullptr;
  }
}
