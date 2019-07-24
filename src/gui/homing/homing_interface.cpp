/**
 * @file homing_interface.cpp
 * @author Simone Comari
 * @date 23 Jul 2019
 * @brief This file includes definitions of classes present in homing_interface.h.
 */

#include "gui/homing/homing_interface.h"
#include "ui_homing_interface.h"

HomingInterface::HomingInterface(QWidget* parent, CableRobot* robot)
  : QDialog(parent), ui(new Ui::HomingInterface), robot_ptr_(robot), ext_close_cmd_(false)
{
  ui->setupUi(this);
  // debug
  ext_close_cmd_ = true;
}

HomingInterface::~HomingInterface() { delete ui; }

//--------- Public functions --------------------------------------------------------//

void HomingInterface::extClose()
{
  ext_close_cmd_ = true;
  close();
}

//--------- Protected slots ---------------------------------------------------------//

void HomingInterface::enableOkButton()
{
  ui->buttonBox->addButton(QDialogButtonBox::Ok);
}

//--------- Private GUI slots -------------------------------------------------------//

void HomingInterface::closeEvent(QCloseEvent* event)
{
  if (ext_close_cmd_)
  {
    ext_close_cmd_ = false;
    rejectedExitRoutine(true);
    event->accept();
  }
  else
  {
    event->ignore();
    // This becomes like user hit Cancel button.
    on_buttonBox_rejected();
  }
}

void HomingInterface::on_buttonBox_accepted()
{
  CLOG(TRACE, "event");
  if (acceptedExitRoutine())
  {
    emit homingSuccess();
    hide();
  }
}

void HomingInterface::on_buttonBox_rejected()
{
  CLOG(TRACE, "event");
  if (rejectedExitRoutine())
  {
    emit homingFailed();
    hide();
  }
}
