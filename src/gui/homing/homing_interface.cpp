/**
 * @file homing_interface.cpp
 * @author Simone Comari
 * @date 02 Mar 2020
 * @brief This file includes definitions of classes present in homing_interface.h.
 */

#include "gui/homing/homing_interface.h"
#include "ui_homing_interface.h"

HomingInterface::HomingInterface(QWidget* parent, CableRobot* robot)
  : QDialog(parent), ui(new Ui::HomingInterface), robot_ptr_(robot), ext_close_cmd_(false)
{
  ui->setupUi(this);
}

HomingInterface::~HomingInterface() { delete ui; }

//--------- Public functions --------------------------------------------------------//

void HomingInterface::extClose()
{
  ext_close_cmd_ = true;
  close();
}

//--------- Protected slots ---------------------------------------------------------//

void HomingInterface::enableOkButton() { ui->buttonBox->addButton(QDialogButtonBox::Ok); }

//--------- Protected functions -----------------------------------------------------//

bool HomingInterface::rejectedExitRoutine(const bool /*force_exit = false*/)
{
  return true;
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
  // "OK" button
  CLOG(TRACE, "event");
  if (acceptedExitRoutine())
  {
    emit homingSuccess();
    hide();
  }
}

void HomingInterface::on_buttonBox_rejected()
{
  // "Cancel" button
  show(); // on "Cancel" clicked widget hides automaticaly otherwise
  CLOG(TRACE, "event");
  if (rejectedExitRoutine())
  {
    emit homingFailed();
    hide();
  }
}
