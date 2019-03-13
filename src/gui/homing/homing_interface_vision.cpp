/**
 * @file homing_interface_vision.cpp
 * @author Simone Comari
 * @date 13 Mar 2019
 * @brief This file includes definitions of classes present in homing_interface_vision.h.
 */

#include "gui/homing/homing_interface_vision.h"
#include "ui_homing_interface_vision.h"

HomingInterfaceVision::HomingInterfaceVision(QWidget* parent, CableRobot* robot)
  : HomingInterface(parent, robot), ui(new Ui::HomingInterfaceVision)
{
  ui->setupUi(this);
}

HomingInterfaceVision::~HomingInterfaceVision()
{
  delete ui;
  CLOG(INFO, "event") << "Homing interface vision closed";
}

//--------- Private slots -----------------------------------------------------------//

void HomingInterfaceVision::on_pushButton_enable_clicked()
{
  bool robot_enabled = ui->pushButton_enable->text() == "Disable";
  CLOG(TRACE, "event") << (robot_enabled ? "DISABLE" : "ENABLE");
  ui->pushButton_enable->setChecked(false);
}

void HomingInterfaceVision::on_pushButton_clearFaults_clicked()
{
  CLOG(TRACE, "event");
  ui->pushButton_start->setChecked(false);
}

void HomingInterfaceVision::on_pushButton_start_clicked()
{
  CLOG(TRACE, "event");
  ui->pushButton_start->setChecked(false);
}

void HomingInterfaceVision::on_pushButton_cancel_clicked()
{
  CLOG(TRACE, "event");
  emit homingFailed();
  hide();
  CLOG(INFO, "event") << "Hide homing interface vision";
}

void HomingInterfaceVision::on_pushButton_done_clicked()
{
  CLOG(TRACE, "event");
  emit homingSuccess();
  hide();
}
