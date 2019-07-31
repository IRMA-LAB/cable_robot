/**
 * @file manual_control_dialog.cpp
 * @author Simone Comari
 * @date 03 Jul 2019
 * @brief This file includes definitions of class present in manual_control_dialog.h.
 */

#include "gui/apps/manual_control_dialog.h"
#include "ui_manual_control_dialog.h"

ManualControlDialog::ManualControlDialog(QWidget* parent, CableRobot* robot)
  : QDialog(parent), ui(new Ui::ManualControlDialog), robot_ptr_(robot), app_(this, robot)
{
  ui->setupUi(this);
  setAttribute(Qt::WA_DeleteOnClose);
  ui->horizontalLayout_display->addWidget(&traj_display_, 1);

  connect(robot_ptr_, SIGNAL(printToQConsole(QString)), this,
          SLOT(appendText2Browser(QString)), Qt::ConnectionType::QueuedConnection);
  connect(&app_, SIGNAL(printToQConsole(QString)), this,
          SLOT(appendText2Browser(QString)), Qt::ConnectionType::DirectConnection);

  // Setup timer to update actual platform position
  connect(&actual_pos_timer_, SIGNAL(timeout()), this, SLOT(updateActualXYZ()));
  actual_pos_timer_.start(kTimerPeriodMsec_);

  // Init target values
  resetTargetXYZ();
}

ManualControlDialog::~ManualControlDialog()
{
  disconnect(robot_ptr_, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));
  disconnect(&app_, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));
  delete ui;
  CLOG(INFO, "event") << "Manual control dialog closed";
}

//--------- Private slots -----------------------------------------------------------//

void ManualControlDialog::appendText2Browser(const QString& text)
{
  if (text.contains("warning", Qt::CaseSensitivity::CaseInsensitive))
  {
    CLOG(WARNING, "browser") << text;
    ui->textBrowser_logs->append(
      QString("<span style='color: orange'>%1</span>").arg(text));
  }
  else if (text.contains("error", Qt::CaseSensitivity::CaseInsensitive))
  {
    CLOG(ERROR, "browser") << text;
    ui->textBrowser_logs->append(QString("<span style='color: red'>%1</span>").arg(text));
  }
  else
  {
    CLOG(INFO, "browser") << text;
    ui->textBrowser_logs->append(text);
  }
}

void ManualControlDialog::updateActualXYZ()
{
  grabnum::Vector3d actual_pos = app_.getActualPos();
  ui->lcdNumber_x->display(actual_pos(Axis::X) * 1000);
  ui->lcdNumber_y->display(actual_pos(Axis::Y) * 1000);
  ui->lcdNumber_z->display(actual_pos(Axis::Z) * 1000);
}

//--------- Private GUI slots -------------------------------------------------------//

void ManualControlDialog::on_spinBox_x_valueChanged(int x_coord)
{
  app_.setTarget(Axis::X, x_coord);
}

void ManualControlDialog::on_spinBox_y_valueChanged(int y_coord)
{
  app_.setTarget(Axis::Y, y_coord);
}

void ManualControlDialog::on_spinBox_z_valueChanged(int z_coord)
{
  app_.setTarget(Axis::Z, z_coord);
}

void ManualControlDialog::on_pushButton_reset_clicked()
{
  CLOG(TRACE, "event");
  app_.resetTarget();
  resetTargetXYZ();
}

void ManualControlDialog::on_pushButton_return_clicked()
{
  CLOG(TRACE, "event");
  app_.resetTarget();
  close();
}

//--------- Private slots -----------------------------------------------------------//

void ManualControlDialog::resetTargetXYZ()
{
  grabnum::Vector3d actual_pos = app_.getActualPos();
  ui->spinBox_x->setValue(static_cast<int>(actual_pos(Axis::X) * 1000));
  ui->spinBox_x->setValue(static_cast<int>(actual_pos(Axis::Y) * 1000));
  ui->spinBox_x->setValue(static_cast<int>(actual_pos(Axis::Z) * 1000));
}
