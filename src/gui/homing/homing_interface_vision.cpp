/**
 * @file homing_interface_vision.cpp
 * @author Simone Comari
 * @date 22 Mar 2019
 * @brief This file includes definitions of classes present in homing_interface_vision.h.
 */

#include "gui/homing/homing_interface_vision.h"
#include "ui_homing_interface_vision.h"

HomingInterfaceVision::HomingInterfaceVision(QWidget* parent, CableRobot* robot)
  : HomingInterface(parent, robot), ui(new Ui::HomingInterfaceVision), app_(this, robot),
    ext_close_cmd_(false)
{
  ui->setupUi(this);

  camera_widget_ = new CameraWidget();
  ui->widgetLayout->insertWidget(1, camera_widget_);
  ui->widgetLayout->setStretch(1, 2);
  ui->widgetLayout->setStretch(3, 1);

  connect(camera_widget_, SIGNAL(printToQConsole(QString)), this,
          SLOT(appendText2Browser(QString)));
  connect(camera_widget_, SIGNAL(newFrameGrabbed(cv::Mat)), &app_,
          SLOT(getNewVideoFrame(cv::Mat)));
  connect(&app_, SIGNAL(printToQConsole(QString)), this,
          SLOT(appendText2Browser(QString)));
}

HomingInterfaceVision::~HomingInterfaceVision()
{
  disconnect(&app_, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));

  disconnect(camera_widget_, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));
  disconnect(camera_widget_, SIGNAL(newFrameGrabbed(cv::Mat)), &app_,
             SLOT(getNewVideoFrame(cv::Mat)));
  camera_widget_->stopVideoStream();
  delete camera_widget_;

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
  camera_widget_->stopVideoStream();
  hide();
  CLOG(INFO, "event") << "Hide homing interface vision";
}

void HomingInterfaceVision::on_pushButton_done_clicked()
{
  CLOG(TRACE, "event");
  emit homingSuccess();
  hide();
}

//--------- Private slots -----------------------------------------------------------//

void HomingInterfaceVision::appendText2Browser(const QString& text)
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

//--------- Private functions -------------------------------------------------------//

void HomingInterfaceVision::closeEvent(QCloseEvent* event)
{
  if (ext_close_cmd_)
  {
    ext_close_cmd_ = false;
    event->accept();
  }
  else
  {
    event->ignore();
    // This becomes like user hit Cancel button.
    ui->pushButton_cancel->click();
  }
}
