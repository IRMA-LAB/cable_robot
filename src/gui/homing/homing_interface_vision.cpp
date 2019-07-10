/**
 * @file homing_interface_vision.cpp
 * @author Simone Comari
 * @date 09 Jul 2019
 * @brief This file includes definitions of classes present in homing_interface_vision.h.
 */

#include "gui/homing/homing_interface_vision.h"
#include "ui_homing_interface_vision.h"

HomingInterfaceVision::HomingInterfaceVision(QWidget* parent, CableRobot* robot)
  : HomingInterface(parent, robot), ui(new Ui::HomingInterfaceVision), app_(this, robot),
    ext_close_cmd_(false)
{
  ui->setupUi(this);

  // Setup first tab --> proprioceptive homing interface
  proprioceptive_widget_ = new HomingInterfaceProprioceptive(this, robot);
  ui->verticalLayout_step1->addWidget(proprioceptive_widget_);

  connect(proprioceptive_widget_, SIGNAL(destroyed()), this, SLOT(close()));
  connect(proprioceptive_widget_, SIGNAL(homingCompleted()), this,
          SLOT(enableVisionTab()));

  // Setup camera widget in second tab
  camera_widget_ = new CameraWidget();
  ui->verticalLayout_step2->insertWidget(1, camera_widget_);
  ui->verticalLayout_step2->setStretch(1, 2);
  ui->verticalLayout_step2->setStretch(3, 1);

  connect(camera_widget_, SIGNAL(printToQConsole(QString)), this,
          SLOT(appendText2Browser(QString)));
  connect(camera_widget_, SIGNAL(newFrameGrabbed(cv::Mat)), &app_,
          SLOT(setNewFrame(cv::Mat)));
  connect(camera_widget_, SIGNAL(calibParamsReady(CameraParams)), &app_,
          SLOT(setCameraParams(CameraParams)));

  connect(&app_, SIGNAL(printToQConsole(QString)), this,
          SLOT(appendText2Browser(QString)));

  // debug
  ext_close_cmd_ = false;
  enableVisionTab();
}

HomingInterfaceVision::~HomingInterfaceVision()
{
  disconnect(&app_, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));

  disconnect(proprioceptive_widget_, SIGNAL(destroyed()), this, SLOT(close()));
  disconnect(proprioceptive_widget_, SIGNAL(homingCompleted()), this,
             SLOT(enableVisionTab()));
  delete proprioceptive_widget_;

  disconnect(camera_widget_, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));
  disconnect(camera_widget_, SIGNAL(newFrameGrabbed(cv::Mat)), &app_,
             SLOT(setNewFrame(cv::Mat)));
  disconnect(camera_widget_, SIGNAL(calibParamsReady(CameraParams)), &app_,
             SLOT(setCameraParams(CameraParams)));
  camera_widget_->stopVideoStream();
  delete camera_widget_;

  delete ui;
  CLOG(INFO, "event") << "Homing interface vision closed";
}

//--------- Private slots -----------------------------------------------------------//

void HomingInterfaceVision::on_pushButton_move_clicked()
{
  CLOG(TRACE, "event");
  // TODO
  ui->pushButton_find->setEnabled(true);
}

void HomingInterfaceVision::on_pushButton_find_clicked()
{
  ui->pushButton_find->setChecked(false);
  if (ui->pushButton_find->text() == "Stop")
  {
    CLOG(TRACE, "event") << "Stop";
    app_.stop();
    ui->pushButton_find->setText("Find Platform Pose");
    ui->pushButton_apply->setDisabled(true);
  }
  else
  {
    CLOG(TRACE, "event") << "Find";
    if (!app_.isCameraCalibrated())
    {
      QMessageBox::information(this, "Camera not calibrated",
                               "Please calibrate the camera or load a calibration file.");
      return;
    }
    robot_ptr_->WaitUntilPlatformSteady(-1.0);
    app_.start();
    ui->pushButton_find->setText("Stop");
    ui->pushButton_apply->setEnabled(true);
  }
}

void HomingInterfaceVision::on_pushButton_apply_clicked()
{
  CLOG(TRACE, "event");
  if (!app_.isPoseReady())
    return;

  app_.stop();
  ui->pushButton_find->setText("Find Platform Pose");
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

void HomingInterfaceVision::enableVisionTab() { ui->tab_vision->setEnabled(true); }

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
