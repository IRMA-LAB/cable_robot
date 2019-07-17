/**
 * @file homing_interface_vision.cpp
 * @author Simone Comari
 * @date 09 Jul 2019
 * @brief This file includes definitions of classes present in homing_interface_vision.h.
 */

#include "gui/homing/homing_interface_vision.h"
#include "ui_homing_interface_vision.h"

HomingInterfaceVision::HomingInterfaceVision(QWidget* parent, CableRobot* robot,
                                             const VisionParams vision_config)
  : HomingInterface(parent, robot), ui(new Ui::HomingInterfaceVision),
    app_(this, robot, vision_config), ext_close_cmd_(false)
{
  ui->setupUi(this);

  // Setup first tab --> proprioceptive homing interface
  proprioceptive_widget_ = new HomingInterfaceProprioceptive(this, robot);
  ui->verticalLayout_step1->addWidget(proprioceptive_widget_);

  connect(proprioceptive_widget_, SIGNAL(destroyed()), this, SLOT(close()));
  connect(proprioceptive_widget_, SIGNAL(homingCompleted()), this,
          SLOT(enableVisionTab()));

  // Setup camera widget in second tab
  camera_widget_ = new CameraWidget(vision_config.camera);
  ui->verticalLayout_step2->insertWidget(1, camera_widget_);
  ui->verticalLayout_step2->setStretch(1, 2);
  ui->verticalLayout_step2->setStretch(3, 1);

  connect(camera_widget_, SIGNAL(printToQConsole(QString)), this,
          SLOT(appendText2Browser(QString)));
  connect(camera_widget_, SIGNAL(videoStreamStopped()), this, SLOT(stopEstimation()));
  connect(camera_widget_, SIGNAL(newFrameGrabbed(cv::Mat)), &app_,
          SLOT(setNewFrame(cv::Mat)));
  connect(camera_widget_, SIGNAL(calibParamsReady(CameraParams)), &app_,
          SLOT(setCameraParams(CameraParams)));

  connect(&app_, SIGNAL(printToQConsole(QString)), this,
          SLOT(appendText2Browser(QString)));
  connect(&app_, SIGNAL(frameReadyToShow(cv::Mat)), camera_widget_,
          SLOT(setAugmentedFrame(cv::Mat)));

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
  appendText2Browser("Moving to observation point...");
}

void HomingInterfaceVision::on_pushButton_find_clicked()
{
  ui->pushButton_find->setChecked(false);
  if (!camera_widget_->isStreaming())
  {
    QMessageBox::information(this, "Video Stream Missing",
                             "Please start and calibrate the camera before proceeding.");
    return;
  }

  if (ui->pushButton_find->text() == "Stop")
  {
    CLOG(TRACE, "event") << "Stop";
    app_.stop();
    ui->pushButton_find->setText("Find Platform Pose");
    ui->pushButton_apply->setDisabled(true);
    camera_widget_->enableStreamType();
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
    camera_widget_->changeStreamType(VideoStreamType::AUGMENTED);
    camera_widget_->enableStreamType(false);
    appendText2Browser("Waiting for platform to be steady...");
    robot_ptr_->WaitUntilPlatformSteady(-1.0);
    appendText2Browser("Looking for chessboard pose...");
    app_.start();
    ui->pushButton_find->setText("Stop");
    ui->pushButton_apply->setEnabled(true);
  }
}

void HomingInterfaceVision::on_pushButton_apply_clicked()
{
  CLOG(TRACE, "event");
  if (!app_.isPoseReady())
  {
    appendText2Browser("WARNING: Chessboard pose not yet found. Please try again.");
    return;
  }

  stopEstimation();
  app_.applyPoseEstimate();
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

void HomingInterfaceVision::enableVisionTab() { ui->tab_vision->setEnabled(true); }

void HomingInterfaceVision::appendText2Browser(const QString& text)
{
  if (text.startsWith("warning", Qt::CaseSensitivity::CaseInsensitive))
  {
    CLOG(WARNING, "browser") << text;
    ui->textBrowser_logs->append(
      QString("<span style='color: orange'>%1</span>").arg(text));
  }
  else if (text.startsWith("error", Qt::CaseSensitivity::CaseInsensitive))
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

void HomingInterfaceVision::stopEstimation()
{
  if (ui->pushButton_find->text() == "Stop")
  {
    app_.stop();
    ui->pushButton_find->setText("Find Platform Pose");
    ui->pushButton_apply->setDisabled(true);
    camera_widget_->enableStreamType();
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
