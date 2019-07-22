/**
 * @file homing_interface_vision.cpp
 * @author Simone Comari
 * @date 18 Jul 2019
 * @brief This file includes definitions of classes present in homing_interface_vision.h.
 */

#include "gui/homing/homing_interface_vision.h"
#include "ui_homing_interface.h"
#include "ui_homing_interface_vision.h"

//------------------------------------------------------------------------------------//
//--------- HomingInterfaceVisionWidget class ----------------------------------------//
//------------------------------------------------------------------------------------//

HomingInterfaceVisionWidget::HomingInterfaceVisionWidget(QWidget* parent,
                                                         CableRobot* robot,
                                                         const VisionParams vision_config)
  : QWidget(parent), camera_widget(vision_config.camera),
    proprioceptive_widget(this, robot), ui(new Ui::HomingInterfaceVisionWidget),
    robot_ptr_(robot), app_(this, robot, vision_config)
{
  ui->setupUi(this);

  // Setup first tab --> proprioceptive homing interface
  ui->verticalLayout_step1->addWidget(&proprioceptive_widget);

  //  connect(proprioceptive_widget_, SIGNAL(destroyed()), this, SLOT(close()));
  connect(&(proprioceptive_widget.app), SIGNAL(homingComplete()), this,
          SLOT(enableVisionTab()));

  // Setup camera widget in second tab
  ui->verticalLayout_step2->insertWidget(1, &camera_widget);
  ui->verticalLayout_step2->setStretch(1, 2);
  ui->verticalLayout_step2->setStretch(3, 1);

  connect(&camera_widget, SIGNAL(printToQConsole(QString)), this,
          SLOT(appendText2Browser(QString)));
  connect(&camera_widget, SIGNAL(videoStreamStopped()), this, SLOT(stopEstimation()));
  connect(&camera_widget, SIGNAL(newFrameGrabbed(cv::Mat)), &app_,
          SLOT(setNewFrame(cv::Mat)));
  connect(&camera_widget, SIGNAL(calibParamsReady(CameraParams)), &app_,
          SLOT(setCameraParams(CameraParams)));

  connect(&app_, SIGNAL(printToQConsole(QString)), this,
          SLOT(appendText2Browser(QString)));
  connect(&app_, SIGNAL(frameReadyToShow(cv::Mat)), &camera_widget,
          SLOT(setAugmentedFrame(cv::Mat)));

  // debug
  enableVisionTab();
}

HomingInterfaceVisionWidget::~HomingInterfaceVisionWidget()
{
  disconnect(&app_, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));

  //  disconnect(proprioceptive_widget_, SIGNAL(destroyed()), this, SLOT(close()));
  disconnect(&(proprioceptive_widget.app), SIGNAL(homingComplete()), this,
             SLOT(enableVisionTab()));

  disconnect(&camera_widget, SIGNAL(printToQConsole(QString)), this,
             SLOT(appendText2Browser(QString)));
  disconnect(&camera_widget, SIGNAL(newFrameGrabbed(cv::Mat)), &app_,
             SLOT(setNewFrame(cv::Mat)));
  disconnect(&camera_widget, SIGNAL(calibParamsReady(CameraParams)), &app_,
             SLOT(setCameraParams(CameraParams)));
  camera_widget.stopVideoStream();

  delete ui;
  CLOG(INFO, "event") << "Homing interface vision closed";
}

//--------- Private GUI slots -------------------------------------------------------//

void HomingInterfaceVisionWidget::on_pushButton_move_clicked()
{
  CLOG(TRACE, "event");
  // TODO
  ui->pushButton_find->setEnabled(true);
  appendText2Browser("Moving to observation point...");
}

void HomingInterfaceVisionWidget::on_pushButton_find_clicked()
{
  ui->pushButton_find->setChecked(false);
  if (!camera_widget.isStreaming())
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
    camera_widget.enableStreamType();
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
    camera_widget.changeStreamType(VideoStreamType::AUGMENTED);
    camera_widget.enableStreamType(false);
    appendText2Browser("Waiting for platform to be steady...");
    robot_ptr_->WaitUntilPlatformSteady(-1.0);
    appendText2Browser("Looking for chessboard pose...");
    app_.start();
    ui->pushButton_find->setText("Stop");
    ui->pushButton_apply->setEnabled(true);
  }
}

void HomingInterfaceVisionWidget::on_pushButton_apply_clicked()
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

//--------- Private slots -----------------------------------------------------------//

void HomingInterfaceVisionWidget::enableVisionTab() { ui->tab_vision->setEnabled(true); }

void HomingInterfaceVisionWidget::appendText2Browser(const QString& text)
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

void HomingInterfaceVisionWidget::stopEstimation()
{
  if (ui->pushButton_find->text() == "Stop")
  {
    app_.stop();
    ui->pushButton_find->setText("Find Platform Pose");
    ui->pushButton_apply->setDisabled(true);
    camera_widget.enableStreamType();
  }
}

//------------------------------------------------------------------------------------//
//--------- HomingInterfaceVision class ----------------------------------------------//
//------------------------------------------------------------------------------------//

HomingInterfaceVision::HomingInterfaceVision(QWidget* parent, CableRobot* robot,
                                             const VisionParams vision_config)
  : HomingInterface(parent, robot), widget_(this, robot, vision_config)
{
  ui->verticalLayout->insertWidget(0, &widget_);
  connect(&(widget_.proprioceptive_widget.app), SIGNAL(homingComplete()), this,
          SLOT(enableOkButton()));
}

HomingInterfaceVision::~HomingInterfaceVision()
{
  disconnect(&(widget_.proprioceptive_widget.app), SIGNAL(homingComplete()), this,
             SLOT(enableOkButton()));
}

//--------- Private function --------------------------------------------------------//

bool HomingInterfaceVision::rejectedExitRoutine(const bool)
{
  widget_.stopEstimation();
  widget_.camera_widget.stopVideoStream();
  return true;
}
