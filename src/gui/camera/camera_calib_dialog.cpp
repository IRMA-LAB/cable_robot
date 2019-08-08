/**
 * @file camera_calib_dialog.cpp
 * @author Simone Comari
 * @date 22 Jul 2019
 * @brief This file includes definitions of class present in camera_calib_dialog.h.
 */

#include "gui/camera/camera_calib_dialog.h"
#include "ui_camera_calib_dialog.h"

const QString CameraCalibDialog::kDefaultCalibFile_ =
  QString(SRCDIR) + "resources/default_calib_params.json";

CameraCalibDialog::CameraCalibDialog(QWidget* parent, const CameraParams default_params)
  : QDialog(parent), ui(new Ui::CameraCalibDialog), settings_win_(this), app_(this),
    default_camera_params_(default_params)
{
  ui->setupUi(this);

  connect(&settings_win_, SIGNAL(cameraCalibSettings(CameraCalibSettings)), this,
          SLOT(startCalibration(CameraCalibSettings)));
  connect(&settings_win_, SIGNAL(cancelClicked()), this, SLOT(show()));

  connect(&app_, SIGNAL(calibrationFailed()), this, SLOT(handleCalibrationFailure()));
  connect(&app_, SIGNAL(calibrationSuccess(CameraParams)), this,
          SLOT(handleCalibrationSuccess(CameraParams)));
  connect(&app_, SIGNAL(augmentedFrameAvailable(cv::Mat)), this,
          SLOT(frwAugmentedFrame(cv::Mat)));
  connect(&app_, SIGNAL(printToQConsole(QString)), this,
          SLOT(frwPrintToQConsole(QString)));
}

CameraCalibDialog::~CameraCalibDialog()
{
  disconnect(&settings_win_, SIGNAL(cameraCalibSettings(CameraCalibSettings)), this,
             SLOT(startCalibration(CameraCalibSettings)));
  disconnect(&settings_win_, SIGNAL(cancelClicked()), this, SLOT(show()));

  disconnect(&app_, SIGNAL(calibrationFailed()), this, SLOT(handleCalibrationFailure()));
  disconnect(&app_, SIGNAL(calibrationSuccess(CameraParams)), this,
             SLOT(handleCalibrationSuccess(CameraParams)));
  disconnect(&app_, SIGNAL(augmentedFrameAvailable(cv::Mat)), this,
             SLOT(frwAugmentedFrame(cv::Mat)));
  disconnect(&app_, SIGNAL(printToQConsole(QString)), this,
             SLOT(frwPrintToQConsole(QString)));
  delete ui;
}

//--------- Public slots  ------------------------------------------------------------//

void CameraCalibDialog::setNewVideoFrame(const cv::Mat& frame)
{
  app_.setNewFrame(frame);
}

//--------- Private slots ------------------------------------------------------------//

void CameraCalibDialog::startCalibration(const CameraCalibSettings& settings)
{
  app_.start(settings);
  app_.show();
  emit calibrationStatusChanged(ON);
}

void CameraCalibDialog::handleCalibrationFailure()
{
  QMessageBox::warning(this, "Calibration Error",
                       "Calibration failed or interrupted!\nPlease try again.");
  this->setEnabled(true);
  emit calibrationStatusChanged(OFF);
}

void CameraCalibDialog::handleCalibrationSuccess(const CameraParams& params)
{
  emit calibrationStatusChanged(OFF);
  emit cameraParamsReady(params);
}

void CameraCalibDialog::frwAugmentedFrame(const cv::Mat& augmented_frame) const
{
  emit augmentedFrameAvailable(augmented_frame);
}

void CameraCalibDialog::frwPrintToQConsole(const QString& msg) const
{
  emit printToQConsole(msg);
}

//--------- Private GUI slots --------------------------------------------------------//

void CameraCalibDialog::closeEvent(QCloseEvent* event)
{
  if (settings_win_.isVisible())
    settings_win_.close();
  if (app_.isVisible())
    app_.close();
  event->accept();
}

void CameraCalibDialog::on_pushButton_newCalib_clicked() { settings_win_.show(); }

void CameraCalibDialog::on_pushButton_load_clicked()
{
  if (settings_win_.isVisible() || app_.isVisible())
    return;

  CLOG(TRACE, "event");
  QString calib_file = QFileDialog::getOpenFileName(
    this, tr("Load Camera Parameters"), tr("../.."), tr("Camera Parameters (*.json)"));
  if (calib_file.isEmpty())
  {
    CLOG(ERROR, "event") << "Calibration file name is empty";
    QMessageBox::warning(this, "File Error", "File name is empty!");
    return;
  }
  if (!parseCalibFile(calib_file))
  {
    CLOG(ERROR, "event") << "Calibration file name is not valid";
    QMessageBox::warning(this, "File Error", "File is not valid!");
    return;
  }
  emit cameraParamsReady(camera_params_);
}

void CameraCalibDialog::on_pushButton_loadDefault_clicked()
{
  if (settings_win_.isVisible() || app_.isVisible())
    return;

  if (default_camera_params_.isEmpty())
    parseCalibFile(kDefaultCalibFile_);
  else
  {
    camera_params_ = default_camera_params_;
    emit printToQConsole("Using default camera parameters");
  }
  emit cameraParamsReady(camera_params_);
}

//--------- Private functions --------------------------------------------------------//

bool CameraCalibDialog::parseCalibFile(const QString& filepath)
{
  // Open file
  CLOG(INFO, "event") << "Parsing file '" << filepath << "'...";

  if (!CameraParamsJsonParser::decodeJson(camera_params_, filepath.toStdString()))
  {
    CLOG(ERROR, "event") << "Missing, invalid or incomplete calibration parameter:"
                         << "camera_matrix";
    QMessageBox::warning(this, "File Error",
                         "Missing, invalid or incomplete calibration parameter: "
                         "dist_coeff");
    return false;
  }
  emit printToQConsole("Using calibration parameters from '" + filepath + "'");
  return true;
}
