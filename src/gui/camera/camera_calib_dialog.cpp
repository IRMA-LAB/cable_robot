/**
 * @file camera_calib_dialog.cpp
 * @author Simone Comari
 * @date 17 Jul 2019
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
  connect(&app_, SIGNAL(calibrationFailed()), this, SLOT(handleCalibrationFailure()));
  connect(&app_, SIGNAL(calibrationSuccess(CameraParams)), this,
          SLOT(handleCalibrationSuccess(CameraParams)));
  connect(&app_, SIGNAL(augmentedFrameAvailable(cv::Mat)), this,
          SLOT(frwAugmentedFrame(cv::Mat)));
  connect(&app_, SIGNAL(printToQConsole(QString)), this,
          SLOT(frwPrintToQConsole(QString)));
}

CameraCalibDialog::~CameraCalibDialog() { delete ui; }

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
  this->show();
  emit calibrationStatusChanged(OFF);
}

void CameraCalibDialog::handleCalibrationSuccess(const CameraParams& params)
{
  emit calibrationStatusChanged(OFF);
  QMessageBox::StandardButton reply = QMessageBox::question(
    this, "Camera Info",
    "Camera calibration complete!\nDo you want to use these camera parameters now?",
    QMessageBox::Yes | QMessageBox::No);
  if (reply == QMessageBox::Yes)
    emit cameraParamsReady(params);
  else
    this->show();
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

void CameraCalibDialog::on_pushButton_newCalib_clicked()
{
  settings_win_.show();
  this->hide();
}

void CameraCalibDialog::on_pushButton_load_clicked()
{
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
