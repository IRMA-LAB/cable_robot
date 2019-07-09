/**
 * @file camera_calib_dialog.cpp
 * @author Simone Comari
 * @date 22 Mar 2019
 * @brief This file includes definitions of class present in camera_calib_dialog.h.
 */

#include "gui/camera/camera_calib_dialog.h"
#include "ui_camera_calib_dialog.h"

const QString CameraCalibDialog::kDefaultCalibFile_ =
  QString(SRCDIR) + "resources/default_calib_params.json";

CameraCalibDialog::CameraCalibDialog(QWidget* parent)
  : QDialog(parent), ui(new Ui::CameraCalibDialog), settings_win_(this), app_(this)
{
  ui->setupUi(this);

  connect(&settings_win_, SIGNAL(cameraCalibSettings(CameraCalibSettings)), this,
          SLOT(startCalibration(CameraCalibSettings)));
  connect(&app_, SIGNAL(calibrationFailed()), this, SLOT(handleCalibrationFailure()));
  connect(&app_, SIGNAL(calibrationSuccess(CameraParams)), this,
          SLOT(handleCalibrationSuccess(CameraParams)));
}

CameraCalibDialog::~CameraCalibDialog() { delete ui; }

//--------- Public slots  ------------------------------------------------------------//

void CameraCalibDialog::getNewVideoFrame(const cv::Mat& frame)
{
  app_.setNewFrame(frame);
}

//--------- Private slots ------------------------------------------------------------//

void CameraCalibDialog::startCalibration(const CameraCalibSettings& settings)
{
  app_.start(settings);
  app_.show();
}

void CameraCalibDialog::handleCalibrationFailure()
{
  QMessageBox::warning(this, "Calibration Error",
                       "Calibration failed or interrupted!\nPlease try again.");
  this->show();
}

void CameraCalibDialog::handleCalibrationSuccess(const CameraParams& params)
{
  QMessageBox::StandardButton reply = QMessageBox::question(
    this, "Camera Info",
    "Camera calibration complete!\nDo you want to use these camera parameters now?",
    QMessageBox::Yes | QMessageBox::No);
  if (reply == QMessageBox::Yes)
    emit cameraParamsReady(params);
  else
    this->show();
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
  parseCalibFile(kDefaultCalibFile_);
  emit cameraParamsReady(camera_params_);
}

//--------- Private functions --------------------------------------------------------//

bool CameraCalibDialog::parseCalibFile(const QString& filepath)
{
  // Open file
  CLOG(INFO, "event") << "Parsing file '" << filepath << "'...";

  // to verify that works
  CameraParams params;
  CameraParamsJsonParser prs;
  if (!prs.decodeJson(params, filepath.toStdString()))
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
