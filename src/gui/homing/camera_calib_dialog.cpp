/**
 * @file camera_calib_dialog.cpp
 * @author Simone Comari
 * @date 22 Mar 2019
 * @brief This file includes definitions of class present in camera_calib_dialog.h.
 */

#include "gui/homing/camera_calib_dialog.h"
#include "ui_camera_calib_dialog.h"

const QString CameraCalibDialog::kDefaultCalibFile_ =
  QString(SRCDIR) + "resources/default_calib_params.json";

CameraCalibDialog::CameraCalibDialog(QWidget* parent)
  : QDialog(parent), ui(new Ui::CameraCalibDialog)
{
  ui->setupUi(this);
}

CameraCalibDialog::~CameraCalibDialog() { delete ui; }

//--------- Public slots  ------------------------------------------------------------//

void CameraCalibDialog::getNewVideoFrame(const cv::Mat& frame)
{
  mutex_.lock();
  frame_               = frame;
  new_frame_available_ = true;
  mutex_.unlock();
}

//--------- Private slots  -----------------------------------------------------------//

void CameraCalibDialog::on_pushButton_newCalib_clicked()
{
  // TODO...
}

void CameraCalibDialog::on_pushButton_load_clicked()
{
  CLOG(TRACE, "event");
  QString calib_file = QFileDialog::getOpenFileName(
    this, tr("Load Calibration File"), tr("../.."), tr("Calibration Files (*.json)"));
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
  emit calibParamsReady(calib_params_);
}

void CameraCalibDialog::on_pushButton_loadDefault_clicked()
{
  parseCalibFile(kDefaultCalibFile_);
  emit calibParamsReady(calib_params_);
}

//--------- Private functions --------------------------------------------------------//

bool CameraCalibDialog::parseCalibFile(const QString& filepath)
{
  // Open file
  CLOG(INFO, "event") << "Parsing file '" << filepath << "'...";
  std::ifstream ifile(filepath.toStdString());
  if (!ifile.is_open())
  {
    CLOG(ERROR, "event") << "Could not open calibration file";
    QMessageBox::warning(this, "File Error", "Could not open file '" + filepath + "'");
    return false;
  }

  // Parse JSON (generic) data
  json calib_results;
  ifile >> calib_results;
  ifile.close();

  // Fill home_data structure
  QString field;
  try
  {
    // TODO..
  }
  catch (json::type_error)
  {
    CLOG(ERROR, "event") << "Missing, invalid or incomplete calibration parameter:"
                         << field;
    QMessageBox::warning(this, "File Error",
                         "Missing, invalid or incomplete calibration parameter: " +
                           field);
    return false;
  }
  emit printToQConsole("Using calibration parameters from '" + filepath + "'");
  return true;
}
