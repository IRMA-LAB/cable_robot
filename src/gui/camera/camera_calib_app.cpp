/**
 * @file camera_calib_app.cpp
 * @author Simone Comari, Marco Caselli
 * @date 24 Jul 2019
 * @brief Implementation of classes declared in camera_calib_app.h
 */

#include "gui/camera/camera_calib_app.h"
#include "ui_camera_calib_app.h"

//------------------------------------------------------------------------------------//
//--------- WorkerThread class -------------------------------------------------------//
//------------------------------------------------------------------------------------//

WorkerThread::WorkerThread(const CameraCalibSettings& settings)
  : settings_(settings), calib_mode_(CAPTURING), stop_(false), new_frame_pending_(false)
{}

//--------- Public functions ---------------------------------------------------------//

void WorkerThread::setNewFrame(const cv::Mat& frame)
{
  if (frame.empty())
    return;

  mutex_.lock();
  latest_frame_      = frame;
  new_frame_pending_ = true;
  new_frame_available_.wakeAll();
  mutex_.unlock();
}

void WorkerThread::stop()
{
  mutex_.lock();
  stop_ = true;
  mutex_.unlock();
}

//--------- Private functions --------------------------------------------------------//

void WorkerThread::run()
{
  while (1)
  {
    mutex_.lock();
    // Check if stop command was sent
    if (stop_)
    {
      mutex_.unlock();
      emit calibrationFailed();
      return;
    }

    // Grab new frame
    if (!new_frame_pending_)
      new_frame_available_.wait(&mutex_);
    cv::Mat frame(latest_frame_);
    new_frame_pending_ = false;
    mutex_.unlock();

    // Process frame
    if (calibrationSamplesCollected(frame))
      break;
  }

  if (runCalibration())
    emit resultReady(camera_params_);
  else
    emit calibrationFailed();

  // Clear samples
  image_points_.clear();
}

bool WorkerThread::calibrationSamplesCollected(const cv::Mat& frame)
{
  if (storeValidFrame(frame))
    emit calibFrameCaptured(image_points_.size(), settings_.num_frames);

  if (image_points_.size() < settings_.num_frames)
    return false;

  return true;
}

bool WorkerThread::storeValidFrame(const cv::Mat& view)
{
  if (!findChessboard(view))
    return false;
  if (catchCalibrationSample(view))
    return true;
  return false;
}

bool WorkerThread::findChessboard(const cv::Mat& image)
{
  bool found = true;
  if (image.rows > 480)
  {
    cv::Mat shrinked;
    double scale = 360.0 / image.rows;
    cv::resize(image, shrinked, cv::Size(), scale, scale, cv::INTER_AREA);
    cv::Mat point_buf;
    found = cv::findChessboardCorners(shrinked, settings_.board_size, point_buf,
                                      settings_.chess_board_flags);
  }

  // Draw the corners.
  cv::Mat augmented_image = image.clone();

  if (found)
  {
    found = cv::findChessboardCorners(image, settings_.board_size, point_buf_,
                                      settings_.chess_board_flags);
    cv::drawChessboardCorners(augmented_image, settings_.board_size, cv::Mat(point_buf_),
                              found);
  }
  emit augmentedFrameAvailable(augmented_image);

  return found;
}

bool WorkerThread::catchCalibrationSample(const cv::Mat& image)
{
  static grabrt::Clock clock;

  image_size_ = image.size();
  if (calib_mode_ == DETECTION && clock.Elapsed() > settings_.delay)
    calib_mode_ = CAPTURING;

  if (calib_mode_ == CAPTURING)
  {
    cv::Mat view_gray;
    cvtColor(image, view_gray, cv::COLOR_BGR2GRAY);
    cornerSubPix(view_gray, point_buf_,
                 cv::Size(settings_.cor_sp_size, settings_.cor_sp_size),
                 cv::Size(settings_.zero_zone, settings_.zero_zone),
                 cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                                  settings_.max_counter, settings_.cor_sp_size));
    if (!image_points_.empty())
      if (!compareFrameAgainstPrev())
        return false;
    // Save sample
    image_points_.push_back(point_buf_);
    clock.Reset();
    calib_mode_ = DETECTION;
    return true;
  }
  return false;
}

bool WorkerThread::compareFrameAgainstPrev()
{
  static uint counter_x_ = 0;
  static uint counter_y_ = 0;
  static uint delta_x_   = 0;
  static uint delta_y_   = 0;
  static uint counter_   = 0;

  for (uint j = 0; j < uint(image_points_.size()); ++j)
  {
    for (uint i = 0; i < uint(settings_.board_size.width); ++i)
    {
      delta_x_ = delta_x_ + uint(std::abs(point_buf_[i].x - image_points_[j][i].x));
      if (counter_ < uint(settings_.board_size.height))
        delta_y_ =
          delta_y_ + int(std::abs(point_buf_[i + settings_.board_size.width].y -
                                  image_points_[j][i + settings_.board_size.width].y));
      counter_++;
    }
    if (delta_x_ > kErrorX_)
      counter_x_++;
    if (delta_y_ > kErrorY_)
      counter_y_++;
    delta_x_ = 0;
    delta_y_ = 0;
    counter_ = 0;
  }
  if ((counter_x_ >= uint(image_points_.size())) ||
      (counter_y_ >= uint(image_points_.size())))
  {
    counter_x_ = 0;
    counter_y_ = 0;
    return true;
  }
  counter_x_ = 0;
  counter_y_ = 0;
  return false;
}

bool WorkerThread::runCalibration()
{

  if (computeCameraParams())
  {
    calib_mode_ = CALIBRATED;
    return true;
  }
  return false;
}

bool WorkerThread::computeCameraParams()
{
  std::vector<cv::Mat> rvecs, tvecs;
  std::vector<float> reproj_errs;
  std::vector<cv::Point3f> new_obj_points;
  std::vector<std::vector<cv::Point3f>> object_points(1);
  for (int i = 0; i < settings_.board_size.height; ++i)
    for (int j = 0; j < settings_.board_size.width; ++j)
      object_points[0].push_back(
        cv::Point3f(j * settings_.square_size, i * settings_.square_size, 0));

  object_points[0][static_cast<size_t>(settings_.board_size.width - 1)].x =
    object_points[0][0].x + settings_.gridWidth();
  new_obj_points = object_points[0];

  object_points.resize(image_points_.size(), object_points[0]);

  // Find intrinsic and extrinsic camera parameters
  double rms;
  if (settings_.use_fisheye)
  {
    cv::Mat _rvecs, _tvecs;
    try
    {
      rms = cv::fisheye::calibrate(
        object_points, image_points_, image_size_, camera_params_.camera_matrix,
        camera_params_.dist_coeff, _rvecs, _tvecs, settings_.calibFlags());
    }
    catch (cv::Exception)
    {
      return false;
    }

    rvecs.reserve(static_cast<size_t>(_rvecs.rows));
    tvecs.reserve(static_cast<size_t>(_tvecs.rows));
    for (int i = 0; i < int(object_points.size()); i++)
    {
      rvecs.push_back(_rvecs.row(i));
      tvecs.push_back(_tvecs.row(i));
    }
  }
  else
  {
    int i_fixedPoint = -1;
    i_fixedPoint     = settings_.board_size.width - 1;
    rms = cv::calibrateCameraRO(object_points, image_points_, image_size_, i_fixedPoint,
                                camera_params_.camera_matrix, camera_params_.dist_coeff,
                                rvecs, tvecs, new_obj_points, settings_.calibFlags());
  }
  // rms is the overall RMS re-projection error
  emit printToQConsole(QString("Re-projection error RMS: %1").arg(rms));

  return checkRange(camera_params_.camera_matrix) &&
         checkRange(camera_params_.dist_coeff);
}

double WorkerThread::computeCalibReprojectionErr(
  const std::vector<std::vector<cv::Point3f>>& object_points,
  const std::vector<cv::Mat>& r_vecs, const std::vector<cv::Mat>& t_vecs,
  std::vector<float>& per_view_errors) const
{
  std::vector<cv::Point2f> image_points2;
  size_t total_points = 0;
  double total_err    = 0, err;
  per_view_errors.resize(object_points.size());

  for (size_t i = 0; i < object_points.size(); ++i)
  {
    // undistorted points coordinates
    if (settings_.use_fisheye)
    {
      cv::fisheye::projectPoints(object_points[i], image_points2, r_vecs[i], t_vecs[i],
                                 camera_params_.camera_matrix, camera_params_.dist_coeff);
    }
    else
    {
      cv::projectPoints(object_points[i], r_vecs[i], t_vecs[i],
                        camera_params_.camera_matrix, camera_params_.dist_coeff,
                        image_points2);
    }
    // compute relative difference norm between the observed feature points imagePoints
    // and the projected using camera_params_.camera_matrix and camera_params_.dist_coeff
    err = cv::norm(image_points_[i], image_points2, cv::NORM_L2);

    size_t n = object_points[i].size();
    // compute and store error for each view and store in a vector, every elements contain
    // reprojection error for image
    per_view_errors[i] = float(std::sqrt(err * err / n));
    total_err += err * err;
    total_points += n;
  }
  // root mean square error same as return of calibrateCameraRO
  return std::sqrt(total_err / total_points);
}

//------------------------------------------------------------------------------------//
//--------- CameraCalibApp class -----------------------------------------------------//
//------------------------------------------------------------------------------------//

CameraCalibApp::CameraCalibApp(QWidget* parent)
  : QDialog(parent), ui(new Ui::CameraCalibApp)
{
  ui->setupUi(this);
}

CameraCalibApp::~CameraCalibApp() { delete ui; }

//--------- Public functions ---------------------------------------------------------//

void CameraCalibApp::start(const CameraCalibSettings& settings)
{
  worker_thread_ = new WorkerThread(settings);
  connect(worker_thread_, &WorkerThread::calibStateUpdate, this,
          &CameraCalibApp::setProgressLabel);
  connect(worker_thread_, &WorkerThread::calibFrameCaptured, this,
          &CameraCalibApp::updateProgressBar);
  connect(worker_thread_, &WorkerThread::resultReady, this,
          &CameraCalibApp::handleResults, Qt::BlockingQueuedConnection);
  connect(worker_thread_, &WorkerThread::calibrationFailed, this,
          &CameraCalibApp::handleErrors);
  connect(worker_thread_, &WorkerThread::finished, this, &CameraCalibApp::workFinished);
  connect(worker_thread_, &WorkerThread::augmentedFrameAvailable, this,
          &CameraCalibApp::frwAugmentedFrame);
  connect(worker_thread_, &WorkerThread::printToQConsole, this,
          &CameraCalibApp::frwPrintToQConsole);
  worker_thread_->start();
}

void CameraCalibApp::setNewFrame(const cv::Mat& frame)
{
  if (worker_thread_ != nullptr && worker_thread_->isRunning())
    worker_thread_->setNewFrame(frame);
}

//--------- Private GUI slots --------------------------------------------------------//

void CameraCalibApp::closeEvent(QCloseEvent* event)
{
  if (worker_thread_ != nullptr && worker_thread_->isRunning())
    worker_thread_->stop();
  event->accept();
}

void CameraCalibApp::on_pushButton_stop_clicked() { worker_thread_->stop(); }

//--------- Private slots ------------------------------------------------------------//

void CameraCalibApp::handleResults(const CameraParams& params)
{
  updateProgressBar(0, 1);
  reply = QMessageBox::question(this, "SaveResults ",
                                "Do you want to save the camera parameters for later?",
                                QMessageBox::Yes | QMessageBox::No);
  if (reply == QMessageBox::Yes)
    saveCameraParams(params);

  emit calibrationSuccess(params);
}

void CameraCalibApp::handleErrors() { emit calibrationFailed(); }

void CameraCalibApp::updateProgressBar(const uint& current, const size_t& total)
{
  ui->progressBar->setValue(static_cast<int>(double(current) / total * 100));
}

void CameraCalibApp::setProgressLabel(const QString& text) { ui->label->setText(text); }

void CameraCalibApp::workFinished()
{
  delete worker_thread_;
  worker_thread_ = nullptr;
  close();
}

void CameraCalibApp::frwAugmentedFrame(const cv::Mat& augmented_frame) const
{
  emit augmentedFrameAvailable(augmented_frame);
}

void CameraCalibApp::frwPrintToQConsole(const QString& msg) const
{
  emit printToQConsole(msg);
}

//--------- Private functions --------------------------------------------------------//

void CameraCalibApp::saveCameraParams(const CameraParams& params)
{
  QString file_name = QFileDialog::getSaveFileName(
    this, tr("Save Camera Parameters"), tr("../.."), tr("Camera Parameters (*.json)"));
  CameraParamsJsonParser::writeJson(params, file_name.toStdString());
}
