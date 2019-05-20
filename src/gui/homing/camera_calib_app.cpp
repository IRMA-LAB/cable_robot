#include "gui/homing/camera_calib_app.h"
#include "ui_camera_calib_app.h"
#include "utils/cameraparamsjsonparser.h"

//------------------------------------------------------------------------------------//
//--------- WorkerThread class -------------------------------------------------------//
//------------------------------------------------------------------------------------//

WorkerThread::WorkerThread(const CameraCalibSettings& settings)
  : settings_(settings), new_frame_pending_(false), stop_(false)
{}

//--------- Public functions ---------------------------------------------------------//

void WorkerThread::setNewFrame(const cv::Mat& frame)
{
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
      emit processFrameError();
      break;
    }

    // Grab new frame
    if (!new_frame_pending_)
      new_frame_available_.wait(&mutex_);
    cv::Mat frame(latest_frame_);
    new_frame_pending_ = false;
    mutex_.unlock();

    // Process frame
    if (processFrame(frame))
      break;
  }
}

bool WorkerThread::processFrame(const cv::Mat& frame)
{
  static int counter = 0; // dummy
  if (storeValidFrame(frame))
    emit calibFrameCaptured(counter++, settings_.num_frames);

  if (counter < settings_.num_frames)
    return false;

  counter = 0;
  CameraParams p;
  runCalibrationAndSave(p);
  emit resultReady(p);
  return true;
}

cv::Mat WorkerThread::getUndistortedImage(const cv::Mat& image)
{
  cv::Mat undistorted;
  if (settings_.use_fisheye)
  {
    cv::fisheye::undistortImage(image, undistorted, camera_params_.camera_matrix,
                                camera_params_.dist_coeff, camera_params_.camera_matrix);
  }
  else
  {
    undistort(image, undistorted, camera_params_.camera_matrix, camera_params_.dist_coeff,
              camera_params_.camera_matrix);
  }
  // Add text
  int base_line      = 0;
  std::string msg    = "Undistorted";
  cv::Size text_size = cv::getTextSize(msg, 5, 1, 1, &base_line);
  cv::Point text_origin(undistorted.cols - 2 * text_size.width - 10,
                        undistorted.rows - 2 * base_line - 10);
  putText(undistorted, msg, text_origin, 1, 1.6, cv::Scalar(0, 255, 0), 2);
  return undistorted;
}

bool WorkerThread::findChessboard(const cv::Mat& image)
{
  bool found = findChessboardCorners(image, settings_.board_size, point_buf_,
                                     settings_.chess_board_flags);
  // Draw the corners.
  drawChessboardCorners(image, settings_.board_size, cv::Mat(point_buf_), found);
  return found;
}

bool WorkerThread::catchCalibrationSample(const cv::Mat& image)
{
  static const clock_t kClocksPerInterval = settings_.delay * 1e-3 * CLOCKS_PER_SEC;

  image_size_ = image.size();
  clock_t elapsed_time = clock() - prev_timestamp_;
  if (calib_mode_ == DETECTION && elapsed_time > kClocksPerInterval)
    calib_mode_ = CAPTURING;

  if (calib_mode_ == CAPTURING)
  {
    cv::Mat view_gray;
    cvtColor(image, view_gray, cv::COLOR_BGR2GRAY);
    cornerSubPix(
      view_gray, point_buf_, cv::Size(11, 11), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001));
    if (!image_points_.empty())
      if (!checkFrameAgainstPrev()) // Compare against previous image(s)
        return false;
    // Save sample
    image_points_.push_back(point_buf_);
    prev_timestamp_ = clock();
    calib_mode_     = DETECTION;
    return true;
  }
  return false;
}

bool WorkerThread::checkFrameAgainstPrev()
{
  for (uint j = 0; j < uint(image_points_.size()); j++)
  {
    for (uint i = 0; i < uint(settings_.board_size.width); i++)
    {
      delta_x_ = delta_x_ + uint(std::abs(point_buf_[i].x - image_points_[j][i].x));
      if (counter_ < uint(settings_.board_size.height))
      {
        delta_y_ =
          delta_y_ + int(std::abs(point_buf_[i + settings_.board_size.width].y -
                                  image_points_[j][i + settings_.board_size.width].y));
      }
      counter_++;
    }
    if (delta_x_ > error_x_)
      counter_x_++;
    if (delta_y_ > error_y_)
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

bool WorkerThread::runCalibrationAndSave(CameraParams & params)
{
  std::vector<cv::Mat> r_vecs, t_vecs;
  std::vector<float> reproj_errs;
  double total_avg_err = 0;
  std::vector<cv::Point3f> new_obj_points;

  if (runCalibration(r_vecs, t_vecs, reproj_errs, total_avg_err, new_obj_points))
  {
    calib_mode_         = CALIBRATED;
    params = camera_params_;
    return true;
  }
  return false;
}

bool WorkerThread::runCalibration(std::vector<cv::Mat>& rvecs,
                                  std::vector<cv::Mat>& tvecs,
                                  std::vector<float>& reprojErrs, double& total_avg_err,
                                  std::vector<cv::Point3f>& new_obj_points)
{

  std::vector<std::vector<cv::Point3f>> object_points(1);
  for (int i = 0; i < settings_.board_size.height; ++i)
    for (int j = 0; j < settings_.board_size.width; ++j)
      object_points[0].push_back(
        cv::Point3f(j * settings_.square_size, i * settings_.square_size, 0));

  object_points[0][settings_.board_size.width - 1].x =
    object_points[0][0].x + settings_.grid_width;
  new_obj_points = object_points[0];

  object_points.resize(image_points_.size(), object_points[0]);

  // Find intrinsic and extrinsic camera parameters
  double rms;
  if (settings_.use_fisheye)
  {
    cv::Mat _rvecs, _tvecs;
    rms = cv::fisheye::calibrate(object_points, image_points_, image_size_,
                                 camera_params_.camera_matrix, camera_params_.dist_coeff,
                                 _rvecs, _tvecs, settings_.calib_flags);

    rvecs.reserve(_rvecs.rows);
    tvecs.reserve(_tvecs.rows);
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
    rms = calibrateCameraRO(object_points, image_points_, image_size_, i_fixedPoint,
                            camera_params_.camera_matrix, camera_params_.dist_coeff,
                            rvecs, tvecs, new_obj_points, settings_.calib_flags);
  }

  std::cout << "New board corners: " << std::endl;
  std::cout << new_obj_points[0] << std::endl;
  std::cout << new_obj_points[settings_.board_size.width - 1] << std::endl;
  std::cout
    << new_obj_points[settings_.board_size.width * (settings_.board_size.height - 1)]
    << std::endl;
  std::cout << new_obj_points.back() << std::endl;

  std::cout << "Re-projection error reported by calibrateCamera: " << rms << std::endl;

  bool ok =
    checkRange(camera_params_.camera_matrix) && checkRange(camera_params_.dist_coeff);

  object_points.clear();
  object_points.resize(image_points_.size(), new_obj_points);
  total_avg_err = computeCalibReprojectionErr(object_points, rvecs, tvecs, reprojErrs);

  return ok;
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
    if (settings_.use_fisheye)
    {
      cv::fisheye::projectPoints(object_points[i], image_points2, r_vecs[i], t_vecs[i],
                                 camera_params_.camera_matrix, camera_params_.dist_coeff);
    }
    else
    {
      projectPoints(object_points[i], r_vecs[i], t_vecs[i], camera_params_.camera_matrix,
                    camera_params_.dist_coeff, image_points2);
    }

    err = norm(image_points_[i], image_points2, cv::NORM_L2);

    size_t n           = object_points[i].size();
    per_view_errors[i] = float(std::sqrt(err * err / n));
    total_err += err * err;
    total_points += n;
  }

  return std::sqrt(total_err / total_points);
}

bool WorkerThread::storeValidFrame(const cv::Mat& view)
{
  if (!findChessboard(view))
    return false;
  if (catchCalibrationSample(view))
   return true;
  return false;
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
          &CameraCalibApp::handleResults);
  connect(worker_thread_, &WorkerThread::processFrameError, this,
          &CameraCalibApp::handleErrors);
  connect(worker_thread_, &WorkerThread::finished, this, &CameraCalibApp::workFinished);
  worker_thread_->start();
}

void CameraCalibApp::setNewFrame(const cv::Mat& frame)
{
  if (worker_thread_ != NULL && worker_thread_->isRunning())
    worker_thread_->setNewFrame(frame);
}

//--------- Private GUI slots --------------------------------------------------------//

void CameraCalibApp::on_pushButton_stop_clicked() { worker_thread_->stop(); }

//--------- Private slots ------------------------------------------------------------//

void CameraCalibApp::handleResults(const CameraParams& params)
{
  QMessageBox::StandardButton reply = QMessageBox::question(
    this, "Save Results", "Do you want to save the camera parameters for later?",
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
  worker_thread_ = NULL;
  close();
}

//--------- Private functions --------------------------------------------------------//

void CameraCalibApp::saveCameraParams(const CameraParams& params)
{
  QString file_name = QFileDialog::getSaveFileName(
    this, tr("Save Camera Parameters"), tr("../.."), tr("Camera Parameters (*.json)"));
  CameraParamsJsonParser jsonParser;

  jsonParser.writeJson(params, file_name.toStdString());
}
