#include "gui/homing/camera_calib_app.h"
#include "ui_camera_calib_app.h"

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
  // TODO ... (store valid frames)
  emit calibFrameCaptured(counter++, settings_.target_frames_num);
  if (counter < settings_.target_frames_num)
    return false;
  counter = 0;
  // TODO ... (perform calibration)
  CameraParams p;
  emit resultReady(p);
  return true;
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
  QString fileName = QFileDialog::getSaveFileName(
    this, tr("Save Camera Parameters"), tr("../.."), tr("Camera Parameters (*.json)"));
  // TODO...
}
