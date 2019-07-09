/**
 * @file camera_widget.cpp
 * @author Simone Comari
 * @date 09 Jul 2019
 * @brief This file includes definitions of classes present in camera_widget.h.
 */

#include "gui/camera/camera_widget.h"
#include "ui_camera_widget.h"

//------------------------------------------------------------------------------------//
//--------- QGraphicsVideoStreamerItem class -----------------------------------------//
//------------------------------------------------------------------------------------//

QGraphicsVideoStreamerItem::QGraphicsVideoStreamerItem()
  : QObject(), QGraphicsPixmapItem()
{
  connect(&record_symbol_timer_, SIGNAL(timeout()), this, SLOT(changeRecSymbolStatus()));
}

//--------- Public functions --------------------------------------------------------//

void QGraphicsVideoStreamerItem::recording(const bool value)
{
  if (value)
    record_symbol_timer_.start(kRecSymIntervalMsec_);
  else
  {
    record_symbol_timer_.stop();
    rec_symbol_active_ = false;
  }
}

//--------- Private functions -------------------------------------------------------//

void QGraphicsVideoStreamerItem::paint(QPainter* painter,
                                       const QStyleOptionGraphicsItem* option,
                                       QWidget* widget)
{
  QGraphicsPixmapItem::paint(painter, option, widget);
  if (rec_symbol_active_)
  {
    painter->setBrush(QBrush(Qt::red));
    painter->setPen(Qt::red);
    int pos  = static_cast<int>(this->pixmap().height() * 0.05);
    int size = static_cast<int>(pos * 1.5);
    painter->drawEllipse(pos, pos, size, size);
  }
}

//------------------------------------------------------------------------------------//
//--------- CameraWidget class -------------------------------------------------------//
//------------------------------------------------------------------------------------//

CameraWidget::CameraWidget() : ui(new Ui::CameraWidget)
{
  ui->setupUi(this);

  // Parse available cameras
  const QList<QCameraInfo> available_cameras = QCameraInfo::availableCameras();
  for (const QCameraInfo& camera_info : available_cameras)
  {
    ui->comboBox_source->addItem(camera_info.description(),
                                 camera_info.deviceName().right(1).toInt());
  }

  ui->pushButton_start->setEnabled(!available_cameras.empty());
  on_comboBox_channel_currentIndexChanged(ui->comboBox_channel->currentText());

  // Setup video streaming window
  ui->graphicsView_video->setScene(new QGraphicsScene(this));
  ui->graphicsView_video->scene()->addItem(&video_streamer_);
}

CameraWidget::~CameraWidget()
{
  if (video_.isOpened())
    ui->pushButton_stop->click();
}

//--------- Public functions --------------------------------------------------------//

void CameraWidget::stopVideoStream()
{
  if (!video_.isOpened())
    return;

  if (video_rec_.isOpened())
    ui->pushButton_stopRec->click();
  video_.release();
  CLOG(INFO, "event") << "Video stream stopped";

  ui->comboBox_source->setEnabled(true);
  ui->pushButton_start->setEnabled(true);
  ui->pushButton_stop->setDisabled(true);
  ui->pushButton_calib->setDisabled(true);
  ui->pushButton_takeImage->setDisabled(true);
  ui->pushButton_record->setDisabled(true);

  QPixmap pixmap = QPixmap::fromImage(qimg_);
  pixmap.fill(Qt::white);
  video_streamer_.setPixmap(pixmap);
  ui->graphicsView_video->fitInView(&video_streamer_, Qt::KeepAspectRatio);
}

//--------- Private GUI slots -------------------------------------------------------//

void CameraWidget::on_comboBox_channel_currentIndexChanged(const QString& arg1)
{
  CLOG(TRACE, "event") << arg1;
  if (arg1 == "Grayscale")
    stream_type_ = VideoStreamType::GRAYSCALE;
  else
    stream_type_ = VideoStreamType::ORIGINAL;
}

void CameraWidget::on_pushButton_start_clicked()
{
  CLOG(TRACE, "event");
  int camera_idx = ui->comboBox_source->currentData().toInt();
  if (!video_.open(camera_idx))
  {
    QMessageBox::critical(this, "Camera Error",
                          "Make sure camera is connected or that the camera is not being "
                          "accessed by another program!");
    return;
  }

  ui->comboBox_source->setDisabled(true);
  ui->pushButton_start->setDisabled(true);
  ui->pushButton_stop->setEnabled(true);
  ui->pushButton_calib->setEnabled(true);
  ui->pushButton_takeImage->setEnabled(true);
  ui->pushButton_record->setEnabled(true);

  stream();
}

void CameraWidget::on_pushButton_stop_clicked()
{
  CLOG(TRACE, "event");
  stopVideoStream();
}

void CameraWidget::on_pushButton_calib_clicked()
{
  CLOG(TRACE, "event");
  deleteCalibDialog();
  calib_dialog_ = new CameraCalibDialog(this);
  connect(this, SIGNAL(newFrameGrabbed(cv::Mat)), calib_dialog_,
          SLOT(getNewVideoFrame(cv::Mat)));
  connect(calib_dialog_, SIGNAL(cameraParamsReady(CameraParams)), this,
          SLOT(storeCameraParams(CameraParams)));
  connect(calib_dialog_, SIGNAL(printToQConsole(QString)), this,
          SLOT(frwPrintToQConsole(QString)));
  calib_dialog_->show();
}

void CameraWidget::on_pushButton_imgDir_clicked()
{
  CLOG(TRACE, "event");
  QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"), "/home",
                                                  QFileDialog::ShowDirsOnly |
                                                    QFileDialog::DontResolveSymlinks);
  if (dir.isEmpty())
  {
    QMessageBox::warning(this, "I/O Error", "Directory filename is empty");
    return;
  }
  ui->lineEdit_imgDir->setText(dir);
}

void CameraWidget::on_pushButton_takeImage_clicked()
{
  CLOG(TRACE, "event");
  const QString dirpath = ui->lineEdit_imgDir->text();
  if (dirpath.isEmpty())
  {
    QMessageBox::warning(this, "I/O Error",
                         "Please select an output directory for the captured image.");
    return;
  }

  const std::string filepath =
    QDir(dirpath)
      .filePath(tr("img_capture_%1.png")
                  .arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss")))
      .toStdString();
  bool result = false;
  try
  {
    result = imwrite(filepath, frame_);
  }
  catch (const cv::Exception& ex)
  {
    QMessageBox::critical(
      this, "Image Capture Error",
      tr("Exception converting image to PNG format: %s").arg(ex.what()));
  }
  if (!result)
  {
    CLOG(ERROR, "event") << "Image capture failed";
    QMessageBox::critical(this, "Image Capture Error", "Can't save PNG file.");
    return;
  }
  CLOG(INFO, "event") << "Image captured";
  // Display captured image for 3 seconds.
  displayCapturedImage();
}

void CameraWidget::on_pushButton_videoDir_clicked()
{
  CLOG(TRACE, "event");
  QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"), "/home",
                                                  QFileDialog::ShowDirsOnly |
                                                    QFileDialog::DontResolveSymlinks);
  if (dir.isEmpty())
  {
    QMessageBox::warning(this, "I/O Error", "Directory filename is empty");
    return;
  }
  ui->lineEdit_videoDir->setText(dir);
}

void CameraWidget::on_pushButton_record_clicked()
{
  CLOG(TRACE, "event");
  const QString dirpath = ui->lineEdit_videoDir->text();
  if (dirpath.isEmpty())
  {
    QMessageBox::warning(this, "I/O Error",
                         "Please select an output directory for the recorded video.");
    return;
  }
  // Select desired codec (must be available at runtime)
  const int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  // Framerate of the created video stream
  const double fps = video_.get(cv::CAP_PROP_FPS);
  const std::string filepath =
    QDir(dirpath)
      .filePath(tr("video_rec_%1.avi")
                  .arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss")))
      .toStdString(); // name of the output video file
  video_rec_.open(filepath, codec, fps, frame_.size(), (frame_.type() == CV_8UC3));
  // Check if we succeeded
  if (!video_rec_.isOpened())
  {
    QMessageBox::critical(this, "Video Recorder Error", "Can't create video file.");
    CLOG(ERROR, "event") << "Video recording failed";
  }
  else
  {
    CLOG(INFO, "event") << "Video recording started";
    ui->pushButton_stopRec->setEnabled(true);
    ui->pushButton_record->setDisabled(true);

    video_streamer_.recording(true);
  }
}

void CameraWidget::on_pushButton_stopRec_clicked()
{
  CLOG(TRACE, "event");
  video_streamer_.recording(false);

  video_rec_.release();
  CLOG(INFO, "event") << "Video recording stopped";

  ui->pushButton_stopRec->setDisabled(true);
  ui->pushButton_record->setEnabled(true);
}

//--------- Private slots -----------------------------------------------------------//

void CameraWidget::storeCameraParams(const CameraParams& params)
{
  camera_params_ = params;
  emit calibParamsReady(camera_params_);
  deleteCalibDialog();
}

//--------- Private functions -------------------------------------------------------//

void CameraWidget::stream()
{
  CLOG(INFO, "event") << "Video stream started";
  cv::Mat frame;
  while (video_.isOpened())
  {
    video_ >> frame;
    if (!frame.empty())
    {
      processFrame(frame);
      emit newFrameGrabbed(frame_);

      if (video_rec_.isOpened())
        video_rec_.write(frame_);

      if (calib_dialog_ == NULL || calib_dialog_->isHidden())
      {
        mapToQImage();
        video_streamer_.setPixmap(QPixmap::fromImage(qimg_.rgbSwapped()));
        ui->graphicsView_video->fitInView(&video_streamer_, Qt::KeepAspectRatio);
      }
    }
    qApp->processEvents();
  }
}

void CameraWidget::processFrame(const cv::Mat& raw_frame)
{
  switch (stream_type_)
  {
    case GRAYSCALE:
      cv::cvtColor(raw_frame, frame_, cv::COLOR_RGB2GRAY);
      break;
    case ORIGINAL:
      frame_ = raw_frame;
      break;
  }
}

void CameraWidget::mapToQImage()
{
  switch (stream_type_)
  {
    case GRAYSCALE:
      qimg_ = QImage(frame_.data, frame_.cols, frame_.rows, (int)frame_.step,
                     QImage::Format_Grayscale8);
      break;
    case ORIGINAL:
      qimg_ = QImage(frame_.data, frame_.cols, frame_.rows, (int)frame_.step,
                     QImage::Format_RGB888);
      break;
  }
}

void CameraWidget::displayStream()
{
  CLOG(TRACE, "event");
  ui->stackedWidget->setCurrentIndex(0);
}

void CameraWidget::displayCapturedImage()
{
  CLOG(TRACE, "event");
  QImage scaled_image = qimg_.scaled(ui->graphicsView_video->size(), Qt::KeepAspectRatio,
                                     Qt::SmoothTransformation);
  ui->label_lastImgPreview->setPixmap(QPixmap::fromImage(scaled_image.rgbSwapped()));
  ui->stackedWidget->setCurrentIndex(1);
  QTimer::singleShot(3000, this, &CameraWidget::displayStream);
}

void CameraWidget::closeEvent(QCloseEvent* event)
{
  if (video_.isOpened())
  {
    QMessageBox::warning(this, "Warning",
                         "Stop the video before closing the application!");
    event->ignore();
  }
  else
    event->accept();
}

void CameraWidget::deleteCalibDialog()
{
  if (calib_dialog_ == NULL)
    return;

  disconnect(this, SIGNAL(newFrameGrabbed(cv::Mat)), calib_dialog_,
             SLOT(getNewVideoFrame(cv::Mat)));
  disconnect(calib_dialog_, SIGNAL(cameraParamsReady(CameraParams)), this,
             SLOT(storeCameraParams(CameraParams)));
  disconnect(calib_dialog_, SIGNAL(printToQConsole(QString)), this,
             SLOT(frwPrintToQConsole(QString)));
  calib_dialog_->close();
  delete calib_dialog_;
  calib_dialog_ = NULL;
}
