/**
 * @file camera_widget.h
 * @author Simone Comari
 * @date 12 Jul 2019
 * @brief This file includes a widget to stream a USB camera.
 */

#ifndef CABLE_ROBOT_HOMING_CAMERA_WIDGET_H
#define CABLE_ROBOT_HOMING_CAMERA_WIDGET_H

#include <QCameraInfo>
#include <QCloseEvent>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QImage>
#include <QMessageBox>
#include <QObject>
#include <QPixmap>
#include <QTimer>
#include <QWidget>

#include "opencv2/opencv.hpp"

#include "easylogging++.h"

#include "gui/camera/camera_calib_dialog.h"

namespace Ui {
class CameraWidget;
}

/**
 * @brief The VideoStreamType enum
 */
enum VideoStreamType
{
  ORIGINAL,
  GRAYSCALE,
  AUGMENTED,
  UNDISTORTED
};

/**
 * @brief The QGraphicsVideoStreamerItem class
 */
class QGraphicsVideoStreamerItem: public QObject, public QGraphicsPixmapItem
{
  Q_OBJECT

 public:
  /**
   * @brief QGraphicsVideoStreamerItem
   */
  QGraphicsVideoStreamerItem();

  /**
   * @brief recording
   * @param value
   */
  void recording(const bool value);

 private slots:
  void changeRecSymbolStatus() { rec_symbol_active_ = !rec_symbol_active_; }

 private:
  static constexpr int kRecSymIntervalMsec_ = 1000;

  QTimer record_symbol_timer_;
  bool rec_symbol_active_ = false;

  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
             QWidget* widget) override final;
};


/**
 * @brief The CameraWidget class
 */
class CameraWidget: public QWidget
{
  Q_OBJECT

 public:
  /**
   * @brief CameraWidget
   */
  CameraWidget();
  ~CameraWidget();

  Ui::CameraWidget* ui;
  const cv::Mat& getLatestFrame() const { return processed_frame_; }

  void changeStreamType(const VideoStreamType new_type);
  void enableStreamType(const bool value = true);

  void stopVideoStream();

  bool isStreaming() const { return video_.isOpened(); }

 signals:
  /**
   * @brief printToQConsole
   */
  void printToQConsole(const QString&) const;
  /**
   * @brief newFrameGrabbed
   */
  void newFrameGrabbed(const cv::Mat&) const;
  /**
   * @brief calibParamsReady
   */
  void calibParamsReady(const CameraParams&) const;
  /**
   * @brief videoStreamStopped
   */
  void videoStreamStopped() const;

 public slots:
  void setAugmentedFrame(const cv::Mat& augm_frame);

 private slots:
  void on_comboBox_channel_currentIndexChanged(const QString& arg1);

  void on_pushButton_start_clicked();
  void on_pushButton_stop_clicked();
  void on_pushButton_calib_clicked();

  void on_pushButton_imgDir_clicked();
  void on_pushButton_takeImage_clicked();

  void on_pushButton_videoDir_clicked();
  void on_pushButton_record_clicked();
  void on_pushButton_stopRec_clicked();

private slots:
  void storeCameraParams(const CameraParams& params);
  void frwPrintToQConsole(const QString& msg) { emit printToQConsole(msg); }
  void handleCalibrationStatusChanged(const CalibrationStatus status);

private:
  CameraCalibDialog* calib_dialog_ = nullptr;

  QMutex mutex_;

  QGraphicsVideoStreamerItem video_streamer_;
  cv::VideoCapture video_;
  cv::VideoWriter video_rec_;
  cv::Mat processed_frame_;
  cv::Mat augmented_frame_;
  cv::Mat display_frame_;
  QImage qimg_;
  VideoStreamType stream_type_;

  CameraParams camera_params_;

  void stream();

  void processFrame(const cv::Mat& raw_frame);
  cv::Mat getUndistortedImage(const cv::Mat& raw_frame);
  void displayFrame();
  void mapToQImage();

  void displayStream();
  void displayCapturedImage();

  void closeEvent(QCloseEvent* event);

  void deleteCalibDialog();
};

#endif // CABLE_ROBOT_HOMING_CAMERA_WIDGET_H
