/**
 * @file homing_interface_vision.h
 * @author Simone Comari
 * @date 22 Mar 2019
 * @brief This file takes care of the functionalities of the vision-based homing
 * interface of cable robot app.
 *
 * The functionalities of the vision-based homing interface include buttons management
 * and signaling with the vision-based homing app, where the actual algorithm is
 * implemented.
 */

#ifndef CABLE_ROBOT_HOMING_INTERFACE_VISION_H
#define CABLE_ROBOT_HOMING_INTERFACE_VISION_H

#include <QMessageBox>
#include <QWidget>

#include "gui/homing/camera_widget.h"
#include "gui/homing/homing_interface.h"
#include "homing/homing_vision_app.h"

namespace Ui {
class HomingInterfaceVision;
}

/**
 * @brief This class implements the controls of the vision-based homing process and
 * allows the user to triggers events, set important parameters and interact with the
 * homing applications.
 */
class HomingInterfaceVision: public HomingInterface
{
  Q_OBJECT

 public:
  /**
   * @brief HomingInterfaceVision constructor.
   * @param parent The parent Qt object, in our case the homing dialog.
   * @param robot Pointer to the cable robot instance, to be passed to the inner app.
   */
  explicit HomingInterfaceVision(QWidget* parent, CableRobot* robot);
  ~HomingInterfaceVision() override final;

 private slots:
  void on_pushButton_enable_clicked();
  void on_pushButton_clearFaults_clicked();
  void on_pushButton_start_clicked();

  void on_pushButton_cancel_clicked();
  void on_pushButton_done_clicked();

 private slots:
  void appendText2Browser(const QString& text);
  void setCalibParams(const CalibParams& params) { app_.setCalibParams(params); }

 private:
  Ui::HomingInterfaceVision* ui;
  CameraWidget* camera_widget_ = NULL;

  HomingVisionApp app_;
  bool ext_close_cmd_;

  void closeEvent(QCloseEvent* event) override final;
};

#endif // CABLE_ROBOT_HOMING_INTERFACE_VISION_H
