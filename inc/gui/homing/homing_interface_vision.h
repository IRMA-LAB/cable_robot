/**
 * @file homing_interface_vision.h
 * @author Simone Comari
 * @date 18 Jul 2019
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

#include "gui/camera/camera_widget.h"
#include "gui/homing/homing_interface.h"
#include "gui/homing/homing_interface_proprioceptive.h"
#include "homing/homing_vision_app.h"

namespace Ui {
class HomingInterfaceVisionWidget;
}

/**
 * @brief This class implements the controls of the vision-based homing process and
 * allows the user to triggers events, set important parameters and interact with the
 * homing applications.
 */
class HomingInterfaceVisionWidget: public QWidget
{
  Q_OBJECT

 public:
  /**
   * @brief HomingInterfaceVision constructor.
   * @param parent The parent Qt object, in our case the homing dialog.
   * @param robot Pointer to the cable robot instance, to be passed to the inner app.
   */
  explicit HomingInterfaceVisionWidget(QWidget* parent, CableRobot* robot,
                                       const VisionParams vision_config);
  ~HomingInterfaceVisionWidget();

  CameraWidget camera_widget;
  HomingInterfaceProprioceptiveWidget proprioceptive_widget;

 private slots:
  void on_pushButton_move_clicked();
  void on_pushButton_find_clicked();
  void on_pushButton_apply_clicked();

 private slots:
  void enableVisionTab();
  // Vision tab slots
  void appendText2Browser(const QString& text);
  void stopEstimation();

 private:
  Ui::HomingInterfaceVisionWidget* ui;

  CableRobot* robot_ptr_;
  HomingVisionApp app_;
};

class HomingInterfaceVision: public HomingInterface
{
 public:
  /**
   * @brief HomingInterfaceVision constructor.
   * @param parent The parent Qt object, in our case the homing dialog.
   * @param robot Pointer to the cable robot instance, to be passed to the inner app.
   */
  explicit HomingInterfaceVision(QWidget* parent, CableRobot* robot,
                                 const VisionParams vision_config);
  ~HomingInterfaceVision() override final;

 private:
  HomingInterfaceVisionWidget widget_;

  bool rejectedExitRoutine(const bool) override final;
};

#endif // CABLE_ROBOT_HOMING_INTERFACE_VISION_H
