/**
 * @file scatter3d_widget.h
 * @author Simone Comari
 * @date 03 Jul 2019
 * @brief This file includes a custom reimplementation of Qt scatter 3D plot.
 */

#ifndef CABLE_ROBOT_SCATTER3DWIDGET_H
#define CABLE_ROBOT_SCATTER3DWIDGET_H

#include <QScreen>
#include <QWidget>
#include <QtCore/qmath.h>
#include <QtCore/qrandom.h>
#include <QtDataVisualization/Q3DInputHandler>
#include <QtDataVisualization/q3dcamera.h>
#include <QtDataVisualization/q3dscatter.h>
#include <QtDataVisualization/q3dscene.h>
#include <QtDataVisualization/q3dtheme.h>
#include <QtDataVisualization/qabstract3dseries.h>
#include <QtDataVisualization/qscatter3dseries.h>
#include <QtDataVisualization/qscatterdataproxy.h>
#include <QtDataVisualization/qvalue3daxis.h>

#include "libgeom/inc/rotations.h"

#include "ctrl/controller_joints_pvt.h"

using namespace QtDataVisualization;

/**
 * @brief A custom implementation of Qt input handler for a 3D scatter widget.
 *
 * This class allows the user to rotate the graph and to follow up on parent window
 * resizing.
 */
class CustomInputHandler: public Q3DInputHandler
{
  Q_OBJECT
 public:
  /**
   * @brief Default constructor.
   * @param parent Optional parent widget.
   */
  explicit CustomInputHandler(QObject* parent = nullptr);
  /**
   * @brief Full constructor.
   * @param parent Parent widget.
   * @param parent_widget_size Parent widget's size.
   */
  explicit CustomInputHandler(QObject* parent, const QSize& parent_widget_size);

 public slots:
  /**
   * @brief Set widget size.
   * @param size New desired widget size.
   */
  void setWidgetSize(const QSize& size);
  /**
   * @brief Set widget width.
   * @param width New desired widget width.
   */
  void setWidgetWidth(const int& width);
  /**
   * @brief Set widget height.
   * @param height New desired widget height.
   */
  void setWidgetHeight(const int& height);

 private:
  bool pressed_;
  QSizeF widget_size_;
  QVector3D prev_target_;
  QPoint start_pos_;

  void mousePressEvent(QMouseEvent* event, const QPoint& mousePos) override;
  void mouseReleaseEvent(QMouseEvent* event, const QPoint& mousePos) override;
  void mouseMoveEvent(QMouseEvent* event, const QPoint& mousePos) override;
};


namespace Ui {
class Scatter3DWidget;
}

/**
 * @brief A custom implementation a 3D scatter widget.
 *
 * Thanks to its sliders, it is possible to zoom to a specific range in each cartesian
 * direction.
 */
class Scatter3DWidget: public QWidget
{
  Q_OBJECT

 public:
  /**
   * @brief Default constructor.
   * @param parent Optional parent widget.
   */
  explicit Scatter3DWidget(QWidget* parent = nullptr);
  ~Scatter3DWidget();

  /**
   * @brief Set 3D scatter data from a given platform trajectory expressed in global
   * reference frame.
   * @param trajectory The trajectory expressed in global reference frame.
   */
  void setTrajectory(const vect<TrajectoryD>& trajectory);

 private slots:
  void on_horizontalSlider_zmin_valueChanged(int value);
  void on_horizontalSlider_ymin_valueChanged(int value);
  void on_horizontalSlider_xmin_valueChanged(int value);

  void on_horizontalSlider_xmax_valueChanged(int value);
  void on_horizontalSlider_ymax_valueChanged(int value);
  void on_horizontalSlider_zmax_valueChanged(int value);

 private:
  Ui::Scatter3DWidget* ui;

  Q3DScatter* graph_;
  CustomInputHandler* input_handler_;
};

#endif // CABLE_ROBOT_SCATTER3DWIDGET_H
