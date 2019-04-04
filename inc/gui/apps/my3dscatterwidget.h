#ifndef CABLE_ROBOT_JOINTS_PVTMY3DSCATTERWIDGET_H
#define CABLE_ROBOT_JOINTS_PVTMY3DSCATTERWIDGET_H

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

class CustomInputHandler: public Q3DInputHandler
{
  Q_OBJECT
 public:
  explicit CustomInputHandler(QObject* parent = NULL);
  explicit CustomInputHandler(QObject* parent, const QSize& parent_widget_size);

  void mousePressEvent(QMouseEvent* event, const QPoint& mousePos) override;
  void mouseReleaseEvent(QMouseEvent* event, const QPoint& mousePos) override;
  void mouseMoveEvent(QMouseEvent* event, const QPoint& mousePos) override;

 public slots:
  void setWidgetSize(const QSize& size);
  void setWidgetWidth(const int& width);
  void setWidgetHeight(const int& height);

 private:
  bool pressed_;
  QSizeF widget_size_;
  QVector3D prev_target_;
  QPoint start_pos_;
};


namespace Ui {
class My3DScatterWidget;
}

class My3DScatterWidget: public QWidget
{
  Q_OBJECT

 public:
  explicit My3DScatterWidget(QWidget* parent = 0);
  ~My3DScatterWidget();

  void setTrajectory(const vect<TrajectoryD>& trajectory);

 private slots:
  void on_horizontalSlider_zmin_valueChanged(int value);
  void on_horizontalSlider_ymin_valueChanged(int value);
  void on_horizontalSlider_xmin_valueChanged(int value);

  void on_horizontalSlider_xmax_valueChanged(int value);
  void on_horizontalSlider_ymax_valueChanged(int value);
  void on_horizontalSlider_zmax_valueChanged(int value);

 private:
  Ui::My3DScatterWidget* ui;

  Q3DScatter* graph_;
  CustomInputHandler* input_handler_;
};

#endif // CABLE_ROBOT_JOINTS_PVTMY3DSCATTERWIDGET_H
