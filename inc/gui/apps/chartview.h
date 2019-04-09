#ifndef CABLE_ROBOT_JOINTS_PVT_CHARTVIEW_H
#define CABLE_ROBOT_JOINTS_PVT_CHARTVIEW_H

#include <QtCharts/QChartView>
#include <QtWidgets/QRubberBand>

#include "gui/apps/joints_pvt_dialog.h"

QT_CHARTS_USE_NAMESPACE

class ChartView: public QChartView
{
 public:
  ChartView(QChart* chart, QWidget* parent = nullptr);

  void setCableTrajectory(const TrajectoryD& traj);
  void setMotorPosTrajectory(const TrajectoryI& traj);
  void setMotorVelTrajectory(const TrajectoryI& traj);
  void setMotorTorqueTrajectory(const TrajectoryS& traj);

private:
  qreal tot_scroll_x_        = 0;
  qreal tot_scroll_y_        = 0;
  qreal tot_scroll_x_zoomed_ = 0;
  qreal tot_scroll_y_zoomed_ = 0;

  bool scrolling_ = false;
  QPointF prev_pos_;

  void resizeEvent(QResizeEvent *event) override;

  void keyPressEvent(QKeyEvent* event) override;

  void mouseDoubleClickEvent(QMouseEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;

  void setTitles(const id_t& id, const QString& traj_type, const QString& unit);
  void resetView();
};

#endif // CABLE_ROBOT_JOINTS_PVT_
