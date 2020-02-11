/**
 * @file chartview.h
 * @author Simone Comari
 * @date 03 Jul 2019
 * @brief This file includes a custom reimplementation of Qt chart view (i.e. 2D plot).
 */

#ifndef CABLE_ROBOT_JOINTS_PVT_CHARTVIEW_H
#define CABLE_ROBOT_JOINTS_PVT_CHARTVIEW_H

#include <QLineSeries>
#include <QScatterSeries>
#include <QtCharts/QChartView>
#include <QtWidgets/QRubberBand>

#include "utils/types.h"

QT_CHARTS_USE_NAMESPACE

/**
 * @brief a custom reimplementation of Qt chart view (i.e. 2D plot).
 *
 * This reimplementation allows the user to easily plot a trajectory and highlight the
 * current point executed.
 *
 * Moreover, more advanced zooming/sliding functionalities are implemented. In particular:
 * - Zoom is achieved with mouse wheel scrolling, right-click selection of zoomed area and
 * plus/minus keyboard-button;
 * - Sliding can be achievied with keyboard arrows and left-click pressing;
 * - A double right-click reset the view to its original status, as well as R key-button.
 */
class ChartView: public QChartView
{
 public:
  /**
   * @brief Full constructor
   * @param chart A pointer to a QChart where to set the custom QChartView.
   * @param parent Optional parent widget.
   */
  explicit ChartView(QChart* chart, QWidget* parent = nullptr);

  /**
   * @brief setCableTrajectory
   * @param traj
   */
  void setCableTrajectory(const TrajectoryD& traj);
  /**
   * @brief setMotorPosTrajectory
   * @param traj
   */
  void setMotorPosTrajectory(const TrajectoryI& traj);
  /**
   * @brief setMotorVelTrajectory
   * @param traj
   */
  void setMotorVelTrajectory(const TrajectoryI& traj);
  /**
   * @brief setMotorTorqueTrajectory
   * @param traj
   */
  void setMotorTorqueTrajectory(const TrajectoryS& traj);

  /**
   * @brief highlightCurrentPoint
   * @param waypoint
   */
  void highlightCurrentPoint(const QPointF& waypoint);
  /**
   * @brief removeHighlight
   */
  void removeHighlight();

 private:
  QScatterSeries highlight_point_;
  QAbstractAxis* axisX_ = nullptr;
  QAbstractAxis* axisY_ = nullptr;

  qreal tot_scroll_x_        = 0;
  qreal tot_scroll_y_        = 0;
  qreal tot_scroll_x_zoomed_ = 0;
  qreal tot_scroll_y_zoomed_ = 0;

  bool scrolling_ = false;
  QPointF prev_pos_;

  void setTitles(const id_t& id, const QString& traj_type, const QString& unit);
  void resetView();

 private:
  void resizeEvent(QResizeEvent* event) override;

  void keyPressEvent(QKeyEvent* event) override;

  void mouseDoubleClickEvent(QMouseEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
};

#endif // CABLE_ROBOT_JOINTS_PVT_
