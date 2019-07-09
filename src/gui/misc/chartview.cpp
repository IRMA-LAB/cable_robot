/**
 * @file chartview.cpp
 * @author Simone Comari
 * @date 09 Jul 2019
 * @brief This file includes definitions of class present in chartview.h.
 */

#include "gui/misc/chartview.h"

ChartView::ChartView(QChart* chart, QWidget* parent) : QChartView(chart, parent)
{
  setRenderHint(QPainter::Antialiasing);
  setRubberBand(QChartView::RectangleRubberBand);
  setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
  setSizeIncrement(1, 1);
  setFocusPolicy(Qt::StrongFocus);

  chart->setAnimationOptions(QChart::SeriesAnimations);
  chart->legend()->hide();

  QPen pen = highlight_point_.pen();
  pen.setWidth(1);
  pen.setColor("red");
  highlight_point_.setPen(pen);
  highlight_point_.setColor("red");
}

//--------- Public functions --------------------------------------------------------//

void ChartView::setCableTrajectory(const TrajectoryD& traj)
{
  chart()->removeAllSeries();
  QLineSeries* series = new QLineSeries();
  for (size_t i = 0; i < traj.timestamps.size(); ++i)
  {
    WayPointD waypoint = traj.waypointFromIndex(i);
    *series << QPointF(waypoint.ts, waypoint.value);
  }
  chart()->addSeries(series);

  setTitles(traj.id, "cable length", "m");
}

void ChartView::setMotorPosTrajectory(const TrajectoryI& traj)
{
  chart()->removeAllSeries();
  QLineSeries* series = new QLineSeries();
  for (size_t i = 0; i < traj.timestamps.size(); ++i)
  {
    WayPointI waypoint = traj.waypointFromIndex(i);
    *series << QPointF(waypoint.ts, waypoint.value);
  }
  chart()->addSeries(series);

  setTitles(traj.id, "motor position", "counts");
}

void ChartView::setMotorVelTrajectory(const TrajectoryI& traj)
{
  chart()->removeAllSeries();
  QLineSeries* series = new QLineSeries();
  for (size_t i = 0; i < traj.timestamps.size(); ++i)
  {
    WayPointI waypoint = traj.waypointFromIndex(i);
    *series << QPointF(waypoint.ts, waypoint.value);
  }
  chart()->addSeries(series);

  setTitles(traj.id, "motor velocity", "counts/s");
}

void ChartView::setMotorTorqueTrajectory(const TrajectoryS& traj)
{
  chart()->removeAllSeries();
  QLineSeries* series = new QLineSeries();
  for (size_t i = 0; i < traj.timestamps.size(); ++i)
  {
    WayPointS waypoint = traj.waypointFromIndex(i);
    *series << QPointF(waypoint.ts, waypoint.value);
  }
  chart()->addSeries(series);

  setTitles(traj.id, "motor torque", "nominal points");
}

void ChartView::highlightCurrentPoint(const QPointF& waypoint)
{
  removeHighlight();
  highlight_point_ << waypoint;
  chart()->addSeries(&highlight_point_);
  chart()->setAxisX(axisX_, &highlight_point_);
  chart()->setAxisY(axisY_, &highlight_point_);
}

void ChartView::removeHighlight()
{
  if (highlight_point_.count() > 0)
  {
    chart()->removeSeries(&highlight_point_);
    highlight_point_.clear();
  }
}

//--------- Private functions --------------------------------------------------------//

void ChartView::setTitles(const id_t& id, const QString& traj_type, const QString& unit)
{
  chart()->setTitle(tr("Actuator #%1 %2 trajectory").arg(id).arg(traj_type));
  chart()->setTitleFont(QFont(chart()->font().family(), 12, QFont::Bold));

  chart()->createDefaultAxes();

  chart()->axisX()->setTitleText("time from start [sec]");
  chart()->axisX()->setTitleFont(QFont(chart()->font().family(), 11, QFont::Medium));
  chart()->axisX()->setTitleVisible();

  chart()->axisY()->setTitleText(tr("set-point [%1]").arg(unit));
  chart()->axisY()->setTitleFont(QFont(chart()->font().family(), 11, QFont::Medium));
  chart()->axisY()->setTitleVisible();

  axisX_ = chart()->axisX();
  axisY_ = chart()->axisY();
}

void ChartView::resetView()
{
  chart()->scroll(-tot_scroll_x_zoomed_, -tot_scroll_y_zoomed_);
  chart()->zoomReset();
  chart()->scroll(-tot_scroll_x_, -tot_scroll_y_);
  tot_scroll_x_ = 0;
  tot_scroll_y_ = 0;
}

//--------- Private GUI events ------------------------------------------------------//

void ChartView::resizeEvent(QResizeEvent* event)
{
  qreal scaling_factor_x =
    static_cast<qreal>(event->size().width()) / event->oldSize().width();
  qreal scaling_factor_y =
    static_cast<qreal>(event->size().height()) / event->oldSize().height();
  tot_scroll_x_ *= scaling_factor_x;
  tot_scroll_y_ *= scaling_factor_y;
  tot_scroll_x_zoomed_ *= scaling_factor_x;
  tot_scroll_y_zoomed_ *= scaling_factor_y;

  QChartView::resizeEvent(event);
}

void ChartView::keyPressEvent(QKeyEvent* event)
{
  switch (event->key())
  {
    case Qt::Key_Plus:
      chart()->zoomIn();
      break;
    case Qt::Key_Minus:
      chart()->zoomOut();
      break;
    case Qt::Key_R:
      resetView();
      break;
    case Qt::Key_Left:
      chart()->isZoomed() ? tot_scroll_x_zoomed_ : tot_scroll_x_ -= 10;
      chart()->scroll(-10, 0);
      break;
    case Qt::Key_Right:
      chart()->isZoomed() ? tot_scroll_x_zoomed_ : tot_scroll_x_ += 10;
      chart()->scroll(10, 0);
      break;
    case Qt::Key_Up:
      chart()->isZoomed() ? tot_scroll_y_zoomed_ : tot_scroll_y_ += 10;
      chart()->scroll(0, 10);
      break;
    case Qt::Key_Down:
      chart()->isZoomed() ? tot_scroll_y_zoomed_ : tot_scroll_y_ -= 10;
      chart()->scroll(0, -10);
      break;
    default:
      QGraphicsView::keyPressEvent(event);
      break;
  }
}

void ChartView::mouseDoubleClickEvent(QMouseEvent* event)
{
  if (event->button() == Qt::MouseButton::RightButton)
  {
    resetView();
    return;
  }
  QChartView::mouseDoubleClickEvent(event);
}

void ChartView::mousePressEvent(QMouseEvent* event)
{
  if (event->button() == Qt::MouseButton::RightButton)
  {
    scrolling_ = true;
    prev_pos_  = event->localPos();
    return;
  }
  QChartView::mousePressEvent(event);
}

void ChartView::mouseMoveEvent(QMouseEvent* event)
{
  if (scrolling_)
  {
    QPointF new_pos = event->localPos();
    QPointF delta   = new_pos - prev_pos_;
    chart()->scroll(-delta.x(), delta.y());
    if (chart()->isZoomed())
    {
      tot_scroll_x_zoomed_ -= delta.x();
      tot_scroll_y_zoomed_ += delta.y();
    }
    else
    {
      tot_scroll_x_ -= delta.x();
      tot_scroll_y_ += delta.y();
    }
    prev_pos_ = new_pos;
    return;
  }
  QChartView::mouseMoveEvent(event);
}

void ChartView::mouseReleaseEvent(QMouseEvent* event)
{
  if (event->button() == Qt::MouseButton::RightButton)
  {
    scrolling_ = false;
    return;
  }
  QChartView::mouseReleaseEvent(event);
}

void ChartView::wheelEvent(QWheelEvent* event)
{
  if (event->delta() > 0)
    chart()->zoomIn();
  else if (event->delta() < 0)
    chart()->zoomOut();
}
