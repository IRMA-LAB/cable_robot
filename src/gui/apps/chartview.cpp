#include "gui/apps/chartview.h"

ChartView::ChartView(QChart* chart, QWidget* parent) : QChartView(chart, parent)
{
  setRenderHint(QPainter::Antialiasing);
  setRubberBand(QChartView::RectangleRubberBand);
  setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
  setSizeIncrement(1, 1);
  setFocusPolicy(Qt::StrongFocus);

  chart->setAnimationOptions(QChart::SeriesAnimations);
  chart->legend()->hide();
}

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
      chart()->scroll(-tot_scroll_x_zoomed_, -tot_scroll_y_zoomed_);
      chart()->zoomReset();
      chart()->scroll(-tot_scroll_x_, -tot_scroll_y_);
      tot_scroll_x_ = 0;
      tot_scroll_y_ = 0;
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
}
