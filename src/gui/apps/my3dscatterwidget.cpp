#include "gui/apps/my3dscatterwidget.h"
#include "ui_my3dscatterwidget.h"

static QVector3D randVector()
{
  return QVector3D((float)(QRandomGenerator::global()->bounded(100)) / 2.0f -
                     (float)(QRandomGenerator::global()->bounded(100)) / 2.0f,
                   (float)(QRandomGenerator::global()->bounded(100)) / 2.0f -
                     (float)(QRandomGenerator::global()->bounded(100)) / 2.0f,
                   (float)(QRandomGenerator::global()->bounded(100)) / 2.0f -
                     (float)(QRandomGenerator::global()->bounded(100)) / 2.0f);
}

My3DScatterWidget::My3DScatterWidget(QWidget* parent)
  : QWidget(parent), ui(new Ui::My3DScatterWidget)
{
  ui->setupUi(this);

  graph_ = new Q3DScatter();
  graph_->activeTheme()->setType(Q3DTheme::ThemeArmyBlue);
  QFont font = graph_->activeTheme()->font();
  font.setPointSize(40.0f);
  graph_->activeTheme()->setFont(font);
  graph_->setShadowQuality(QAbstract3DGraph::ShadowQualityNone);
  graph_->scene()->activeCamera()->setCameraPreset(Q3DCamera::CameraPresetIsometricRight);
  QScatterDataProxy* proxy = new QScatterDataProxy;
  QScatter3DSeries* series = new QScatter3DSeries(proxy);
  series->setItemLabelFormat(
    QStringLiteral("@xTitle: @xLabel @zTitle: @zLabel @yTitle: @yLabel"));
  series->setMeshSmooth(true);
  series->setMesh(QAbstract3DSeries::MeshPoint);
  series->setItemSize(0.0f);
  graph_->addSeries(series);
  graph_->axisX()->setTitle("X [m]");
  graph_->axisX()->setTitleVisible(true);
  graph_->axisY()->setTitle("Z [m]");
  graph_->axisY()->setTitleVisible(true);
  graph_->axisZ()->setTitle("Y [m]");
  graph_->axisZ()->setTitleVisible(true);
  input_handler_ = new CustomInputHandler(graph_);
  graph_->setActiveInputHandler(input_handler_);

  QWidget* container = QWidget::createWindowContainer(graph_);
  QSize screenSize   = graph_->screen()->size();
  container->setMinimumSize(QSize(screenSize.width() / 3.5, screenSize.height() / 2.5));
  container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  container->setSizeIncrement(1, 1);
  container->setFocusPolicy(Qt::StrongFocus);

  ui->verticalLayout->insertWidget(0, container, 1);

  connect(graph_, &Q3DScatter::widthChanged, input_handler_,
          &CustomInputHandler::setWidgetWidth);
  connect(graph_, &Q3DScatter::heightChanged, input_handler_,
          &CustomInputHandler::setWidgetHeight);
}

My3DScatterWidget::~My3DScatterWidget() { delete ui; }

void My3DScatterWidget::setTrajectory(const vect<TrajectoryD>& trajectory)
{
  float data_min_f = 1e10;
  float data_max_f = -1e10;
  int m_itemCount  = 900;

  QScatterDataArray* dataArray = new QScatterDataArray;
  dataArray->resize(m_itemCount);
  QScatterDataItem* ptrToDataArray = &dataArray->first();
  for (int i = 0; i < m_itemCount; i++)
  {
    QVector3D data_point = randVector();
    ptrToDataArray->setPosition(data_point);
    ptrToDataArray++;

    float point_min = qMin(data_point.x(), qMin(data_point.y(), data_point.z()));
    data_min_f      = qMin(data_min_f, point_min);
    float point_max = qMax(data_point.x(), qMax(data_point.y(), data_point.z()));
    data_max_f      = qMax(data_max_f, point_max);
  }

  int data_min = qFloor(static_cast<double>(data_min_f));
  int data_max = qCeil(static_cast<double>(data_max_f));
  ui->horizontalSlider_xmin->setRange(data_min, data_max);
  ui->horizontalSlider_ymin->setRange(data_min, data_max);
  ui->horizontalSlider_zmin->setRange(data_min, data_max);
  ui->horizontalSlider_xmin->setValue(data_min);
  ui->horizontalSlider_ymin->setValue(data_min);
  ui->horizontalSlider_zmin->setValue(data_min);

  ui->horizontalSlider_xmax->setRange(data_min, data_max);
  ui->horizontalSlider_ymax->setRange(data_min, data_max);
  ui->horizontalSlider_zmax->setRange(data_min, data_max);
  ui->horizontalSlider_xmax->setValue(data_min);
  ui->horizontalSlider_ymax->setValue(data_min);
  ui->horizontalSlider_zmax->setValue(data_min);

  graph_->seriesList().at(0)->dataProxy()->resetArray(dataArray);
  graph_->axisX()->setRange(data_min_f, data_max_f);
  graph_->axisY()->setRange(data_min_f, data_max_f);
  graph_->axisZ()->setRange(data_min_f, data_max_f);
  graph_->setAspectRatio(1.0);
  graph_->setHorizontalAspectRatio(1.0);
  graph_->scene()->activeCamera()->setZoomLevel(140);
}

void My3DScatterWidget::on_horizontalSlider_zmin_valueChanged(int value)
{
  graph_->axisY()->setMin(value);
}

void My3DScatterWidget::on_horizontalSlider_ymin_valueChanged(int value)
{
  graph_->axisZ()->setMin(value);
}

void My3DScatterWidget::on_horizontalSlider_xmin_valueChanged(int value)
{
  graph_->axisX()->setMin(value);
}

void My3DScatterWidget::on_horizontalSlider_zmax_valueChanged(int value)
{
  graph_->axisY()->setMax(ui->horizontalSlider_ymax->maximum() -
                          (value - ui->horizontalSlider_ymax->minimum()));
}

void My3DScatterWidget::on_horizontalSlider_ymax_valueChanged(int value)
{
  graph_->axisZ()->setMax(ui->horizontalSlider_zmax->maximum() -
                          (value - ui->horizontalSlider_zmax->minimum()));
}

void My3DScatterWidget::on_horizontalSlider_xmax_valueChanged(int value)
{
  graph_->axisX()->setMax(ui->horizontalSlider_xmax->maximum() -
                          (value - ui->horizontalSlider_xmax->minimum()));
}


CustomInputHandler::CustomInputHandler(QObject* parent)
  : Q3DInputHandler(parent), pressed_(false)
{}

CustomInputHandler::CustomInputHandler(QObject* parent, const QSize& parent_widget_size)
  : Q3DInputHandler(parent), pressed_(false)
{
  setWidgetSize(parent_widget_size);
}

void CustomInputHandler::setWidgetSize(const QSize& size) { widget_size_ = size; }

void CustomInputHandler::setWidgetWidth(const int& width)
{
  widget_size_.setWidth(width);
}

void CustomInputHandler::setWidgetHeight(const int& height)
{
  widget_size_.setHeight(height);
}

void CustomInputHandler::mousePressEvent(QMouseEvent* event, const QPoint& mousePos)
{
  Q3DInputHandler::mousePressEvent(event, mousePos);
  if (event->button() == Qt::LeftButton)
  {
    prev_target_ = scene()->activeCamera()->target();
    start_pos_   = mousePos;
    pressed_     = true;
  }
}

void CustomInputHandler::mouseReleaseEvent(QMouseEvent* event, const QPoint& mousePos)
{
  Q3DInputHandler::mouseReleaseEvent(event, mousePos);
  if (event->button() == Qt::LeftButton)
    pressed_ = false;
}

void CustomInputHandler::mouseMoveEvent(QMouseEvent* event, const QPoint& mousePos)
{
  Q3DInputHandler::mouseMoveEvent(event, mousePos);
  if (!pressed_)
    return;

  // Scale mouse motion in 2D normalized vector.
  float du =
    2 * (mousePos.x() - start_pos_.x()) / static_cast<float>(widget_size_.width());
  float dv =
    2 * (mousePos.y() - start_pos_.y()) / static_cast<float>(widget_size_.height());
  // Target motion is opposite to mouse one.
  // Note: image y-axis points points downwards.
  grabnum::Vector3f view_vec({-du, dv, 0});

  // Build rotation matrix from fixed camera frame (XY plane parallel to image plane) to
  // moving plot frame using RPY.
  // Note: x and y rotation are swapped here (xRotation = pitch, yRotation = roll).
  grabnum::Matrix3f R =
    grabgeom::RPY2Rot(qDegreesToRadians(scene()->activeCamera()->yRotation()),
                      qDegreesToRadians(scene()->activeCamera()->xRotation()), 0.0);

  // Transform target motion into plot frame.
  grabnum::Vector3f camera_target = R.Transpose() * view_vec;

  // Actual plot frame is not standard: Z frame is opposite to cartesian rule.
  scene()->activeCamera()->setTarget(
    QVector3D(camera_target(1), camera_target(2), -camera_target(3)) + prev_target_);
}
