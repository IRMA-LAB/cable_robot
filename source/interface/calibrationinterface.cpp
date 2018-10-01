#include "calibrationinterface.h"
#include "ui_calibrationinterface.h"

CalibrationInterface::CalibrationInterface(QWidget* parent, CableRobotMaster* master)
  : QWidget(parent), cable_robot_master_(master), ui(new Ui::CalibrationInterface)
{
  ui->setupUi(this);

  connect(this, &CalibrationInterface::GoBackIdle, cable_robot_master_,
          &CableRobotMaster::CollectMasterRequest);
}

CalibrationInterface::~CalibrationInterface() { delete ui; }

void CalibrationInterface::closeEvent(QCloseEvent* event)
{
  emit GoBackIdle(CableRobotMaster::IDLE);
  event->accept();
  delete this;
}
