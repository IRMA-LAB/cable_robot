#include "demointerface33.h"
#include "ui_demointerface33.h"

DemoInterface33::DemoInterface33(QWidget* parent, CableRobotMaster* master)
  : QWidget(parent), cable_robot_master_(master), ui(new Ui::DemoInterface33)
{
  ui->setupUi(this);

  connect(this, &DemoInterface33::GoBackIdle, cable_robot_master_,
          &CableRobotMaster::CollectMasterRequest);
}

DemoInterface33::~DemoInterface33() { delete ui; }

void DemoInterface33::closeEvent(QCloseEvent* event)
{
  emit GoBackIdle(CableRobotMaster::IDLE);
  event->accept();
  delete this;
}
