#include "demointerface66.h"
#include "ui_demointerface66.h"

DemoInterface66::DemoInterface66(QWidget* parent, CableRobotMaster* master)
  : QWidget(parent), cable_robot_master_(master), ui(new Ui::DemoInterface66)
{
  ui->setupUi(this);

  connect(this, &DemoInterface66::GoBackIdle, cable_robot_master_,
          &CableRobotMaster::CollectMasterRequest);
}

DemoInterface66::~DemoInterface66() { delete ui; }

void DemoInterface66::closeEvent(QCloseEvent* event)
{
  emit GoBackIdle(CableRobotMaster::IDLE);
  event->accept();
  delete this;
}
