#include "manualinterface66.h"
#include "ui_manualinterface66.h"

ManualInterface66::ManualInterface66(QWidget* parent, CableRobotMaster* master)
  : QWidget(parent), cable_robot_master_(master), ui(new Ui::ManualInterface66)
{
  ui->setupUi(this);

  connect(this, &ManualInterface66::GoBackIdle, cable_robot_master_,
          &CableRobotMaster::CollectMasterRequest);
}

ManualInterface66::~ManualInterface66() { delete ui; }

void ManualInterface66::closeEvent(QCloseEvent* event)
{
  emit GoBackIdle(CableRobotMaster::IDLE);
  event->accept();
  delete this;
}
