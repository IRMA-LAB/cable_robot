#include "manualinterface33.h"
#include "ui_manualinterface33.h"

ManualInterface33::ManualInterface33(QWidget* parent, CableRobotMaster* master)
  : QWidget(parent), cable_robot_master_(master), ui(new Ui::ManualInterface33)
{
  ui->setupUi(this);

  connect(this, &ManualInterface33::GoBackIdle, cable_robot_master_,
          &CableRobotMaster::CollectMasterRequest);
}

ManualInterface33::~ManualInterface33() { delete ui; }

void ManualInterface33::closeEvent(QCloseEvent* event)
{
  emit GoBackIdle(CableRobotMaster::IDLE);
  event->accept();
  delete this;
}
