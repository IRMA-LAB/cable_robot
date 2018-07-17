#include "manualinterface66.h"
#include "ui_manualinterface66.h"

ManualInterface66::ManualInterface66(QWidget* parent, CableRobotMaster* theMaster)
  : QWidget(parent), cableRobotMaster(theMaster), ui(new Ui::ManualInterface66)
{
  ui->setupUi(this);

  connect(this, &ManualInterface66::GoBackIdle, cableRobotMaster,
          &CableRobotMaster::CollectMasterRequest);
}

ManualInterface66::~ManualInterface66() { delete ui; }

void ManualInterface66::closeEvent(QCloseEvent* event)
{
  emit GoBackIdle(CableRobotMaster::idle);
  event->accept();
  delete this;
}
