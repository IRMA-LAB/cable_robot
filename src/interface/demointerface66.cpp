#include "demointerface66.h"
#include "ui_demointerface66.h"

DemoInterface66::DemoInterface66(QWidget *parent, CableRobotMaster *theMaster) :
    QWidget(parent),
    cableRobotMaster(theMaster),
    ui(new Ui::DemoInterface66)
{
    ui->setupUi(this);

    connect(this,&DemoInterface66::GoBackIdle,cableRobotMaster,&CableRobotMaster::CollectMasterRequest);
}

DemoInterface66::~DemoInterface66()
{
    delete ui;
}

void DemoInterface66::closeEvent(QCloseEvent *event)
{
    emit GoBackIdle(CableRobotMaster::idle);
    event->accept();
    delete this;
}
