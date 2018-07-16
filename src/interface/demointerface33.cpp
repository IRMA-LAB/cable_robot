#include "demointerface33.h"
#include "ui_demointerface33.h"

DemoInterface33::DemoInterface33(QWidget *parent, CableRobotMaster *theMaster) :
    QWidget(parent),
    cableRobotMaster(theMaster),
    ui(new Ui::DemoInterface33)
{
    ui->setupUi(this);

    connect(this,&DemoInterface33::GoBackIdle,cableRobotMaster,&CableRobotMaster::CollectMasterRequest);
}

DemoInterface33::~DemoInterface33()
{
    delete ui;
}

void DemoInterface33::closeEvent(QCloseEvent *event)
{
    emit GoBackIdle(CableRobotMaster::idle);
    event->accept();
    delete this;
}
