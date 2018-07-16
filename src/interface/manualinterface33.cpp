#include "manualinterface33.h"
#include "ui_manualinterface33.h"

ManualInterface33::ManualInterface33(QWidget *parent, CableRobotMaster *theMaster) :
    QWidget(parent),
    cableRobotMaster(theMaster),
    ui(new Ui::ManualInterface33)
{
    ui->setupUi(this);

    connect(this,&ManualInterface33::GoBackIdle,cableRobotMaster,&CableRobotMaster::CollectMasterRequest);

}

ManualInterface33::~ManualInterface33()
{
    delete ui;
}

void ManualInterface33::closeEvent(QCloseEvent *event)
{
    emit GoBackIdle(CableRobotMaster::idle);
    event->accept();
    delete this;
}
