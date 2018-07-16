#include "calibrationinterface.h"
#include "ui_calibrationinterface.h"

CalibrationInterface::CalibrationInterface(QWidget *parent, CableRobotMaster *theMaster) :
    QWidget(parent),
    cableRobotMaster(theMaster),
    ui(new Ui::CalibrationInterface)
{
    ui->setupUi(this);

    connect(this,&CalibrationInterface::GoBackIdle,cableRobotMaster,&CableRobotMaster::CollectMasterRequest);
}

CalibrationInterface::~CalibrationInterface()
{
    delete ui;
}

void CalibrationInterface::closeEvent(QCloseEvent *event)
{
    emit GoBackIdle(CableRobotMaster::idle);
    event->accept();
    delete this;
}
