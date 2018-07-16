#include "hominginterface.h"
#include "ui_hominginterface.h"

HomingInterface::HomingInterface(QWidget *parent, CableRobotMaster *theMaster) :
    QWidget(parent),
    cableRobotMaster(theMaster),
    ui(new Ui::HomingInterface)
{
    ui->setupUi(this);
    cableRobot = &cableRobotMaster->cableRobot;
    ui->MainProcessControlBox->setDisabled(true);
    ui->HomingStartButton->setDisabled(true);
    ui->InternaHomingButton->setDisabled(true);
    //ui->LoadExtenalHomingButton->setDisabled(true);
    ui->HomingStopSaveButton->setDisabled(true);

    connect(this,&HomingInterface::GoBackIdle,cableRobot,&CableRobot::CollectRobotRequest);
    connect(this,&HomingInterface::SendClearFaultRequest,cableRobot,&CableRobot::CollectClearFaultRequest);
    connect(this,&HomingInterface::SendEnableRequest,cableRobot,&CableRobot::CollectEnableRequest);
    connect(this,&HomingInterface::SendHomingProcessControl,cableRobot,&CableRobot::CollectHomingProcessControl);
    connect(this,&HomingInterface::SendMeasurementRequest,cableRobot,&CableRobot::CollectMeasurementRequest);
    connect(this,&HomingInterface::SendHomingData,cableRobot,&CableRobot::CollectHomingData);

    connect(cableRobot,&CableRobot::SendClearFaultRequestProcessed,this,&HomingInterface::CollectClearFaultRequestProcessed);
    connect(cableRobot,&CableRobot::SendEnableRequestProcessed,this,&HomingInterface::CollectEnableCommandProcessed);
    connect(cableRobot,&CableRobot::SendFaultPresentAdvice,this,&HomingInterface::CollectFaultPresentAdvice);
    connect(cableRobot,&CableRobot::SendHomingControl,this,&HomingInterface::CollectHomingControl);
    connect(cableRobot,&CableRobot::SendMeasurement,this,&HomingInterface::CollectMeasurements);
    ui->RobotLogBrowser->append("Welcome to the Homing Panel!\n\nPlease enable the Robot or clear previous faults before starting!\n");
    QDir::setCurrent("/home/labpc/Desktop");
    dataFile.setFileName("homingDataFile.txt");
    if (dataFile.open(QIODevice::WriteOnly)) {
        ui->RobotLogBrowser->append("Homing data file successfully created in /home/labpc/Desktop");
    }
}

HomingInterface::~HomingInterface()
{
    delete ui;
}

void HomingInterface::closeEvent(QCloseEvent *event)
{
    emit GoBackIdle(CableRobot::idle);
    event->accept();
    delete this;
}

void HomingInterface::on_HomingEnableButton_toggled(bool checked)
{
    emit SendEnableRequest(checked);
}

void HomingInterface::on_HomingClearFaultsButton_clicked()
{
    emit SendClearFaultRequest();
}

void HomingInterface::on_HomingAcquireDataButton_clicked()
{
    emit SendMeasurementRequest();
}

void HomingInterface::on_HomingStopSaveButton_clicked()
{
    dataFile.close();
}

void HomingInterface::on_InternaHomingButton_clicked()
{

}

void HomingInterface::on_LoadExtenalHomingButton_clicked()
{
    if (dataFile.isOpen()) dataFile.close();
    QDir::setCurrent("/home/labpc/Desktop");
    dataFile.setFileName("homingResultFile.txt");
    if (dataFile.open(QIODevice::ReadOnly)) {
        ui->RobotLogBrowser->append("Loading homing data...\n");
    }
    QTextStream in(&dataFile);
    QVector<double> homingData;
    while(!in.atEnd()) {
        QString line = in.readLine();
        homingData.push_back(line.toDouble());
    }

    dataFile.close();
    emit SendHomingData(homingData);

}

void HomingInterface::CollectFaultPresentAdvice(int theMotor)
{
    ui->RobotLogBrowser->append("Motor " + QString::number(theMotor) + " in fault, please reset.");
}

void HomingInterface::CollectEnableCommandProcessed(int status, int theMotor)
{
    if (status == set) ui->RobotLogBrowser->append("Motor " + QString::number(theMotor) + " enabled.");
    if (status == reset) ui->RobotLogBrowser->append("Motor " + QString::number(theMotor) + " disabled.");
}

void HomingInterface::CollectClearFaultRequestProcessed(int theMotor)
{
    ui->RobotLogBrowser->append("Motor " + QString::number(theMotor) + " fault cleared.");
}

void HomingInterface::CollectHomingControl(int state)
{
    if (state) {
        ui->HomingStartButton->setEnabled(true);
        ui->InternaHomingButton->setDisabled(true);
        ui->LoadExtenalHomingButton->setDisabled(true);
        ui->HomingStopSaveButton->setDisabled(true);
    } else {
        ui->HomingStartButton->toggle();
        ui->HomingStartButton->setDisabled(true);
        ui->InternaHomingButton->setEnabled(true);
        ui->LoadExtenalHomingButton->setEnabled(true);
        ui->HomingStopSaveButton->setEnabled(true);
    }
}

void HomingInterface::CollectMeasurements(QVector<double> measurements)
{
    QTextStream out(&dataFile);
    for (int i=0; i<measurements.size(); i++) {
        out << measurements[i] << "\t";
        ui->RobotLogBrowser->append(QString::number(measurements[i],'f',12));
    }
    out << endl;
    ui->RobotLogBrowser->append("\n");
}

void HomingInterface::on_HomingStartButton_toggled(bool checked)
{
    if (checked) {
        ui->MainProcessControlBox->setEnabled(true);
        emit SendHomingProcessControl(set);
    } else {
        ui->MainProcessControlBox->setDisabled(true);
        emit SendHomingProcessControl(reset);
    }
}
