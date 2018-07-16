#include "actuatorpvtinterface33.h"
#include "ui_actuatorpvtinterface33.h"

ActuatorPvtInterface33::ActuatorPvtInterface33(QWidget *parent, CableRobotMaster *theMaster) :
    QWidget(parent),
    cableRobotMaster(theMaster),
    ui(new Ui::ActuatorPvtInterface33)
{
    ui->setupUi(this);
    cableRobot = &cableRobotMaster->cableRobot;
    ui->StartButton->setDisabled(true);
    ui->ExportDataButton->setDisabled(true);
    connect(this,&ActuatorPvtInterface33::GoBackIdle,cableRobotMaster,&CableRobotMaster::CollectMasterRequest);
    connect(this,&ActuatorPvtInterface33::SendSimulationData,cableRobot,&CableRobot::CollectDataPointers);
    connect(this,&ActuatorPvtInterface33::SendClearFaultRequest,cableRobot,&CableRobot::CollectClearFaultRequest);
    connect(this,&ActuatorPvtInterface33::SendEnableRequest,cableRobot,&CableRobot::CollectEnableRequest);
    connect(this,&ActuatorPvtInterface33::SendStartRequest,cableRobot,&CableRobot::CollectStartRequest);

    connect(cableRobot,&CableRobot::SendData,this,&ActuatorPvtInterface33::CollectData);
    connect(cableRobot,&CableRobot::SendActuatorPvt33Control,this,&ActuatorPvtInterface33::CollectActuatorPvt33Control);
    connect(cableRobot,&CableRobot::SendClearFaultRequestProcessed,this,&ActuatorPvtInterface33::CollectClearFaultRequestProcessed);
    connect(cableRobot,&CableRobot::SendEnableRequestProcessed,this,&ActuatorPvtInterface33::CollectEnableCommandProcessed);
    connect(cableRobot,&CableRobot::SendFaultPresentAdvice,this,&ActuatorPvtInterface33::CollectFaultPresentAdvice);

    QDir::setCurrent("/home/labpc/Desktop");
    dataFile.setFileName("Pvt33Data.txt");
    if (dataFile.open(QIODevice::ReadOnly)) {
        ui->RobotLogBrowser->append("Pvt Data Found");
    }
}

ActuatorPvtInterface33::~ActuatorPvtInterface33()
{
    delete ui;
}

void ActuatorPvtInterface33::closeEvent(QCloseEvent *event)
{
    emit GoBackIdle(CableRobotMaster::idle);
    event->accept();
    delete this;
}


void ActuatorPvtInterface33::on_ImportDataButton_clicked()
{
    if (!dataFile.isOpen()) {
        QDir::setCurrent("/home/labpc/Desktop");
        dataFile.setFileName("Pvt33Data.txt");
        if (dataFile.open(QIODevice::ReadOnly)) {
            QTextStream in(&dataFile);
            QString line = in.readLine();
            int numberOfData = line.toInt()/3;
            int index = 0;
            while(!in.atEnd()) {
                for (int i=0;i<3;i++) {
                   line = in.readLine();
                    cableLengthIn[i].push_back(line.toDouble());
                }
                ui->RobotLogBrowser->append(QString::number(cableLengthIn[0][index]) + "\t" + QString::number(cableLengthIn[1][index]) + "\t" + QString::number(cableLengthIn[2][index]) + "\t" );
                index++;
            }
            dataFile.close();
            emit SendSimulationData(numberOfData,&cableLengthIn[0][0],&cableLengthIn[1][0],&cableLengthIn[2][0]);
        }
    } else {
        QTextStream in(&dataFile);
        QString line = in.readLine();
        int numberOfData = line.toInt()/3;
        int index = 0;
        while(!in.atEnd()) {
            for (int i=0;i<3;i++) {
               line = in.readLine();
                cableLengthIn[i].push_back(line.toDouble());
            }
            ui->RobotLogBrowser->append(QString::number(cableLengthIn[0][index]) + "\t" + QString::number(cableLengthIn[1][index]) + "\t" + QString::number(cableLengthIn[2][index]) + "\t" );
            index++;
        }
        dataFile.close();
        emit SendSimulationData(numberOfData,&cableLengthIn[0][0],&cableLengthIn[1][0],&cableLengthIn[2][0]);
    }


}

void ActuatorPvtInterface33::on_StartButton_toggled(bool checked)
{

}

void ActuatorPvtInterface33::on_ExportDataButton_clicked()
{
    if (dataFile.isOpen()) {
        dataFile.close();
    }
    dataFile.setFileName("pulleyAngles.txt");
    if (dataFile.open(QIODevice::WriteOnly)) {
        ui->RobotLogBrowser->append("Writing Data to File...");
        QTextStream out(&dataFile);
        for (int i=0; i<pulleyAnglesOut[0].size(); i++){
           for (int j=0; j<3;j++) {
               out << pulleyAnglesOut[j][i] << "\t";
           }
           out << endl;
        }
        out << endl;
        dataFile.close();
        ui->RobotLogBrowser->append("Data Saved, You can Quit.");
    } else {
        ui->RobotLogBrowser->append("An error Occurred... Try Again!!");
    }
}

void ActuatorPvtInterface33::on_EnableButton_toggled(bool checked)
{
    emit SendEnableRequest(checked);
}

void ActuatorPvtInterface33::on_ClearFaultsButton_clicked()
{
    emit SendClearFaultRequest();
}

void ActuatorPvtInterface33::CollectFaultPresentAdvice(int theMotor)
{
    ui->RobotLogBrowser->append("Motor " + QString::number(theMotor) + " in fault, please reset.");
}

void ActuatorPvtInterface33::CollectEnableCommandProcessed(int status, int theMotor)
{
    if (status == set) ui->RobotLogBrowser->append("Motor " + QString::number(theMotor) + " enabled.");
    if (status == reset) ui->RobotLogBrowser->append("Motor " + QString::number(theMotor) + " disabled.");
}

void ActuatorPvtInterface33::CollectClearFaultRequestProcessed(int theMotor)
{
    ui->RobotLogBrowser->append("Motor " + QString::number(theMotor) + " fault cleared.");
}

void ActuatorPvtInterface33::CollectActuatorPvt33Control(int state)
{
    switch (state) {
    case 0: {
        ui->StartButton->setDisabled(true);
        ui->ExportDataButton->setDisabled(true);
        break;
    }
    case 1: {
        ui->StartButton->setEnabled(true);
        ui->ExportDataButton->setDisabled(true);
        break;
    }
    case 2: {
        ui->StartButton->setDisabled(true);
        ui->ExportDataButton->setEnabled(true);
    }
    }
}

void ActuatorPvtInterface33::CollectData(double s0, double s1, double s2)
{
    pulleyAnglesOut[0].push_back(s0);
    pulleyAnglesOut[1].push_back(s1);
    pulleyAnglesOut[2].push_back(s2);
}

void ActuatorPvtInterface33::on_StartButton_clicked()
{
    emit SendStartRequest();
}
