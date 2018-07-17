#include "hominginterface.h"
#include "ui_hominginterface.h"

HomingInterface::HomingInterface(QWidget* parent, CableRobotMaster* master)
  : QWidget(parent), cable_robot_master_(master), ui(new Ui::HomingInterface)
{
  ui->setupUi(this);
  cable_robot_ = &cable_robot_master_->cable_robot_;
  ui->MainProcessControlBox->setDisabled(true);
  ui->HomingStartButton->setDisabled(true);
  ui->InternaHomingButton->setDisabled(true);
  // ui->LoadExtenalHomingButton->setDisabled(true);
  ui->HomingStopSaveButton->setDisabled(true);

  connect(this, &HomingInterface::GoBackIdle, cable_robot_,
          &CableRobot::CollectRobotRequest);
  connect(this, &HomingInterface::SendClearFaultRequest, cable_robot_,
          &CableRobot::CollectClearFaultRequest);
  connect(this, &HomingInterface::SendEnableRequest, cable_robot_,
          &CableRobot::CollectEnableRequest);
  connect(this, &HomingInterface::SendHomingProcessControl, cable_robot_,
          &CableRobot::CollectHomingProcessControl);
  connect(this, &HomingInterface::SendMeasurementRequest, cable_robot_,
          &CableRobot::CollectMeasurementRequest);
  connect(this, &HomingInterface::SendHomingData, cable_robot_,
          &CableRobot::CollectHomingData);

  connect(cable_robot_, &CableRobot::SendClearFaultRequestProcessed, this,
          &HomingInterface::CollectClearFaultRequestProcessed);
  connect(cable_robot_, &CableRobot::SendEnableRequestProcessed, this,
          &HomingInterface::CollectEnableCommandProcessed);
  connect(cable_robot_, &CableRobot::SendFaultPresentAdvice, this,
          &HomingInterface::CollectFaultPresentAdvice);
  connect(cable_robot_, &CableRobot::SendHomingControl, this,
          &HomingInterface::CollectHomingControl);
  connect(cable_robot_, &CableRobot::SendMeasurement, this,
          &HomingInterface::CollectMeasurements);
  ui->RobotLogBrowser->append("Welcome to the Homing Panel!\n\nPlease enable "
                              "the Robot or clear previous faults before "
                              "starting!\n");
  QDir::setCurrent("/home/labpc/Desktop");
  data_file_.setFileName("homingDataFile.txt");
  if (data_file_.open(QIODevice::WriteOnly))
  {
    ui->RobotLogBrowser->append(
      "Homing data file successfully created in /home/labpc/Desktop");
  }
}

HomingInterface::~HomingInterface() { delete ui; }

void HomingInterface::closeEvent(QCloseEvent* event)
{
  emit GoBackIdle(CableRobot::IDLE);
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

void HomingInterface::on_HomingStopSaveButton_clicked() { data_file_.close(); }

void HomingInterface::on_InternaHomingButton_clicked() {}

void HomingInterface::on_LoadExtenalHomingButton_clicked()
{
  if (data_file_.isOpen())
    data_file_.close();
  QDir::setCurrent("/home/labpc/Desktop");
  data_file_.setFileName("homingResultFile.txt");
  if (data_file_.open(QIODevice::ReadOnly))
  {
    ui->RobotLogBrowser->append("Loading homing data...\n");
  }
  QTextStream in(&data_file_);
  QVector<double> homingData;
  while (!in.atEnd())
  {
    QString line = in.readLine();
    homingData.push_back(line.toDouble());
  }

  data_file_.close();
  emit SendHomingData(homingData);
}

void HomingInterface::CollectFaultPresentAdvice(int theMotor)
{
  ui->RobotLogBrowser->append("Motor " + QString::number(theMotor) +
                              " in fault, please reset.");
}

void HomingInterface::CollectEnableCommandProcessed(int status, int motor)
{
  if (status == SET)
    ui->RobotLogBrowser->append("Motor " + QString::number(motor) + " enabled.");
  if (status == RESET)
    ui->RobotLogBrowser->append("Motor " + QString::number(motor) + " disabled.");
}

void HomingInterface::CollectClearFaultRequestProcessed(int motor)
{
  ui->RobotLogBrowser->append("Motor " + QString::number(motor) + " fault cleared.");
}

void HomingInterface::CollectHomingControl(int state)
{
  if (state)
  {
    ui->HomingStartButton->setEnabled(true);
    ui->InternaHomingButton->setDisabled(true);
    ui->LoadExtenalHomingButton->setDisabled(true);
    ui->HomingStopSaveButton->setDisabled(true);
  }
  else
  {
    ui->HomingStartButton->toggle();
    ui->HomingStartButton->setDisabled(true);
    ui->InternaHomingButton->setEnabled(true);
    ui->LoadExtenalHomingButton->setEnabled(true);
    ui->HomingStopSaveButton->setEnabled(true);
  }
}

void HomingInterface::CollectMeasurements(QVector<double> measurements)
{
  QTextStream out(&data_file_);
  for (int i = 0; i < measurements.size(); i++)
  {
    out << measurements[i] << "\t";
    ui->RobotLogBrowser->append(QString::number(measurements[i], 'f', 12));
  }
  out << endl;
  ui->RobotLogBrowser->append("\n");
}

void HomingInterface::on_HomingStartButton_toggled(bool checked)
{
  if (checked)
  {
    ui->MainProcessControlBox->setEnabled(true);
    emit SendHomingProcessControl(SET);
  }
  else
  {
    ui->MainProcessControlBox->setDisabled(true);
    emit SendHomingProcessControl(RESET);
  }
}
