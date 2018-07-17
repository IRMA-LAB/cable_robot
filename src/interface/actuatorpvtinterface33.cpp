#include "actuatorpvtinterface33.h"
#include "ui_actuatorpvtinterface33.h"

ActuatorPvtInterface33::ActuatorPvtInterface33(QWidget* parent,
                                               CableRobotMaster* master)
  : QWidget(parent), cable_robot_master_(master), ui(new Ui::ActuatorPvtInterface33)
{
  ui->setupUi(this);
  cable_robot_ = &cable_robot_master_->cable_robot_;
  ui->StartButton->setDisabled(true);
  ui->ExportDataButton->setDisabled(true);
  connect(this, &ActuatorPvtInterface33::GoBackIdle, cable_robot_master_,
          &CableRobotMaster::CollectMasterRequest);
  connect(this, &ActuatorPvtInterface33::SendSimulationData, cable_robot_,
          &CableRobot::CollectDataPointers);
  connect(this, &ActuatorPvtInterface33::SendClearFaultRequest, cable_robot_,
          &CableRobot::CollectClearFaultRequest);
  connect(this, &ActuatorPvtInterface33::SendEnableRequest, cable_robot_,
          &CableRobot::CollectEnableRequest);
  connect(this, &ActuatorPvtInterface33::SendStartRequest, cable_robot_,
          &CableRobot::CollectStartRequest);

  connect(cable_robot_, &CableRobot::SendData, this, &ActuatorPvtInterface33::CollectData);
  connect(cable_robot_, &CableRobot::SendActuatorPvt33Control, this,
          &ActuatorPvtInterface33::CollectActuatorPvt33Control);
  connect(cable_robot_, &CableRobot::SendClearFaultRequestProcessed, this,
          &ActuatorPvtInterface33::CollectClearFaultRequestProcessed);
  connect(cable_robot_, &CableRobot::SendEnableRequestProcessed, this,
          &ActuatorPvtInterface33::CollectEnableCommandProcessed);
  connect(cable_robot_, &CableRobot::SendFaultPresentAdvice, this,
          &ActuatorPvtInterface33::CollectFaultPresentAdvice);

  QDir::setCurrent("/home/labpc/Desktop");
  data_file_.setFileName("Pvt33Data.txt");
  if (data_file_.open(QIODevice::ReadOnly))
  {
    ui->RobotLogBrowser->append("Pvt Data Found");
  }
}

ActuatorPvtInterface33::~ActuatorPvtInterface33() { delete ui; }

void ActuatorPvtInterface33::closeEvent(QCloseEvent* event)
{
  emit GoBackIdle(CableRobotMaster::IDLE);
  event->accept();
  delete this;
}

void ActuatorPvtInterface33::on_ImportDataButton_clicked()
{
  if (!data_file_.isOpen())
  {
    QDir::setCurrent("/home/labpc/Desktop");
    data_file_.setFileName("Pvt33Data.txt");
    if (data_file_.open(QIODevice::ReadOnly))
    {
      QTextStream in(&data_file_);
      QString line = in.readLine();
      int numberOfData = line.toInt() / 3;
      int index = 0;
      while (!in.atEnd())
      {
        for (int i = 0; i < 3; i++)
        {
          line = in.readLine();
          cable_len_in_[i].push_back(line.toDouble());
        }
        ui->RobotLogBrowser->append(QString::number(cable_len_in_[0][index]) + "\t" +
                                    QString::number(cable_len_in_[1][index]) + "\t" +
                                    QString::number(cable_len_in_[2][index]) + "\t");
        index++;
      }
      data_file_.close();
      emit SendSimulationData(numberOfData, &cable_len_in_[0][0], &cable_len_in_[1][0],
                              &cable_len_in_[2][0]);
    }
  }
  else
  {
    QTextStream in(&data_file_);
    QString line = in.readLine();
    int numberOfData = line.toInt() / 3;
    int index = 0;
    while (!in.atEnd())
    {
      for (int i = 0; i < 3; i++)
      {
        line = in.readLine();
        cable_len_in_[i].push_back(line.toDouble());
      }
      ui->RobotLogBrowser->append(QString::number(cable_len_in_[0][index]) + "\t" +
                                  QString::number(cable_len_in_[1][index]) + "\t" +
                                  QString::number(cable_len_in_[2][index]) + "\t");
      index++;
    }
    data_file_.close();
    emit SendSimulationData(numberOfData, &cable_len_in_[0][0], &cable_len_in_[1][0],
                            &cable_len_in_[2][0]);
  }
}

void ActuatorPvtInterface33::on_StartButton_toggled(bool /*checked*/) {}

void ActuatorPvtInterface33::on_ExportDataButton_clicked()
{
  if (data_file_.isOpen())
  {
    data_file_.close();
  }
  data_file_.setFileName("pulleyAngles.txt");
  if (data_file_.open(QIODevice::WriteOnly))
  {
    ui->RobotLogBrowser->append("Writing Data to File...");
    QTextStream out(&data_file_);
    for (int i = 0; i < pulley_angles_out_[0].size(); i++)
    {
      for (int j = 0; j < 3; j++)
      {
        out << pulley_angles_out_[j][i] << "\t";
      }
      out << endl;
    }
    out << endl;
    data_file_.close();
    ui->RobotLogBrowser->append("Data Saved, You can Quit.");
  }
  else
  {
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
  ui->RobotLogBrowser->append("Motor " + QString::number(theMotor) +
                              " in fault, please reset.");
}

void ActuatorPvtInterface33::CollectEnableCommandProcessed(int status, int motor)
{
  if (status == SET)
    ui->RobotLogBrowser->append("Motor " + QString::number(motor) + " enabled.");
  if (status == RESET)
    ui->RobotLogBrowser->append("Motor " + QString::number(motor) + " disabled.");
}

void ActuatorPvtInterface33::CollectClearFaultRequestProcessed(int motor)
{
  ui->RobotLogBrowser->append("Motor " + QString::number(motor) + " fault cleared.");
}

void ActuatorPvtInterface33::CollectActuatorPvt33Control(int state)
{
  switch (state)
  {
  case 0:
  {
    ui->StartButton->setDisabled(true);
    ui->ExportDataButton->setDisabled(true);
    break;
  }
  case 1:
  {
    ui->StartButton->setEnabled(true);
    ui->ExportDataButton->setDisabled(true);
    break;
  }
  case 2:
  {
    ui->StartButton->setDisabled(true);
    ui->ExportDataButton->setEnabled(true);
  }
  }
}

void ActuatorPvtInterface33::CollectData(double s0, double s1, double s2)
{
  pulley_angles_out_[0].push_back(s0);
  pulley_angles_out_[1].push_back(s1);
  pulley_angles_out_[2].push_back(s2);
}

void ActuatorPvtInterface33::on_StartButton_clicked() { emit SendStartRequest(); }
