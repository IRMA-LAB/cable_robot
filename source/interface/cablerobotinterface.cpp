#include "cablerobotinterface.h"
#include "ui_cablerobotinterface.h"

CableRobotInterface::CableRobotInterface(QWidget* parent, CableRobotMaster* master)
  : QWidget(parent), cable_robot_master_(master), ui(new Ui::CableRobotInterface)
{
  ui->setupUi(this);
  ui->StandardModesBlock->setDisabled(true);
  ui->UserModesBlock->setDisabled(true);

  connect(cable_robot_master_, &CableRobotMaster::SendStartUp, this,
          &CableRobotInterface::CollectStartUp);
  connect(cable_robot_master_, &CableRobotMaster::SendMasterRequestProcessed, this,
          &CableRobotInterface::CollectMasterRequestProcessed);
  connect(&cable_robot_master_->cable_robot_, &CableRobot::SendRobotRequestProcessed,
          this, &CableRobotInterface::CollectRobotRequestProcessed);

  connect(this, &CableRobotInterface::SendMasterRequest, cable_robot_master_,
          &CableRobotMaster::CollectMasterRequest);
  connect(this, &CableRobotInterface::SendMotorNumber, cable_robot_master_,
          &CableRobotMaster::CollectMotorNumber);
  connect(this, &CableRobotInterface::SendRobotRequest,
          &cable_robot_master_->cable_robot_, &CableRobot::CollectRobotRequest);
}

CableRobotInterface::~CableRobotInterface() { delete ui; }

void CableRobotInterface::on_ActuatorControlButton_clicked()
{
  emit SendMotorNumber(ui->MotorSpinBox->value());
  emit SendMasterRequest(CableRobotMaster::ACTUATOR_CTRL);
}

void CableRobotInterface::on_EasyCatButton_clicked()
{
  emit SendMasterRequest(CableRobotMaster::EASYCAT_CTRL);
}

void CableRobotInterface::on_StandardRobotButton_toggled(bool checked)
{
  if (checked)
    emit SendMasterRequest(CableRobotMaster::STD_ROBOT_OPERATION);
  else
    emit SendMasterRequest(CableRobotMaster::IDLE);
}

void CableRobotInterface::on_UserRobotButton_toggled(bool checked)
{
  if (checked)
    emit SendMasterRequest(CableRobotMaster::USER_ROBOT_OPERATION);
  else
    emit SendMasterRequest(CableRobotMaster::IDLE);
}

void CableRobotInterface::on_CalibrationButton_clicked()
{
  emit SendRobotRequest(CableRobot::CALIBRATION);
}

void CableRobotInterface::on_HomingButton_clicked()
{
  emit SendRobotRequest(CableRobot::HOMING);
}

void CableRobotInterface::on_Robot66ManualButton_clicked()
{
  emit SendRobotRequest(CableRobot::ROBOT66MANUAL);
}

void CableRobotInterface::on_Robot66DemoButton_clicked()
{
  emit SendRobotRequest(CableRobot::ROBOT66DEMO);
}

void CableRobotInterface::on_Robot33ActuatorPvtButton_clicked()
{
  emit SendRobotRequest(CableRobot::ROBOT33ACTUATOR_PVT);
}

void CableRobotInterface::on_Robot33AutomaticButton_clicked()
{
  emit SendRobotRequest(CableRobot::ROBOT33AUTOMATIC);
}

void CableRobotInterface::on_Robot33ManualButton_clicked()
{
  emit SendRobotRequest(CableRobot::ROBOT33MANUAL);
}

void CableRobotInterface::CollectStartUp()
{
  ui->MasterLogBrowser->append("Master Has Started");
}

void CableRobotInterface::CollectMasterRequestProcessed(int state)
{
  switch (state)
  {
  case CableRobotMaster::ACTUATOR_CTRL:
  {
    ActuatorInterface* actuatorInterface =
      new ActuatorInterface(nullptr, cable_robot_master_);
    actuatorInterface->show();
    ui->StandardModesBlock->setDisabled(true);
    ui->UserModesBlock->setDisabled(true);
    this->setDisabled(true);
    break;
  }
  case CableRobotMaster::EASYCAT_CTRL:
  {
    ui->StandardModesBlock->setDisabled(true);
    ui->UserModesBlock->setDisabled(true);
    // this->hide();
    break;
  }
  case CableRobotMaster::STD_ROBOT_OPERATION:
  {
    ui->StandardModesBlock->setEnabled(true);
    ui->UserModesBlock->setDisabled(true);
    ui->UserRobotButton->setDisabled(true);
    ui->EasyCatButton->setDisabled(true);
    ui->ActuatorControlButton->setDisabled(true);
    ui->MotorSpinBox->setDisabled(true);
    break;
  }
  case CableRobotMaster::USER_ROBOT_OPERATION:
  {
    ui->UserModesBlock->setEnabled(true);
    ui->StandardModesBlock->setDisabled(true);
    ui->StandardRobotButton->setDisabled(true);
    ui->EasyCatButton->setDisabled(true);
    ui->ActuatorControlButton->setDisabled(true);
    ui->MotorSpinBox->setDisabled(true);
    break;
  }
  default:
  {
    this->setEnabled(true);
    ui->StandardModesBlock->setDisabled(true);
    ui->UserModesBlock->setDisabled(true);
    ui->StandardRobotButton->setEnabled(true);
    ui->UserRobotButton->setEnabled(true);
    ui->EasyCatButton->setEnabled(true);
    ui->ActuatorControlButton->setEnabled(true);
    ui->MotorSpinBox->setEnabled(true);
    break;
  }
  }
}

void CableRobotInterface::CollectRobotRequestProcessed(int state)
{
  switch (state)
  {
  case CableRobot::CALIBRATION:
  {
    CalibrationInterface* calibration_interface =
      new CalibrationInterface(nullptr, cable_robot_master_);
    calibration_interface->show();
    break;
  }
  case CableRobot::HOMING:
  {
    HomingInterface* homing_interface = new HomingInterface(nullptr, cable_robot_master_);
    homing_interface->show();
    this->setDisabled(true);
    break;
  }
  case CableRobot::ROBOT33ACTUATOR_PVT:
  {
    ActuatorPvtInterface33* actuator_pvt_interface33 =
      new ActuatorPvtInterface33(nullptr, cable_robot_master_);
    actuator_pvt_interface33->show();
    break;
  }
  case CableRobot::ROBOT33AUTOMATIC:
  {
    DemoInterface33* demo_interface33 = new DemoInterface33(nullptr, cable_robot_master_);
    demo_interface33->show();
    break;
  }
  case CableRobot::ROBOT33MANUAL:
  {
    ManualInterface33* manual_interface33 =
      new ManualInterface33(nullptr, cable_robot_master_);
    manual_interface33->show();
    break;
  }
  case CableRobot::ROBOT66MANUAL:
  {
    ManualInterface66* manual_interface66 =
      new ManualInterface66(nullptr, cable_robot_master_);
    manual_interface66->show();
    break;
  }
  case CableRobot::ROBOT66DEMO:
  {
    DemoInterface66* demo_interface66 = new DemoInterface66(nullptr, cable_robot_master_);
    demo_interface66->show();
    break;
  }
  default:
  {
    if (!this->isEnabled())
      this->setEnabled(true);
    break;
  }
  }
}
