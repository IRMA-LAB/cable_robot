#include "cablerobotinterface.h"
#include "ui_cablerobotinterface.h"

CableRobotInterface::CableRobotInterface(QWidget* parent,
                                         CableRobotMaster* theMaster)
  : QWidget(parent), cableRobotMaster(theMaster),
    ui(new Ui::CableRobotInterface)
{
  ui->setupUi(this);
  ui->StandardModesBlock->setDisabled(true);
  ui->UserModesBlock->setDisabled(true);

  connect(cableRobotMaster, &CableRobotMaster::SendStartUp, this,
          &CableRobotInterface::CollectStartUp);
  connect(cableRobotMaster, &CableRobotMaster::SendMasterRequestProcessed, this,
          &CableRobotInterface::CollectMasterRequestProcessed);
  connect(&cableRobotMaster->cableRobot, &CableRobot::SendRobotRequestProcessed,
          this, &CableRobotInterface::CollectRobotRequestProcessed);

  connect(this, &CableRobotInterface::SendMasterRequest, cableRobotMaster,
          &CableRobotMaster::CollectMasterRequest);
  connect(this, &CableRobotInterface::SendMotorNumber, cableRobotMaster,
          &CableRobotMaster::CollectMotorNumber);
  connect(this, &CableRobotInterface::SendRobotRequest,
          &cableRobotMaster->cableRobot, &CableRobot::CollectRobotRequest);
}

CableRobotInterface::~CableRobotInterface() { delete ui; }

void CableRobotInterface::on_ActuatorControlButton_clicked()
{
  emit SendMotorNumber(ui->MotorSpinBox->value());
  emit SendMasterRequest(CableRobotMaster::actuatorControl);
}

void CableRobotInterface::on_EasyCatButton_clicked()
{
  emit SendMasterRequest(CableRobotMaster::easyCatControl);
}

void CableRobotInterface::on_StandardRobotButton_toggled(bool checked)
{
  if (checked)
    emit SendMasterRequest(CableRobotMaster::standardRobotOperation);
  else
    emit SendMasterRequest(CableRobotMaster::idle);
}

void CableRobotInterface::on_UserRobotButton_toggled(bool checked)
{
  if (checked)
    emit SendMasterRequest(CableRobotMaster::userRobotOperation);
  else
    emit SendMasterRequest(CableRobotMaster::idle);
}

void CableRobotInterface::on_CalibrationButton_clicked()
{
  emit SendRobotRequest(CableRobot::calibration);
}

void CableRobotInterface::on_HomingButton_clicked()
{
  emit SendRobotRequest(CableRobot::homing);
}

void CableRobotInterface::on_Robot66ManualButton_clicked()
{
  emit SendRobotRequest(CableRobot::robot66Manual);
}

void CableRobotInterface::on_Robot66DemoButton_clicked()
{
  emit SendRobotRequest(CableRobot::robot66Demo);
}

void CableRobotInterface::on_Robot33ActuatorPvtButton_clicked()
{
  emit SendRobotRequest(CableRobot::robot33ActuatorPvt);
}

void CableRobotInterface::on_Robot33AutomaticButton_clicked()
{
  emit SendRobotRequest(CableRobot::robot33Automatic);
}

void CableRobotInterface::on_Robot33ManualButton_clicked()
{
  emit SendRobotRequest(CableRobot::robot33Manual);
}

void CableRobotInterface::CollectStartUp()
{
  ui->MasterLogBrowser->append("Master Has Started");
}

void CableRobotInterface::CollectMasterRequestProcessed(int state)
{
  switch (state)
  {
  case CableRobotMaster::actuatorControl:
  {
    ActuatorInterface* actuatorInterface =
      new ActuatorInterface(nullptr, cableRobotMaster);
    actuatorInterface->show();
    ui->StandardModesBlock->setDisabled(true);
    ui->UserModesBlock->setDisabled(true);
    this->setDisabled(true);
    break;
  }
  case CableRobotMaster::easyCatControl:
  {
    ui->StandardModesBlock->setDisabled(true);
    ui->UserModesBlock->setDisabled(true);
    // this->hide();
    break;
  }
  case CableRobotMaster::standardRobotOperation:
  {
    ui->StandardModesBlock->setEnabled(true);
    ui->UserModesBlock->setDisabled(true);
    ui->UserRobotButton->setDisabled(true);
    ui->EasyCatButton->setDisabled(true);
    ui->ActuatorControlButton->setDisabled(true);
    ui->MotorSpinBox->setDisabled(true);
    break;
  }
  case CableRobotMaster::userRobotOperation:
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
  case CableRobot::calibration:
  {
    CalibrationInterface* calirationInterface =
      new CalibrationInterface(nullptr, cableRobotMaster);
    calirationInterface->show();
    break;
  }
  case CableRobot::homing:
  {
    HomingInterface* homingInterface =
      new HomingInterface(nullptr, cableRobotMaster);
    homingInterface->show();
    this->setDisabled(true);
    break;
  }
  case CableRobot::robot33ActuatorPvt:
  {
    ActuatorPvtInterface33* actuatorPvtInterface33 =
      new ActuatorPvtInterface33(nullptr, cableRobotMaster);
    actuatorPvtInterface33->show();
    break;
  }
  case CableRobot::robot33Automatic:
  {
    DemoInterface33* demoInterface33 =
      new DemoInterface33(nullptr, cableRobotMaster);
    demoInterface33->show();
    break;
  }
  case CableRobot::robot33Manual:
  {
    ManualInterface33* manualInterface33 =
      new ManualInterface33(nullptr, cableRobotMaster);
    manualInterface33->show();
    break;
  }
  case CableRobot::robot66Manual:
  {
    ManualInterface66* manualInterface66 =
      new ManualInterface66(nullptr, cableRobotMaster);
    manualInterface66->show();
    break;
  }
  case CableRobot::robot66Demo:
  {
    DemoInterface66* demoInterface66 =
      new DemoInterface66(nullptr, cableRobotMaster);
    demoInterface66->show();
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
