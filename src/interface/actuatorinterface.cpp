#include "actuatorinterface.h"
#include "ui_actuatorinterface.h"

#include <iostream>
using namespace std;

ActuatorInterface::ActuatorInterface(QWidget* parent,
                                     CableRobotMaster* theMaster)
  : QWidget(parent), cableRobotMaster(theMaster), ui(new Ui::ActuatorInterface)
{
  ui->setupUi(this);
  ui->FaultResetButton->setDisabled(true);
  ui->CyclicPositionBox->setDisabled(true);
  ui->CyclicTorqueBox->setDisabled(true);
  ui->CyclicVelocityBox->setDisabled(true);
  for (int i = 0; i < GoldSoloWhistleDrive::GoldSoloWhistleDomainInputs; i++)
    ui->InputPdosTable->setItem(i, 0, &inputItems[i]);

  connect(this, &ActuatorInterface::GoBackIdle, cableRobotMaster,
          &CableRobotMaster::CollectMasterRequest);

  connect(this, &ActuatorInterface::SendClearFaultRequest, cableRobotMaster,
          &CableRobotMaster::CollectClearFaultRequest);
  connect(this, &ActuatorInterface::SendCommandUpdateRequest, cableRobotMaster,
          &CableRobotMaster::CollectCommandUpdateRequest);
  connect(this, &ActuatorInterface::SendEnableRequest, cableRobotMaster,
          &CableRobotMaster::CollectEnableRequest);
  connect(this, &ActuatorInterface::SendOperationModeChangeRequest,
          cableRobotMaster,
          &CableRobotMaster::CollectOperationModeChangeRequest);

  connect(cableRobotMaster, &CableRobotMaster::SendClearFaultRequestProcessed,
          this, &ActuatorInterface::CollectClearFaultRequestProcessed);
  connect(cableRobotMaster,
          &CableRobotMaster::SendCommandUpdateRequestProcessed, this,
          &ActuatorInterface::CollectCommandUpdateRequestProcessed);
  connect(cableRobotMaster, &CableRobotMaster::SendEnableRequestProcessed, this,
          &ActuatorInterface::CollectEnableCommandProcessed);
  connect(cableRobotMaster, &CableRobotMaster::SendFaultPresentAdvice, this,
          &ActuatorInterface::CollectFaultPresentAdvice);
  connect(cableRobotMaster,
          &CableRobotMaster::SendOperationModeChangeRequestProcessed, this,
          &ActuatorInterface::CollectOperationModeChangeRequestProcessed);
  connect(cableRobotMaster, &CableRobotMaster::SendGuiData, this,
          &ActuatorInterface::CollectGuiData);
}

ActuatorInterface::~ActuatorInterface() { delete ui; }

void ActuatorInterface::on_EnableButton_toggled(bool checked)
{
  emit SendEnableRequest(checked);
}

void ActuatorInterface::on_FaultResetButton_clicked()
{
  emit SendClearFaultRequest();
}

void ActuatorInterface::on_PositionEnable_toggled(bool checked)
{
  if (checked)
  {
    emit SendOperationModeChangeRequest(GoldSoloWhistleDrive::cyclicPosition);
  }
}

void ActuatorInterface::on_TorqueEnable_toggled(bool checked)
{
  if (checked)
  {
    emit SendOperationModeChangeRequest(GoldSoloWhistleDrive::cyclicTorque);
  }
}

void ActuatorInterface::on_VelocityEnable_toggled(bool checked)
{
  if (checked)
  {
    emit SendOperationModeChangeRequest(GoldSoloWhistleDrive::cyclicVelocity);
  }
}

void ActuatorInterface::on_MovePlusBut_pressed()
{
  if (ui->PositionEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::PositionMovePlus, set);
}

void ActuatorInterface::on_MovePlusBut_released()
{
  if (ui->PositionEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::PositionMovePlus, reset);
}

void ActuatorInterface::on_MoveMinusButton_pressed()
{
  if (ui->PositionEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::PositionMoveMinus, set);
}

void ActuatorInterface::on_MoveMinusButton_released()
{
  if (ui->PositionEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::PositionMoveMinus, reset);
}

void ActuatorInterface::on_MicroMovePlusButton_pressed()
{
  if (ui->PositionEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::PositionMicroMovePlus, set);
}

void ActuatorInterface::on_MicroMovePlusButton_released()
{
  if (ui->PositionEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::PositionMicroMovePlus, reset);
}

void ActuatorInterface::on_MicroMoveMinusButton_pressed()
{
  if (ui->PositionEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::PositionMicroMoveMinus, set);
}

void ActuatorInterface::on_MicroMoveMinusButton_released()
{
  if (ui->PositionEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::PositionMicroMoveMinus, reset);
}

void ActuatorInterface::on_TorquePlusButton_pressed()
{
  if (ui->TorqueEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::TorquePlus, set);
}

void ActuatorInterface::on_TorquePlusButton_released()
{
  if (ui->TorqueEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::TorquePlus, reset);
}

void ActuatorInterface::on_TorqueMinusButton_pressed()
{
  if (ui->TorqueEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::TorqueMinus, set);
}

void ActuatorInterface::on_TorqueMinusButton_released()
{
  if (ui->TorqueEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::TorqueMinus, reset);
}

void ActuatorInterface::on_SpeedPlusButton_pressed()
{
  if (ui->VelocityEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::SpeedPlus, set);
}

void ActuatorInterface::on_SpeedPlusButton_released()
{
  if (ui->VelocityEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::SpeedPlus, reset);
}

void ActuatorInterface::on_SpeedMinusButton_pressed()
{
  if (ui->VelocityEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::SpeedMinus, set);
}

void ActuatorInterface::on_SpeedMinusButton_released()
{
  if (ui->VelocityEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::SpeedMinus, reset);
}

void ActuatorInterface::CollectEnableCommandProcessed(int status)
{
  if (status == set)
  {
    ui->CyclicPositionBox->setEnabled(true);
    ui->CyclicTorqueBox->setEnabled(true);
    ui->CyclicVelocityBox->setEnabled(true);
  }
  else
  {
    ui->CyclicPositionBox->setDisabled(true);
    ui->CyclicTorqueBox->setDisabled(true);
    ui->CyclicVelocityBox->setDisabled(true);
  }
}

void ActuatorInterface::CollectFaultPresentAdvice()
{
  ui->CyclicPositionBox->setDisabled(true);
  ui->CyclicTorqueBox->setDisabled(true);
  ui->CyclicVelocityBox->setDisabled(true);
  ui->FaultResetButton->setEnabled(true);
  ui->EnableButton->setDisabled(true);
}

void ActuatorInterface::CollectClearFaultRequestProcessed()
{
  ui->EnableButton->setEnabled(true);
  if (ui->EnableButton->isChecked())
    ui->EnableButton->toggle();
  ui->FaultResetButton->setDisabled(true);
}

void ActuatorInterface::CollectOperationModeChangeRequestProcessed(
  int modeOfOperation)
{
  switch (modeOfOperation)
  {
  case GoldSoloWhistleDrive::cyclicPosition:
  {
    if (ui->TorqueEnable->isChecked())
      ui->TorqueEnable->setChecked(false);
    else if (ui->VelocityEnable->isChecked())
      ui->VelocityEnable->setChecked(false);
    break;
  }
  case GoldSoloWhistleDrive::cyclicTorque:
  {
    if (ui->PositionEnable->isChecked())
      ui->PositionEnable->setChecked(false);
    else if (ui->VelocityEnable->isChecked())
      ui->VelocityEnable->setChecked(false);
    break;
  }
  case GoldSoloWhistleDrive::cyclicVelocity:
  {
    if (ui->TorqueEnable->isChecked())
      ui->TorqueEnable->setChecked(false);
    else if (ui->PositionEnable->isChecked())
      ui->PositionEnable->setChecked(false);
    break;
  }
  default:
    break;
  }
}

void ActuatorInterface::CollectCommandUpdateRequestProcessed(int command,
                                                             int status)
{
  cout << "Command " << command << " is set to " << status << "." << endl;
}

void ActuatorInterface::CollectGuiData(GoldSoloWhistleDrive::InputPdos* thePdos,
                                       int n)
{
  switch (n)
  {
  case GoldSoloWhistleDrive::statusWordElement:
  {
    // ui->InputPdosTable->item(0,n)->setData(0,(quint16)thePdos->statusWord.to_ulong());
    inputItems[n].setData(0, (quint16)thePdos->statusWord.to_ulong());
    break;
  }
  case GoldSoloWhistleDrive::modesOfOperationElement:
  {
    // ui->InputPdosTable->item(0,n)->setData(0,(qint8)thePdos->modesOfOperationDisplay);
    inputItems[n].setData(0, (qint8)thePdos->modesOfOperationDisplay);
    break;
  }
  case GoldSoloWhistleDrive::positionActualvalueElement:
  {
    // ui->InputPdosTable->item(0,n)->setData(0,(qint32)thePdos->positionActualValue);
    inputItems[n].setData(0, (qint32)thePdos->positionActualValue);
    break;
  }
  case GoldSoloWhistleDrive::velocityActualvalueElement:
  {
    // ui->InputPdosTable->item(0,n)->setData(0,(qint32)thePdos->velocityActualValue);
    inputItems[n].setData(0, (qint32)thePdos->velocityActualValue);
    break;
  }
  case GoldSoloWhistleDrive::torqueActualValueElement:
  {
    // ui->InputPdosTable->item(0,n)->setData(0,(qint16)thePdos->torqueActualValue);
    inputItems[n].setData(0, (qint16)thePdos->torqueActualValue);
    break;
  }
  case GoldSoloWhistleDrive::digitalInputsElement:
  {
    // ui->InputPdosTable->item(0,n)->setData(0,(quint32)thePdos->digitalInputs);
    inputItems[n].setData(0, (quint32)thePdos->digitalInputs);
    break;
  }
  case GoldSoloWhistleDrive::auxiliaryPositionActualValueElement:
  {
    // ui->InputPdosTable->item(0,n)->setData(0,(qint32)thePdos->auxiliaryPositionActualValue);
    inputItems[n].setData(0, (qint32)thePdos->auxiliaryPositionActualValue);
    break;
  }
  }
}

void ActuatorInterface::closeEvent(QCloseEvent* event)
{
  emit GoBackIdle(CableRobotMaster::idle);
  event->accept();
  delete this;
}
