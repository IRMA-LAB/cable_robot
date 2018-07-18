#include "actuatorinterface.h"
#include "ui_actuatorinterface.h"

#include <iostream>
using namespace std;

ActuatorInterface::ActuatorInterface(QWidget* parent, CableRobotMaster* master)
  : QWidget(parent), cable_robot_master_(master), ui(new Ui::ActuatorInterface)
{
  ui->setupUi(this);
  ui->FaultResetButton->setDisabled(true);
  ui->CyclicPositionBox->setDisabled(true);
  ui->CyclicTorqueBox->setDisabled(true);
  ui->CyclicVelocityBox->setDisabled(true);
  for (uint8_t i = 0; i < GoldSoloWhistleDrive::kGoldSoloWhistleDomainInputs_; i++)
    ui->InputPdosTable->setItem(i, 0, &input_items_[i]);

  connect(this, &ActuatorInterface::GoBackIdle, cable_robot_master_,
          &CableRobotMaster::CollectMasterRequest);
  connect(this, &ActuatorInterface::SendClearFaultRequest, cable_robot_master_,
          &CableRobotMaster::CollectClearFaultRequest);
  connect(this, &ActuatorInterface::SendCommandUpdateRequest, cable_robot_master_,
          &CableRobotMaster::CollectCommandUpdateRequest);
  connect(this, &ActuatorInterface::SendEnableRequest, cable_robot_master_,
          &CableRobotMaster::CollectEnableRequest);
  connect(this, &ActuatorInterface::SendOperationModeChangeRequest, cable_robot_master_,
          &CableRobotMaster::CollectOperationModeChangeRequest);

  connect(cable_robot_master_, &CableRobotMaster::SendClearFaultRequestProcessed, this,
          &ActuatorInterface::CollectClearFaultRequestProcessed);
  connect(cable_robot_master_, &CableRobotMaster::SendCommandUpdateRequestProcessed, this,
          &ActuatorInterface::CollectCommandUpdateRequestProcessed);
  connect(cable_robot_master_, &CableRobotMaster::SendEnableRequestProcessed, this,
          &ActuatorInterface::CollectEnableCommandProcessed);
  connect(cable_robot_master_, &CableRobotMaster::SendFaultPresentAdvice, this,
          &ActuatorInterface::CollectFaultPresentAdvice);
  connect(cable_robot_master_, &CableRobotMaster::SendOperationModeChangeRequestProcessed,
          this, &ActuatorInterface::CollectOperationModeChangeRequestProcessed);
  connect(cable_robot_master_, &CableRobotMaster::SendGuiData, this,
          &ActuatorInterface::CollectGuiData);
}

ActuatorInterface::~ActuatorInterface() { delete ui; }

void ActuatorInterface::on_EnableButton_toggled(bool checked)
{
  emit SendEnableRequest(checked);
}

void ActuatorInterface::on_FaultResetButton_clicked() { emit SendClearFaultRequest(); }

void ActuatorInterface::on_PositionEnable_toggled(bool checked)
{
  if (checked)
    emit SendOperationModeChangeRequest(GoldSoloWhistleDrive::CYCLIC_POSITION);
}

void ActuatorInterface::on_TorqueEnable_toggled(bool checked)
{
  if (checked)
    emit SendOperationModeChangeRequest(GoldSoloWhistleDrive::CYCLIC_TORQUE);
}

void ActuatorInterface::on_VelocityEnable_toggled(bool checked)
{
  if (checked)
    emit SendOperationModeChangeRequest(GoldSoloWhistleDrive::CYCLIC_VELOCITY);
}

void ActuatorInterface::on_MovePlusBut_pressed()
{
  if (ui->PositionEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::POS_MOVE_UP, SET);
}

void ActuatorInterface::on_MovePlusBut_released()
{
  if (ui->PositionEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::POS_MOVE_UP, RESET);
}

void ActuatorInterface::on_MoveMinusButton_pressed()
{
  if (ui->PositionEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::POS_MOVE_DOWN, SET);
}

void ActuatorInterface::on_MoveMinusButton_released()
{
  if (ui->PositionEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::POS_MOVE_DOWN, RESET);
}

void ActuatorInterface::on_MicroMovePlusButton_pressed()
{
  if (ui->PositionEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::POS_MICRO_MOVE_UP, SET);
}

void ActuatorInterface::on_MicroMovePlusButton_released()
{
  if (ui->PositionEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::POS_MICRO_MOVE_UP, RESET);
}

void ActuatorInterface::on_MicroMoveMinusButton_pressed()
{
  if (ui->PositionEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::POS_MICRO_MOVE_DOWN, SET);
}

void ActuatorInterface::on_MicroMoveMinusButton_released()
{
  if (ui->PositionEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::POS_MICRO_MOVE_DOWN, RESET);
}

void ActuatorInterface::on_TorquePlusButton_pressed()
{
  if (ui->TorqueEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::TORQUE_UP, SET);
}

void ActuatorInterface::on_TorquePlusButton_released()
{
  if (ui->TorqueEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::TORQUE_UP, RESET);
}

void ActuatorInterface::on_TorqueMinusButton_pressed()
{
  if (ui->TorqueEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::TORQUE_DOWN, SET);
}

void ActuatorInterface::on_TorqueMinusButton_released()
{
  if (ui->TorqueEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::TORQUE_DOWN, RESET);
}

void ActuatorInterface::on_SpeedPlusButton_pressed()
{
  if (ui->VelocityEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::SPEED_UP, SET);
}

void ActuatorInterface::on_SpeedPlusButton_released()
{
  if (ui->VelocityEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::SPEED_UP, RESET);
}

void ActuatorInterface::on_SpeedMinusButton_pressed()
{
  if (ui->VelocityEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::SPEED_DOWN, SET);
}

void ActuatorInterface::on_SpeedMinusButton_released()
{
  if (ui->VelocityEnable->isChecked())
    emit SendCommandUpdateRequest(ServoMotor::SPEED_DOWN, RESET);
}

void ActuatorInterface::CollectEnableCommandProcessed(int status)
{
  if (status == SET)
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
  {
    ui->EnableButton->toggle();
  }
  ui->FaultResetButton->setDisabled(true);
}

void ActuatorInterface::CollectOperationModeChangeRequestProcessed(int op_mode)
{
  switch (op_mode)
  {
    case GoldSoloWhistleDrive::CYCLIC_POSITION:
    {
      if (ui->TorqueEnable->isChecked())
        ui->TorqueEnable->setChecked(false);
      else if (ui->VelocityEnable->isChecked())
        ui->VelocityEnable->setChecked(false);
      break;
    }
    case GoldSoloWhistleDrive::CYCLIC_TORQUE:
    {
      if (ui->PositionEnable->isChecked())
        ui->PositionEnable->setChecked(false);
      else if (ui->VelocityEnable->isChecked())
        ui->VelocityEnable->setChecked(false);
      break;
    }
    case GoldSoloWhistleDrive::CYCLIC_VELOCITY:
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

void ActuatorInterface::CollectCommandUpdateRequestProcessed(int command, int status)
{
  cout << "Command " << command << " is set to " << status << "." << endl;
}

void ActuatorInterface::CollectGuiData(GoldSoloWhistleDrive::InputPdos* pdos, int n)
{
  switch (n)
  {
    case GoldSoloWhistleDrive::kStatusWordElement:
    {
      // ui->InputPdosTable->item(0,n)->setData(0,(quint16)thePdos->statusWord.to_ulong());
      input_items_[n].setData(0, static_cast<qint16>(pdos->status_word.to_ulong()));
      break;
    }
    case GoldSoloWhistleDrive::kModesOfOperationElement:
    {
      // ui->InputPdosTable->item(0,n)->setData(0,(qint8)thePdos->modesOfOperationDisplay);
      input_items_[n].setData(0, static_cast<qint8>(pdos->modes_of_operation_display));
      break;
    }
    case GoldSoloWhistleDrive::kPositionActualvalueElement:
    {
      // ui->InputPdosTable->item(0,n)->setData(0,(qint32)thePdos->positionActualValue);
      input_items_[n].setData(0, static_cast<qint32>(pdos->position_actual_value));
      break;
    }
    case GoldSoloWhistleDrive::kVelocityActualvalueElement:
    {
      // ui->InputPdosTable->item(0,n)->setData(0,(qint32)thePdos->velocityActualValue);
      input_items_[n].setData(0, static_cast<qint32>(pdos->velocity_actual_value));
      break;
    }
    case GoldSoloWhistleDrive::kTorqueActualValueElement:
    {
      // ui->InputPdosTable->item(0,n)->setData(0,(qint16)thePdos->torqueActualValue);
      input_items_[n].setData(0, static_cast<qint16>(pdos->torque_actual_value));
      break;
    }
    case GoldSoloWhistleDrive::kDigitalInputsElement:
    {
      // ui->InputPdosTable->item(0,n)->setData(0,(quint32)thePdos->digitalInputs);
      input_items_[n].setData(0, static_cast<qint32>(pdos->digital_inputs));
      break;
    }
    case GoldSoloWhistleDrive::kAuxiliaryPositionActualValueElement:
    {
      // ui->InputPdosTable->item(0,n)->setData(0,(qint32)thePdos->auxiliaryPositionActualValue);
      input_items_[n].setData(0, static_cast<qint32>(pdos->auxiliary_position_actual_value));
      break;
    }
  }
}

void ActuatorInterface::closeEvent(QCloseEvent* event)
{
  emit GoBackIdle(CableRobotMaster::IDLE);
  event->accept();
  delete this;
}
