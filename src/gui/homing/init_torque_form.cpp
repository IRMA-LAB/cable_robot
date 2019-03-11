/**
 * @file init_torque_form.cpp
 * @author Simone Comari
 * @date 11 Mar 2019
 * @brief File containing definitions of widget class declared in init_torque_form.h.
 */

#include "gui/homing/init_torque_form.h"
#include "ui_init_torque_form.h"

InitTorqueForm::InitTorqueForm(const id_t motor_id, QWidget* parent)
  : QWidget(parent), ui(new Ui::InitTorqueForm)
{
  ui->setupUi(this);

  // Change motor ID in corresponding label
  QString label = ui->label_torqueRange->text();
  label.replace("#", QString("#%1").arg(motor_id));
  ui->label_torqueRange->setText(label);
}

InitTorqueForm::~InitTorqueForm() { delete ui; }

void InitTorqueForm::SetInitTorque(const int value)
{
  ui->spinBox_initTorque->setValue(value);
  // Update maximum torque minimum value (max torque >= init torque)
  ui->spinBox_maxTorque->setMaximum(value - kTorqueMeasTol_); // pulling torque < 0
}

void InitTorqueForm::SetMaxTorque(const int value)
{
  ui->spinBox_maxTorque->setValue(std::min(value, ui->spinBox_maxTorque->maximum()));
}

qint16 InitTorqueForm::GetInitTorque() const
{
  return static_cast<qint16>(ui->spinBox_initTorque->value());
}

qint16 InitTorqueForm::GetMaxTorque() const
{
  return static_cast<qint16>(ui->spinBox_maxTorque->value());
}

qint16 InitTorqueForm::GetMaxTorqueMinumum() const
{
  return static_cast<qint16>(ui->spinBox_maxTorque->maximum());
}

void InitTorqueForm::EnableInitTorque(const bool value)
{
  ui->spinBox_initTorque->setEnabled(value);
}

void InitTorqueForm::EnableMaxTorque(const bool value)
{
  ui->spinBox_maxTorque->setEnabled(value);
}
