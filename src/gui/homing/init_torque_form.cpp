#include "gui/homing/init_torque_form.h"
#include "ui_init_torque_form.h"

InitTorqueForm::InitTorqueForm(const quint8 motor_id, QWidget* parent)
  : QWidget(parent), ui(new Ui::InitTorqueForm)
{
  ui->setupUi(this);

  // Change motor ID in corresponding label
  QString label = ui->label_torqueRange->text();
  label.replace("#", QString("#%1").arg(motor_id));
  ui->label_torqueRange->setText(label);
}

InitTorqueForm::~InitTorqueForm() { delete ui; }

qint16 InitTorqueForm::GetInitTorque() const
{
  return static_cast<qint16>(ui->spinBox_initTorque->value());
}

void InitTorqueForm::SetInitTorque(const qint16 value)
{
  ui->spinBox_initTorque->setValue(value);
}

qint16 InitTorqueForm::GetMaxTorque() const
{
  return static_cast<qint16>(ui->spinBox_maxTorque->value());
}

void InitTorqueForm::EnableInitTorque(const bool value)
{
  ui->spinBox_initTorque->setEnabled(value);
}

void InitTorqueForm::EnableMaxTorque(const bool value)
{
  ui->spinBox_maxTorque->setEnabled(value);
}
