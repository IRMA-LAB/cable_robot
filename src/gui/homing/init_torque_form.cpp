#include "gui/homing/init_torque_form.h"
#include "ui_init_torque_form.h"

InitTorqueForm::InitTorqueForm(const quint8 motor_id, QWidget* parent)
  : QWidget(parent), ui(new Ui::InitTorqueForm)
{
  ui->setupUi(this);

  // Change motor ID in corresponding label
  QString label = ui->label_initTorque->text();
  label.replace("#", QString("#%1").arg(motor_id));
  ui->label_initTorque->setText(label);
}

InitTorqueForm::~InitTorqueForm() { delete ui; }

qint16 InitTorqueForm::GetInitTorque() const
{
  return static_cast<qint16>(ui->spinBox_initTorque->value());
}
