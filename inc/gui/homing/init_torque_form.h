#ifndef INIT_TORQUE_FORM_H
#define INIT_TORQUE_FORM_H

#include <QWidget>

namespace Ui
{
class InitTorqueForm;
}

class InitTorqueForm : public QWidget
{
  Q_OBJECT

public:
  explicit InitTorqueForm(const quint8 motor_id, QWidget* parent = 0);
  ~InitTorqueForm();

  qint16 GetInitTorque() const;

private:
  Ui::InitTorqueForm* ui;
};

#endif // INIT_TORQUE_FORM_H
