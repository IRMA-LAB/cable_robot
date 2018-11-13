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

  void SetInitTorque(const qint16 value);

  qint16 GetInitTorque() const;
  qint16 GetMaxTorque() const;

  void EnableInitTorque(const bool value);
  void EnableMaxTorque(const bool value);

private:
  Ui::InitTorqueForm* ui;
};

#endif // INIT_TORQUE_FORM_H
