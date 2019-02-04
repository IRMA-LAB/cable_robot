#ifndef INIT_TORQUE_FORM_H
#define INIT_TORQUE_FORM_H

#include <QWidget>

namespace Ui {
class InitTorqueForm;
}

class InitTorqueForm: public QWidget {
  Q_OBJECT

 public:
  InitTorqueForm(const id_t motor_id, QWidget* parent = NULL);
  ~InitTorqueForm();

  void SetInitTorque(const qint16 value);
  void SetMaxTorque(const int value);

  qint16 GetInitTorque() const;
  qint16 GetMaxTorque() const;
  qint16 GetMaxTorqueMinumum() const;

  void EnableInitTorque(const bool value);
  void EnableMaxTorque(const bool value);

 private:
  static constexpr int kTorqueMeasTol_ = 10;

  Ui::InitTorqueForm* ui;
};

#endif // INIT_TORQUE_FORM_H
