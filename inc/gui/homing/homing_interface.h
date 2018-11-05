#ifndef HOMING_INTERFACE_H
#define HOMING_INTERFACE_H

#include <QDialog>

#include "libcdpr/inc/types.h"

class HomingInterface : public QDialog
{
  Q_OBJECT

public:
  explicit HomingInterface(QWidget* parent, const grabcdpr::Params* config)
    : QDialog(parent), config_ptr_(config) {}
  virtual ~HomingInterface() {}

signals:
  void homingFailed();
  void homingSuccess();

protected:
  const grabcdpr::Params* config_ptr_;
};

#endif // HOMING_INTERFACE_H
