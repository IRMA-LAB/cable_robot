#ifndef HOMING_INTERFACE_PROPRIOCEPTIVE_H
#define HOMING_INTERFACE_PROPRIOCEPTIVE_H

#include <QFileDialog>
#include <QMessageBox>

#include "gui/homing/homing_interface.h"
#include "homing/homing_proprioceptive.h"

namespace Ui
{
class HomingInterfaceProprioceptive;
}

class HomingInterfaceProprioceptive : public HomingInterface
{
  Q_OBJECT

public:
  explicit HomingInterfaceProprioceptive(QWidget* parent, const grabcdpr::Params* config);
  ~HomingInterfaceProprioceptive();

signals:
  void homingFailed();
  void homingSuccess();

public slots:
  void FaultPresent(const bool);
  void AcquisitionCompleteCb();

private slots:

  void on_pushButton_enable_clicked();
  void on_pushButton_clearFaults_clicked();
  void on_pushButton_start_clicked();
  void on_pushButton_acquire_clicked();

  void on_radioButton_internal_clicked();
  void on_radioButton_external_clicked();
  void on_pushButton_extFile_clicked();
  void on_pushButton_ok_clicked();

  void on_pushButton_cancel_clicked();
  void on_pushButton_done_clicked();

  void AppendText2Browser(const QString& text);

private:
  Ui::HomingInterfaceProprioceptive* ui;

  HomingProprioceptive app_;
};

#endif // HOMING_INTERFACE_PROPRIOCEPTIVE_H
