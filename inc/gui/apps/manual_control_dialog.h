#ifndef MANUAL_CONTROL_DIALOG_H
#define MANUAL_CONTROL_DIALOG_H

#include <QDialog>

#include "apps/manual_control_app.h"

namespace Ui {
  class ManualControlDialog;
}

class ManualControlDialog : public QDialog
{
  Q_OBJECT

public:
  explicit ManualControlDialog(QWidget *parent, CableRobot* robot);
  ~ManualControlDialog();

private slots:
  void appendText2Browser(const QString& text);

private slots:
  void on_pushButton_return_clicked();
  void on_pushButton_next_clicked();

private:
  Ui::ManualControlDialog *ui;

  ManualControlApp* app_ = nullptr;
};

#endif // MANUAL_CONTROL_DIALOG_H
