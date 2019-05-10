#ifndef INPUT_FORM_H
#define INPUT_FORM_H

#include <QWidget>
#include <QFileDialog>
#include <QMessageBox>

#include "easylogging++.h"

namespace Ui {
  class InputForm;
}

class InputForm : public QWidget
{
  Q_OBJECT

public:
  explicit InputForm(QWidget *parent = 0);
  ~InputForm();

  bool isInputEmpty() const;
  QString getFilepath() const;

private slots:
  void on_pushButton_fileSelection_clicked();

private:
  Ui::InputForm *ui;
};

#endif // INPUT_FORM_H
