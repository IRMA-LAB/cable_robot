#ifndef HOMING_DIALOG_H
#define HOMING_DIALOG_H

#include <QDialog>

namespace Ui {
class HomingDialog;
}

class HomingDialog : public QDialog
{
    Q_OBJECT

public:
    HomingDialog(QWidget *parent = 0);
    ~HomingDialog();

private slots:

private:
    Ui::HomingDialog *ui;
};

#endif // HOMING_DIALOG_H
