#ifndef CALIBRATION_DIALOG_H
#define CALIBRATION_DIALOG_H

#include <QDialog>

namespace Ui {
class CalibrationDialog;
}

class CalibrationDialog : public QDialog
{
    Q_OBJECT

public:
    CalibrationDialog(QWidget *parent = 0);
    ~CalibrationDialog();

private slots:

private:
    Ui::CalibrationDialog *ui;
};

#endif // CALIBRATION_DIALOG_H
