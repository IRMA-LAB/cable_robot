#ifndef CALIBRATION_DIALOG_H
#define CALIBRATION_DIALOG_H

#include <QDialog>

#include "libcdpr/inc/types.h"
#include "easylogging++.h"

#include "robot/cablerobot.h"

namespace Ui {
class CalibrationDialog;
}

class CalibrationDialog : public QDialog
{
    Q_OBJECT

public:
    CalibrationDialog(QWidget *parent, CableRobot* robot);
    ~CalibrationDialog();

signals:
    void enableMainGUI();
    void calibrationEnd();

private slots:
    void on_buttonBox_accepted();

    void on_buttonBox_rejected();

private:
    Ui::CalibrationDialog *ui;

    CableRobot* robot_;
};

#endif // CALIBRATION_DIALOG_H
