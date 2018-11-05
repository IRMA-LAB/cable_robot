#ifndef CALIBRATION_DIALOG_H
#define CALIBRATION_DIALOG_H

#include <QDialog>

#include "libcdpr/inc/types.h"

namespace Ui {
class CalibrationDialog;
}

class CalibrationDialog : public QDialog
{
    Q_OBJECT

public:
    CalibrationDialog(QWidget *parent, const grabcdpr::Params& config);
    ~CalibrationDialog();

signals:
    void enableMainGUI();
    void calibrationEnd();

private slots:
    void on_buttonBox_accepted();

    void on_buttonBox_rejected();

private:
    Ui::CalibrationDialog *ui;

    const grabcdpr::Params config_;
};

#endif // CALIBRATION_DIALOG_H
