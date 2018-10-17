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
    CalibrationDialog(QWidget *parent, QString &config_filename);
    ~CalibrationDialog();

signals:
    void enableMainGUI();
    void calibrationEnd();

private slots:
    void on_buttonBox_accepted();

    void on_buttonBox_rejected();

private:
    Ui::CalibrationDialog *ui;

    QString config_filename_;
};

#endif // CALIBRATION_DIALOG_H
