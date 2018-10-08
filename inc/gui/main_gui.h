#ifndef MAIN_GUI_H
#define MAIN_GUI_H

#include <QDialog>

#include "calibration_dialog.h"
#include "homing_dialog.h"

namespace Ui {
class MainGUI;
}

class MainGUI : public QDialog
{
    Q_OBJECT

public:
    MainGUI(QWidget *parent, QString & config_filename);
    ~MainGUI();

private slots:
    void on_pushButton_calib_clicked();

    void on_pushButton_homing_clicked();

    void on_pushButton_startApp_clicked();

    void on_pushButton_enable_clicked();
    void on_pushButton_faultReset_clicked();

private:
    Ui::MainGUI *ui;
    CalibrationDialog *calib_dialog;
    HomingDialog *homing_dialog;

    QString config_filename_;
};

#endif // MAIN_GUI_H
