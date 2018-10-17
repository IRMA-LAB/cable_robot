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
    HomingDialog(QWidget *parent, QString &config_filename);
    ~HomingDialog();

signals:
    void enableMainGUI(bool);
    void homingFailed();
    void homingSuccess();

private slots:
    void on_buttonBox_accepted();

    void on_buttonBox_rejected();

private:
    Ui::HomingDialog *ui;

    QString config_filename_;
};

#endif // HOMING_DIALOG_H
