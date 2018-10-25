#ifndef HOMING_DIALOG_H
#define HOMING_DIALOG_H

#include <QDialog>

#include "libcdpr/inc/types.h"

namespace Ui {
class HomingDialog;
}

class HomingDialog : public QDialog
{
    Q_OBJECT

public:
    HomingDialog(QWidget *parent, const grabcdpr::Params& config);
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

    const grabcdpr::Params& config_;
};

#endif // HOMING_DIALOG_H
